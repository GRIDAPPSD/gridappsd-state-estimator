#define DIAGONAL_P
#define DEBUG_PRIMARY
//#define DEBUG_FILES
#define GS_OPTIMIZE

#define PI 3.1415926535

// negligable macros
//#define USE_NEGL

#ifdef USE_NEGL
#define NEGL 1.0e-16
#ifdef GS_OPTIMIZE
#define gs_entry_diagonal_negl(A,ij,val) if (val>NEGL || -val>NEGL) gs_entry_diagonal(A,ij,val)
#define gs_entry_firstcol_negl(A,i,val) if (val>NEGL || -val>NEGL) gs_entry_firstcol(A,i,val)
#define gs_entry_fullsquare_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_fullsquare(A,i,j,val)
#define gs_entry_colorder_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_colorder(A,i,j,val)
#else
#define cs_entry_negl(A,i,j,val) if (val>NEGL || -val>NEGL) cs_entry(A,i,j,val)
#endif
#else
#ifdef GS_OPTIMIZE
#define gs_entry_diagonal_negl(A,ij,val) gs_entry_diagonal(A,ij,val)
#define gs_entry_firstcol_negl(A,i,val) gs_entry_firstcol(A,i,val)
#define gs_entry_fullsquare_negl(A,i,j,val) gs_entry_fullsquare(A,i,j,val)
#define gs_entry_colorder_negl(A,i,j,val) gs_entry_colorder(A,i,j,val)
#else
#define cs_entry_negl(A,i,j,val) cs_entry(A,i,j,val)
#endif
#endif

#include <iostream>

// global logging stream--either a file or stdout based on invocation
std::ostream* selog = &std::cout;

#include "state_estimator_gridappsd.hpp"
#include "SEProducer.hpp"
#include "GenericConsumer.hpp"
#include "TopoProcConsumer.hpp"
#include "VnomConsumer.hpp"
#include "SensorDefConsumer.hpp"
#include "SharedQueue.hpp"
#include "SELoopConsumer.hpp"
#include "SELoopWorker.hpp"

#include "gridappsd_requests.hpp"
using gridappsd_requests::sparql_query;

////#include "sparql_queries.hpp"
//#include "sparql_queries_CIM100.hpp"
//using sparql_queries::sparq_nodes;
//using sparql_queries::sparq_conducting_equipment_vbase;
//using sparql_queries::sparq_transformer_end_vbase;
//using sparql_queries::sparq_energy_consumer_pq;
//using sparql_queries::sparq_ratio_tap_changer_nodes;

#include "state_estimator_util.hpp"

#include "State.hpp"
#include "SensorArray.hpp"

// standard data types
#include <string>
#include <complex>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// Store node names in a linked list and hash node name to their position
// Iterate over the linked list to access all nodes or states
// Note that positions are one-indexed
// Store sensor names in a linked list and hash node names to various params
#define SLIST std::list<std::string>
#define SIMAP std::unordered_map<std::string,unsigned int>
#define SDMAP std::unordered_map<std::string,double>
#define SCMAP std::unordered_map<std::string,std::complex<double>>
#define SSMAP std::unordered_map<std::string,std::string>

// reverse lookup of nodenames by index
#define ISMAP std::unordered_map<unsigned int,std::string>

// Hash address (i,j) to the index of a sparse matrix vector
#define ICMAP std::unordered_map<unsigned int,std::complex<double>>
#define IMMAP std::unordered_map<unsigned int,ICMAP>

// Store x and z in a list one-indexed by position
#define IDMAP std::unordered_map<unsigned int,double>
#define IMDMAP std::unordered_map<unsigned int,IDMAP>

#ifdef DEBUG_PRIMARY
// temporary flag to hold up initialization until the platform has finished
// its own initialization for the simulation based on producing measurements
bool blockedFlag = true;
#endif

int main(int argc, char** argv) {
	
	// ------------------------------------------------------------------------
	// INITIALIZE THE STATE ESTIMATOR SESSION WITH RUNTIME ARGS
	// ------------------------------------------------------------------------
	state_estimator_gridappsd::state_estimator_session se;
	if ( !se.init(argc,argv) ) return 0;

	// ------------------------------------------------------------------------
	// INITIALIZE THE GRIDAPPS SESSION
	// ------------------------------------------------------------------------
	state_estimator_gridappsd::gridappsd_session gad(se);

    // declare the thread-safe queue shared between SELoopConsumer (writer)
    // and SELoopWorker (reader)
    SharedQueue<json> workQueue;

	// ------------------------------------------------------------------------
	// START THE AMQ INTERFACE
	// ------------------------------------------------------------------------

	try {
		activemq::library::ActiveMQCPP::initializeLibrary();

		// --------------------------------------------------------------------
		// LISTEN FOR SIMULATION LOG MESSAGES
		// --------------------------------------------------------------------

		// measurements come from the simulation output
		string simlogTopic = "goss.gridappsd.simulation.log."+gad.simid;

		SELoopConsumer simLogConsumer(&workQueue, gad.brokerURI, gad.username,
            gad.password, simlogTopic, "topic");
		Thread simLogConsumerThread(&simLogConsumer);
		simLogConsumerThread.start();	// execute simLogConsumer.run()
		simLogConsumer.waitUntilReady();	// wait for the startup latch release

#ifdef DEBUG_PRIMARY
        // determine whether to write to a log file or stdout based on whether
        // this is a platform vs. command line invocation
        static std::ofstream logfile;
        if (gad.stateEstimatorFromPlatformFlag) {
            logfile.open("/tmp/state-estimator.log");
            selog = &logfile;
        }

		*selog << "\nListening for simulation log messages on "+simlogTopic+'\n' << std::flush;
#endif

		// --------------------------------------------------------------------
		// LISTEN FOR SIMULATION MEASUREMENTS
		// --------------------------------------------------------------------

		// measurements come from either simulation output or sensors
        string topic = gad.useSensorsForEstimatesFlag?
            "goss.gridappsd.simulation.gridappsd-sensor-simulator."+gad.simid+".output":
            "goss.gridappsd.simulation.output."+gad.simid;

		SELoopConsumer measurementConsumer(&workQueue, gad.brokerURI,
                                gad.username, gad.password, topic, "topic");
		Thread measurementConsumerThread(&measurementConsumer);
		measurementConsumerThread.start();	// execute measurementConsumer.run()
		measurementConsumer.waitUntilReady();	// wait for the startup latch release

#ifdef DEBUG_PRIMARY
        if (gad.useSensorsForEstimatesFlag)
            *selog << "\nListening for sensor-simulator output on "+topic+'\n' << std::flush;
        else
            *selog << "\nListening for simulation output on "+topic+'\n' << std::flush;
#endif

#ifdef DEBUG_PRIMARY
        // only block initialization for command line invocations
        //if (false) {
        if (!gad.stateEstimatorFromPlatformFlag) {
		    *selog << "\nWaiting for measurement before continuing with initialization\n" << std::flush;
            while (blockedFlag) sleep(1);
		    *selog << "\nGot measurement--continuing with initialization\n" << std::flush;
        } else {
		    *selog << "\nNOT waiting before continuing with initialization\n" << std::flush;
        }
#endif

		// --------------------------------------------------------------------
		// MAKE SOME SPARQL QUERIES
		// --------------------------------------------------------------------

		// get nodes, bus mRIDs and phases
		SSMAP node_bmrids;
		SSMAP node_phs;
		state_estimator_util::get_nodes(gad,node_bmrids,node_phs);

		// --------------------------------------------------------------------
		// TOPOLOGY PROCESSOR
		// --------------------------------------------------------------------

		// Set up the ybus consumer
		string ybusTopic = "goss.gridappsd.se.response."+gad.simid+".ybus";
		TopoProcConsumer ybusConsumer(gad.brokerURI,gad.username,gad.password,ybusTopic,"queue");
		Thread ybusConsumerThread(&ybusConsumer);
		ybusConsumerThread.start();		// execute ybusConsumer.run()
		ybusConsumer.waitUntilReady();	// wait for latch release

		// Set up the vnom consumer
		string vnomTopic = "goss.gridappsd.se.response."+gad.simid+".vnom";
		VnomConsumer vnomConsumer(gad.brokerURI,gad.username,gad.password,vnomTopic,"queue");
		Thread vnomConsumerThread(&vnomConsumer);
		vnomConsumerThread.start();		// execute vnomConsumer.run()
		vnomConsumer.waitUntilReady();	// wait for latch release

		// Set up the producer to request the ybus and vnom
		string topoRequestTopic = "goss.gridappsd.process.request.config";
		SEProducer topoRequester(gad.brokerURI,gad.username,gad.password,topoRequestTopic,"queue");
		string ybusRequestText = 
			"{\"configurationType\":\"YBus Export\",\"parameters\":{\"simulation_id\":\"" 
			+ gad.simid + "\"}}";
		string vnomRequestText = 
			"{\"configurationType\":\"Vnom Export\",\"parameters\":{\"simulation_id\":\""
			+ gad.simid + "\"}}";
		topoRequester.send(ybusRequestText,ybusTopic);
		topoRequester.send(vnomRequestText,vnomTopic);
		topoRequester.close();

		// Initialize topology
		uint node_qty;		// number of nodes
		SLIST node_names;	// list of node names
		SIMAP node_idxs;	// map from node name to unit-indexed position
        ISMAP node_name_lookup;
		IMMAP Y;			// double map from indices to complex admittance
		// G, B, g, and b are derived from Y:
		//	-- Gij = std::real(Y[i][j]);
		//	-- Bij = std::imag(Y[i][j]);
		//	-- gij = std::real(-1.0*Y[i][j]);
		//	-- bij = std::imag(-1.0*Y[i][j]);
		
		// Wait for topological processor and retrieve topology
		ybusConsumerThread.join();
		ybusConsumer.fillTopo(node_qty,node_names,node_idxs,node_name_lookup,Y);
		ybusConsumer.close();

		// Initialize nominal voltages
		SCMAP node_vnoms;
		
		// Wait for the vnom processor and retrive vnom
        vnomConsumerThread.join();
		vnomConsumer.fillVnom(node_vnoms);
        vnomConsumer.close();
        
		// BUILD THE A-MATRIX
		IMDMAP Amat;
        SSMAP reg_cemrid_primbus_map;
        SSMAP reg_cemrid_regbus_map;
        SSMAP regid_primnode_map;
        SSMAP regid_regnode_map;
		state_estimator_util::build_A_matrix(gad,Amat,node_idxs,
                reg_cemrid_primbus_map,reg_cemrid_regbus_map,
                regid_primnode_map,regid_regnode_map);

		// --------------------------------------------------------------------
		// SENSOR INITILIZER
		// --------------------------------------------------------------------
	    // map conducting equipment terminals to bus names	
//      SSMAP term_bus_map;
//      state_estimator_util::build_term_bus_map(gad, term_bus_map);

		// Set up the sensors consumer
		string sensTopic = "goss.gridappsd.se.response."+gad.simid+".cimdict";
		SensorDefConsumer sensConsumer(gad.brokerURI,gad.username,gad.password, 
                reg_cemrid_primbus_map,reg_cemrid_regbus_map,
                sensTopic,"queue");
		Thread sensConsumerThread(&sensConsumer);
		sensConsumerThread.start();		// execute sensConsumer.run()
		sensConsumer.waitUntilReady();	// wait for latch release

		// Set up the producer to request sensor data
		string sensRequestTopic = "goss.gridappsd.process.request.config";
		string sensRequestText = 
			"{\"configurationType\":\"CIM Dictionary\",\"parameters\":{\"simulation_id\":\""
			+ gad.simid + "\"}}";
		SEProducer sensRequester(gad.brokerURI,gad.username,gad.password,sensRequestTopic,"queue");
		sensRequester.send(sensRequestText,sensTopic);
		sensRequester.close();

		// Initialize sensors
		SensorArray zary;
        SSMAP mmrid_pos_type_map;
//		uint numms; 	// number of sensors
//		SLIST mns;		// sensor name [list of strings]
//		SSMAP mts;		// sensor type [sn->str]
//		SDMAP msigs;	// sensor sigma: standard deviation [sn->double]
//		SSMAP mnd1s;	// point node or from node for flow sensors [sn->str]
//		SSMAP mnd2s;	// point node or to node for flow sensors [sn->str]
//		SDMAP mvals;	// value of the latest measurement [sn->double]
		
		// Wait for sensor initializer and retrieve sensors
		sensConsumerThread.join();

        // Add Sensors
		sensConsumer.fillSens(zary, mmrid_pos_type_map);
		sensConsumer.close();

        // system base power, functionally arbitrary -- can be tweaked
        // for better numerical stability
        // values in the 1e+6 to 1e+12 seem to converge for all models
        // values in this range make a negligible difference in the condition
        // of the inverted Supd matrix
        // values both below this range like 1e+4 and above like 1e+14
        // result in higher condition numbers that could lead to instability
        // at least for the 9500-node model
		const double sbase = 1.0e+6;
		//const double sbase = 1.0e+12;

		// Add Pseudo-Measurements
		state_estimator_util::insert_pseudo_measurements(gad,zary,
				node_names,node_vnoms,sbase);

#ifdef DEBUG_PRIMARY
        *selog << "\nzsigs/zvals after adding pseudo-measurements:\n" << std::flush;
        for ( auto& zid : zary.zids ) {
            *selog << "\tzid: " << zid << ", ztype: " << zary.ztypes[zid] << ", zsig: " << zary.zsigs[zid] << ", zvals: " << zary.zvals[zid] << "\n" << std::flush;
        }
#endif

        // Initialize class that does the state estimates
		SELoopWorker loopWorker(&workQueue, gad.brokerURI, gad.username,
            gad.password, gad.simid, zary, node_qty, node_names, node_idxs,
            node_vnoms, node_bmrids, node_phs, node_name_lookup, sbase, Y, Amat,
            regid_primnode_map, regid_regnode_map, mmrid_pos_type_map);

#ifdef DEBUG_PRIMARY
		*selog << "\nStarting the SE work loop\n" << std::flush;
#endif
		loopWorker.workLoop();
		
        // we'll never get here
		return 0;

	} catch (...) {
		cerr << "Error: Unhandled Exception\n" << std::flush;
		throw NULL;
	}
}
