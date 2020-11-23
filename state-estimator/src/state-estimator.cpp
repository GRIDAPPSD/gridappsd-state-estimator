#define DIAGONAL_P
#define SWITCHES
//#define NET_INJECTION
#define DEBUG_PRIMARY
//#define DEBUG_FILES
//#define DEBUG_SIZES
//#define SBASE_TESTING
#define GS_OPTIMIZE
//#define TEST_HARNESS_DIR "test_4"
//#define TEST_HARNESS_DIR "test_13assets"

// conditional compilation logic to properly synchronize test harness
// related symbols
// TEST_HARNESS_SIM_SYNC indicates whether there is a synchronized simulation
// running along with reading test harness files so that results can be
// published and plotted to match running simulation measurements.
// Setting this indicates that some data is initialized from query responses
// like node_vnoms, node_bmrids, and node_phs because it is needed to publish
// data for plotting.  The ieee13nodecktassets model supports this while
// the 4-bus MATLAB reference model does not because there is not a
// corresponding GridAPPS-D model.  The value of TEST_HARNESS_SIM_SYNC also
// indicates whether to read the timestamp for measurement data from file
// or get it from running simulation message even though the rest of the
// measurement data is read from the file. It should only be set when
// TEST_HARNESS_DIR is also set and thus is should remain in the ifdef
// block below and then TEST_HARNESS_SIM_SYNC should be uncommented and
// commented out as needed.
#ifdef TEST_HARNESS_DIR
//#define TEST_HARNESS_SIM_SYNC
#endif

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
#define SSLISTMAP std::unordered_map<std::string,SLIST>

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
#if !defined( TEST_HARNESS_DIR ) || defined( TEST_HARNESS_SIM_SYNC )
		state_estimator_util::get_nodes(gad,node_bmrids,node_phs);
#endif

		// --------------------------------------------------------------------
		// TOPOLOGY PROCESSOR
		// --------------------------------------------------------------------

		// Initialize topology
		uint node_qty;      // number of nodes
		SLIST node_names;   // list of node names
		SIMAP node_idxs;    // map from node name to unit-indexed position
        ISMAP node_name_lookup;
		IMMAP Yphys;        // double map from indices to complex admittance
		// G, B, g, and b are derived from Yphys:
		//	-- Gij = std::real(Yphys[i][j]);
		//	-- Bij = std::imag(Yphys[i][j]);
		//	-- gij = std::real(-1.0*Yphys[i][j]);
		//	-- bij = std::imag(-1.0*Yphys[i][j]);

		// Initialize nominal voltages
		SCMAP node_vnoms;

#ifndef TEST_HARNESS_DIR
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
		topoRequester.send(ybusRequestText,ybusTopic);
		string vnomRequestText = 
			"{\"configurationType\":\"Vnom Export\",\"parameters\":{\"simulation_id\":\""
			+ gad.simid + "\"}}";
		topoRequester.send(vnomRequestText,vnomTopic);
		topoRequester.close();

		// Wait for topological processor and retrieve topology
		ybusConsumerThread.join();
		ybusConsumer.fillTopo(node_qty,node_names,node_idxs,
                              node_name_lookup,Yphys);
		ybusConsumer.close();

		// Wait for the vnom processor and retrive vnom
        vnomConsumerThread.join();
        vnomConsumer.fillVnom(node_vnoms);
        vnomConsumer.close();
#else
        string filename = TEST_HARNESS_DIR;
        filename += "/ysparse.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading ybus from test harness file: " << filename << "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        string line;
        getline(ifs, line);  // throwaway header line
        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell;
            getline(lineStream, cell, ','); int i = stoi(cell);
            getline(lineStream, cell, ','); int j = stoi(cell);
            getline(lineStream, cell, ','); double G = stod(cell);
            getline(lineStream, cell, ','); double B = stod(cell);

            Yphys[i][j] = complex<double>(G,B);
            if ( i != j ) Yphys[j][i] = complex<double>(G,B);
        }
        ifs.close();

        filename = TEST_HARNESS_DIR;
        filename += "/nodelist.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading nodelist from test harness file: " << filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);

        node_qty = 0;
        while ( getline(ifs, line) ) {
            // Extract the node name
            string node_name = regex_replace(line,regex("\""),"");
            // Store the node information
            node_names.push_back(node_name);
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }
        ifs.close();

#ifdef TEST_HARNESS_SIM_SYNC
        filename = TEST_HARNESS_DIR;
        filename += "/vnom.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading vnom from test harness file: " << filename << "\n" << std::flush;
#endif
        ifs.open(filename);
        getline(ifs, line);  // throwaway header line
        while (getline(ifs, line)) {
            std::stringstream lineStream(line);
            string node, cell;
            getline(lineStream, node, ',');
            getline(lineStream, cell, ','); double mag = stod(cell);
            getline(lineStream, cell, ','); double arg = stod(cell);
            double vre = mag * cos( arg * PI/180 );
            double vim = mag * sin( arg * PI/180);
            complex<double> vnom = complex<double>(vre,vim);
            node_vnoms[node] = vnom;
        }
        ifs.close();
#else
        for ( auto& node_name : node_names ) {
            node_vnoms[node_name] = 1;
        }
#endif
#endif
        
        // system base power, functionally arbitrary -- can be tweaked
        // for better numerical stability if needed
        // all values in the approximate range 1e-140 to 1e+150 converge
        // and only numeric overflow/underflow results in failures for the
        // 3 models tested (ieee13nodecktassets, ieee123, test9500new)
        // values in the 1e+6 to 1e+12 range result in minimum Supd condition
        // numbers with the range for lowest condition varying somewhat between
        // the 3 models tested
#ifdef SBASE_TESTING
        double spower = (double)std::stoi(argv[3]);
		const double sbase = pow(10.0, spower);
#else
		const double sbase = 1.0e+6;
#endif

		// --------------------------------------------------------------------
		// SENSOR INITIALIZATION
		// --------------------------------------------------------------------

		// A matrix data structures
		IMDMAP Amat;
        SSMAP regid_primnode_map;
        SSMAP regid_regnode_map;

        // Sensors data structures
        SensorArray zary;
        SSMAP mmrid_pos_type_map;
        SSMAP switch_node1s;
        SSMAP switch_node2s;

#ifndef TEST_HARNESS_DIR
        SSMAP reg_cemrid_primbus_map;
        SSMAP reg_cemrid_regbus_map;
		state_estimator_util::build_A_matrix(gad,Amat,node_idxs,
                reg_cemrid_primbus_map,reg_cemrid_regbus_map,
                regid_primnode_map,regid_regnode_map);

        // map conducting equipment to bus names
        SSLISTMAP cemrid_busnames_map;
        state_estimator_util::build_cemrid_busnames_map(gad,
                cemrid_busnames_map);

		// Add Pseudo-Measurements
        SDMAP node_nominal_Pinj_map;
        SDMAP node_nominal_Qinj_map;
		state_estimator_util::get_nominal_energy_consumer_injections(gad,
				node_vnoms,node_nominal_Pinj_map,node_nominal_Qinj_map);

		// Set up the sensors consumer
		string sensTopic = "goss.gridappsd.se.response."+gad.simid+".cimdict";
		SensorDefConsumer sensConsumer(gad.brokerURI,gad.username,gad.password, 
               cemrid_busnames_map,reg_cemrid_primbus_map,reg_cemrid_regbus_map,
               node_nominal_Pinj_map,node_nominal_Qinj_map,
               sbase,sensTopic,"queue");
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

		// Wait for sensor initializer and retrieve sensors
		sensConsumerThread.join();

        // Add Sensors
		sensConsumer.fillSens(zary, mmrid_pos_type_map,
                              switch_node1s, switch_node2s);
		sensConsumer.close();

        // For the test harness, SensorDefConsumer reads the file for all
        // measurements so no need to do anything for pseudo-measurements
		// Add Pseudo-Measurements
		state_estimator_util::insert_pseudo_measurements(gad,zary,
				node_names,node_vnoms,sbase);
#ifdef DEBUG_PRIMARY
        //*selog << "\nzsigs/zvals after adding pseudo-measurements:\n" << std::flush;
        //for ( auto& zid : zary.zids ) {
        //    *selog << "\tzid: " << zid << ", ztype: " << zary.ztypes[zid] << ", zsig: " << zary.zsigs[zid] << ", zvals: " << zary.zvals[zid] << "\n" << std::flush;
        //}
#endif
#else
        filename = TEST_HARNESS_DIR;
        filename += "/regid.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading regulator mappings from test harness file: " << filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);
        getline(ifs, line); // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string regid, primnode, regnode;
            getline(lineStream, regid, ',');
            getline(lineStream, primnode, ',');
            getline(lineStream, regnode, ',');

            regid_primnode_map[regid] = primnode;
            regid_regnode_map[regid] = regnode;

            uint primidx = node_idxs[primnode];
            uint regidx = node_idxs[regnode];
            // initialize the A matrix
            Amat[primidx][regidx] = 1; // this will change
            Amat[regidx][primidx] = 1; // this stays unity and may not be required
        }
        ifs.close();

        filename = TEST_HARNESS_DIR;
        filename += "/measurements.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading sensor measurements from test harness file: " << filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);
        getline(ifs, line); // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell, zid;
            getline(lineStream, cell, ',');
            getline(lineStream, zid, ','); zary.zids.push_back(zid);
            zary.zidxs[zid] = zary.zqty++;
            zary.ztypes[zid] = cell;
            getline(lineStream, cell, ','); zary.znode1s[zid] = cell;
            getline(lineStream, cell, ','); zary.znode2s[zid] = cell;
            getline(lineStream, cell, ','); zary.zvals[zid] = stod(cell);
            getline(lineStream, cell, ','); zary.zsigs[zid] = stod(cell);
            getline(lineStream, cell, ','); zary.zpseudos[zid] = cell=="1";
            getline(lineStream, cell, ','); zary.znomvals[zid] = stod(cell);
        }
        ifs.close();
#endif

        // Initialize class that does the state estimates
        SELoopWorker loopWorker(&workQueue, &gad, zary, node_qty, node_names,
            node_idxs, node_vnoms, node_bmrids, node_phs, node_name_lookup,
            sbase, Yphys, Amat, regid_primnode_map, regid_regnode_map,
            mmrid_pos_type_map, switch_node1s, switch_node2s);

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
