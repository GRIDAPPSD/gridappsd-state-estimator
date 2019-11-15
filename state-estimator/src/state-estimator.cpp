#define DIAGONAL_P
#define DEBUG_PRIMARY
//#define DEBUG_FILES

#define PI 3.1415926535

#include "state_estimator_gridappsd.hpp"
#include "SEProducer.hpp"
#include "GenericConsumer.hpp"
#include "TopoProcConsumer.hpp"
#include "VnomConsumer.hpp"
#include "SensorDefConsumer.hpp"
#include "SELoopConsumer.hpp"

#include "json.hpp"
using json = nlohmann::json;

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

#include <iostream>

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

int main(int argc, char** argv){
	
	// ------------------------------------------------------------------------
	// INITIALIZE THE STATE ESTIMATOR SESSION WITH RUNTIME ARGS
	// ------------------------------------------------------------------------
	state_estimator_gridappsd::state_estimator_session se;
	if ( !se.init(argc,argv) ) return 0;

	// ------------------------------------------------------------------------
	// INITIALIZE THE GRIDAPPS SESSION
	// ------------------------------------------------------------------------
	state_estimator_gridappsd::gridappsd_session gad(se);

	// ------------------------------------------------------------------------
	// START THE AMQ INTERFACE
	// ------------------------------------------------------------------------

	try {
		activemq::library::ActiveMQCPP::initializeLibrary();

		// --------------------------------------------------------------------
		// MAKE SOME SPARQL QUERIES
		// --------------------------------------------------------------------

		// get nodes, bus mRIDs and phases
		SSMAP node_bmrids;
		SSMAP node_phs;
		state_estimator_util::get_nodes(gad,node_bmrids,node_phs);



//		// PSEUDO-MEASUREMENTS
//		json jpsm = sparql_query(gad,"psm",sparq_energy_consumer_pq(gad.modelID));
//		cout << jpsm.dump() + '\n' << std::flush;


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
		IMDMAP A;
		state_estimator_util::build_A_matrix(gad,A,node_idxs);


/*
		// INITIALIZE THE STATE VECTOR
		IDMAP xV;	// container for voltage magnitude states
		IDMAP xT;	// container for voltage angle states
		for ( auto& node : node_names ) {
			xV[node_idxs[node]] = abs(node_vnoms[node]);
			xT[node_idxs[node]] = 180/PI * arg(node_vnoms[node]);
		}
		int xqty = xV.size() + xT.size();
		if ( xqty != 2*node_qty) throw "x initialization failed";
*/

		// --------------------------------------------------------------------
		// SENSOR INITILIZER
		// --------------------------------------------------------------------
	    // map conducting equipment terminals to bus names	
        SSMAP term_bus_map;
        state_estimator_util::build_term_bus_map(gad, term_bus_map);

		// Set up the sensors consumer
		string sensTopic = "goss.gridappsd.se.response."+gad.simid+".cimdict";
		SensorDefConsumer sensConsumer(gad.brokerURI,gad.username,gad.password, 
            term_bus_map,sensTopic,"queue");
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
//		uint numms; 	// number of sensors
//		SLIST mns;		// sensor name [list of strings]
//		SSMAP mts;		// sensor type [sn->str]
//		SDMAP msigs;	// sensor sigma: standard deviation [sn->double]
//		SSMAP mnd1s;	// point node or from node for flow sensors [sn->str]
//		SSMAP mnd2s;	// point node or to node for flow sensors [sn->str]
//		SDMAP mvals;	// value of the latest measurement [sn->double]
		
		// Wait for sensor initializer and retrieve sensors
		sensConsumerThread.join();

        // TODO: Uncomment the following two lines to add sensors
		sensConsumer.fillSens(zary);
		sensConsumer.close();

		// Add Pseudo-Measurements
		const double sbase = 1000000.0;
		state_estimator_util::insert_pseudo_measurements(gad,zary,
				node_names,node_vnoms,sbase);

		// --------------------------------------------------------------------
		// LISTEN FOR MEASUREMENTS
		// --------------------------------------------------------------------

		// ideally we want to compute an estimate on a thread at intervals and
		//   collect measurements in the meantime

		// measurements come from the simulation output
		string simoutTopic = "goss.gridappsd.simulation.output."+gad.simid;
		SELoopConsumer loopConsumer(gad.brokerURI,gad.username,gad.password,
			simoutTopic,"topic",gad.simid,zary,
			node_qty,node_names,node_idxs,node_vnoms,node_bmrids,node_phs,
			node_name_lookup,sbase,Y,A);
		Thread loopConsumerThread(&loopConsumer);
		loopConsumerThread.start();	// execute loopConsumer.run()
		loopConsumer.waitUntilReady();	// wait for the startup latch release
		
#ifdef DEBUG_PRIMARY
		cout << "\nListening for simulation output on "+simoutTopic+'\n' << std::flush;
#endif
		
		// wait for the estimator to exit:
		loopConsumerThread.join(); loopConsumer.close();

		// now we're done
		return 0;

	} catch (...) {
		cerr << "Error: Unhandled Exception\n" << std::flush;
		throw NULL;
	}
	
}
