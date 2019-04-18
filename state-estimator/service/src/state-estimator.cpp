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

#include "sparql_queries.hpp"
using sparql_queries::sparq_conducting_equipment_vbase;
using sparql_queries::sparq_transformer_end_vbase;
using sparql_queries::sparq_energy_consumer_pq;
using sparql_queries::sparq_ratio_tap_changer_nodes;

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

// Hash address (i,j) to the index of a sparse matrix vector
#define ICMAP std::unordered_map<unsigned int,std::complex<double>>
#define IMMAP std::unordered_map<unsigned int,ICMAP>

// Store x and z in a list one-indexed by position
#define IDMAP std::unordered_map<unsigned int,double>

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
		// MAKE SOME MANUAL QUERIES
		// --------------------------------------------------------------------

		json jvbase1 = sparql_query(gad,"vbase1",sparq_conducting_equipment_vbase(gad.modelID));
		json jvbase2 = sparql_query(gad,"vbase2",sparq_transformer_end_vbase(gad.modelID));

		// VBASE magnitude: process results
		SLIST busnames;		// bus names
		SSMAP busids;		// bus name -> bus mRID
		SDMAP busvbases;	// bus name -> bus vbase
		for ( auto& bus : jvbase1["data"]["results"]["bindings"] ) {
			cout << bus.dump() + '\n';
			string busname = bus["busname"]["value"];
			string busid = bus["busid"]["value"];
			string vbasestr = bus["vbase"]["value"];
			double vbase = stod( vbasestr );
			busnames.push_back(busname);
			busids[busname] = busid;
			busvbases[busname] = vbase;
		}
		for ( auto& bus : jvbase2["data"]["results"]["bindings"] ) {
			cout << bus.dump() + '\n';
			string busname = bus["busname"]["value"];
			string busid = bus["busid"]["value"];
			string vbasestr = bus["vbase"]["value"];
			double vbase = stod( vbasestr );
			busnames.push_back(busname);
			busids[busname] = busid;
			busvbases[busname] = vbase;
		}
		busnames.sort();
		busnames.unique();

		// VBASE (magnitude): report results
		cout << "\n\nBuses from CIM:\n";
		for ( auto& busname : busnames ) cout << busname << " -> " << busvbases[busname] << '\n';


		// PSEUDO-MEASUREMENTS
		json jpsm = sparql_query(gad,"psm",sparq_energy_consumer_pq(gad.modelID));
		cout << jpsm.dump() + '\n';


		// --------------------------------------------------------------------
		// TOPOLOGY PROCESSOR
		// --------------------------------------------------------------------

		// Set up the ybus consumer
		string ybusTopic = "goss.gridappsd.se.response."+gad.simid+".ybus";
		TopoProcConsumer ybusConsumer(gad.brokerURI,gad.username,gad.password,ybusTopic,"queue");
		Thread ybusConsumerThread(&ybusConsumer);
		ybusConsumerThread.start();		// execute ybusConsumer.run()
		ybusConsumer.waitUntilReady();	// wait for latch release

		// Here, we can set up a consumer for the vnom file
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
//		topoRequester.send(vnomRequestText,vnomTopic);
		topoRequester.close();

		// Initialize topology
		uint node_qty;		// number of nodes
		SLIST node_names;	// list of node names
		SIMAP node_idxs;	// map from node name to unit-indexed position
		IMMAP Y;			// double map from indices to complex admittance
		// G, B, g, and b are derived from Y:
		//	-- Gij = std::real(Y[i][j]);
		//	-- Bij = std::imag(Y[i][j]);
		//	-- gij = std::real(-1.0*Y[i][j]);
		//	-- bij = std::imag(-1.0*Y[i][j]);
		
		// Wait for topological processor and retrieve topology
		ybusConsumerThread.join();
		ybusConsumer.fillTopo(node_qty,node_names,node_idxs,Y);
		ybusConsumer.close();

		// Initialize nominal voltages
		SCMAP node_vnoms;
		
		// Wait for the vnom processor and retrive vnom
		vnomConsumer.fillVnom(node_vnoms);
		int ctr = 0;
		for ( auto& node : node_names) {
			ctr ++;
			cout << node << " vnom: " << node_vnoms[node] << '\n';
		} cout << ctr << " total nodes\n";
		
		// BUILD THE A-MATRIX
		IMMAP A;
		json jregs = sparql_query(gad,"regs",sparq_ratio_tap_changer_nodes(gad.modelID));
		cout << jregs.dump() + '\n';
		for ( auto& reg : jregs["data"]["results"]["bindings"] ) {

			// Get the primary node
			string primbus = reg["primbus"]["value"];
			string primph = reg["primphs"]["value"];
			string primnode = primbus; for ( auto& c : primnode ) c = toupper(c);
			cout << primbus + '\t' + primph + '\n';
			if (!primph.compare("A")) primnode += ".1";
			if (!primph.compare("B")) primnode += ".2";
			if (!primph.compare("C")) primnode += ".3";
			if (!primph.compare("s1")) primnode += ".1";
			if (!primph.compare("s2")) primnode += ".2";
			uint primidx = node_idxs[primnode];
			cout << primnode + " index: " << primidx << '\n';

			// get the regulation node
			string regbus = reg["regbus"]["value"];
			string regph = reg["regphs"]["value"];
			string regnode = regbus; for ( auto& c : regnode ) c = toupper(c);
			cout << regbus + '\t' + regph + '\n';
			if (!regph.compare("A")) regnode += ".1";
			if (!regph.compare("B")) regnode += ".2";
			if (!regph.compare("C")) regnode += ".3";
			if (!regph.compare("s1")) regnode += ".1";
			if (!regph.compare("s2")) regnode += ".2";
			uint regidx = node_idxs[regnode];
			cout << regnode + " index: " << regidx << '\n';

			// initialize the A matrix
			A[primidx][regidx] = 1;		// this will change
			A[regidx][primidx] = 1;		// this stays unity and may not be required
		}



		// INITIALIZE THE STATE VECTOR
		IDMAP xV;	// container for voltage magnitude states
		IDMAP xT;	// container for voltage angle states
		for ( auto& node : node_names ) {
			xV[node_idxs[node]] = abs(node_vnoms[node]);
			xT[node_idxs[node]] = 180/PI * arg(node_vnoms[node]);
		}
		int xqty = xV.size() + xT.size();
		if ( xqty != 2*node_qty) throw "x initialization failed";
	
		// --------------------------------------------------------------------
		// SENSOR INITILIZER
		// --------------------------------------------------------------------
		
		// Set up the sensors consumer
		string sensTopic = "goss.gridappsd.se.response."+gad.simid+".cimdict";
		SensorDefConsumer sensConsumer(gad.brokerURI,gad.username,gad.password,sensTopic,"queue");
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

//// NO SENSORS RIGHT NOW
		sensConsumer.fillSens(zary);
		sensConsumer.close();


		// Initialize containers to hold pseudo-measurements
		SDMAP pseudoP, pseudoQ;

/* PSEUDO MEASUREMENTS EQUAL TO ZERO
		// Add nominal load injections
		for ( auto& load : jpsm["data"]["results"]["bindings"] ) {
			cout << load.dump() + "\n";
			string bus = load["busname"]["value"]; for ( char& c : bus ) c = toupper(c);
			cout << "bus: " + bus + '\n';
			if ( !load.count("phase") ) {
				// This is a 3-phase balanced load (handle D and Y the same)
				string sptot = load["pfixed"]["value"]; double ptot = stod(sptot);
				string sqtot = load["qfixed"]["value"]; double qtot = stod(sqtot);
				// Add injection to phase A
				pseudoP[bus+".1"] -= ptot/3.0/2.0;
				pseudoQ[bus+".1"] -= qtot/3.0/2.0;
				// Add injection to phase B
				pseudoP[bus+".2"] -= ptot/3.0/2.0;
				pseudoQ[bus+".2"] -= qtot/3.0/2.0;
				// Add injection to phase C
				pseudoP[bus+".3"] -= ptot/3.0/2.0;
				pseudoQ[bus+".3"] -= qtot/3.0/2.0;
			} else {
				cout << "not in phase\n";
				// This is a 1-phase load
				string spph = load["pfixedphase"]["value"]; double pph = stod(spph);
				string sqph = load["qfixedphase"]["value"]; double qph = stod(sqph);
				cout << "pph: " << pph << "\t\t" << "qph: " << qph << '\n';
				string phase = load["phase"]["value"];
				// determine the node
				string node = bus;
				if (!phase.compare("A")) node += ".1";
				if (!phase.compare("B")) node += ".2";
				if (!phase.compare("C")) node += ".3";
				if (!phase.compare("s1")) node += ".1";
				if (!phase.compare("s2")) node += ".2";
				// Handle Wye or Delta load
				string conn = load["conn"]["value"];
				if ( !conn.compare("Y") ) {
					// Wye-connected load - injections are 
					pseudoP[node] -= pph/2.0;
					pseudoQ[node] -= qph/2.0;
				}
				if ( !conn.compare("D") ) {
					// Delta-connected load - injections depend on load current
					complex<double> sload = complex<double>(pph,qph);
					// Find the nominal voltage across the load
					string n2 = bus;
					if (!phase.compare("A")) n2 += ".2";
					if (!phase.compare("B")) n2 += ".3";
					if (!phase.compare("C")) n2 += ".1";
					if (!phase.compare("s1")) n2 += ".2";
					if (!phase.compare("s2")) n2 += ".1";
					complex<double> vload = node_vnoms[node] - node_vnoms[n2];
					// Positive injection into the named node
					pseudoP[node] += real(sload/vload*node_vnoms[node])/2.0;
					pseudoQ[node] += imag(sload/vload*node_vnoms[node])/2.0;
					// Negative injection into the second node
					pseudoP[n2] -= real(sload/vload*node_vnoms[n2])/2.0;
					pseudoQ[n2] -= imag(sload/vload*node_vnoms[n2])/2.0;
				}
			}
		}
*/

/*
		// Add these injections to the sensor array
		for ( auto& node : node_names ) {
			// WHAT TO DO ABOUT THE MRIDS??
			//  - THESE MEASUREMENTS DON'T GET UPDATED
			//  - MAYBE WE DON'T ADD TO THE MRIDS

			// Add the P injection
			string pinj_zid = "pseudo_P_"+node;
			zary.zids.push_back(pinj_zid);
			zary.zidxs[pinj_zid] = zary.zqty++;
			zary.ztypes	[pinj_zid] = "Pi";
			zary.zsigs	[pinj_zid] = 5000.0;
			zary.znode1s[pinj_zid] = node;
			zary.znode2s[pinj_zid] = node;
			zary.zvals	[pinj_zid] = pseudoP[node];
			zary.znew	[pinj_zid] = false;

			// Add the Q injection
			string qinj_zid = "pseudo_Q_"+node;
			zary.zids.push_back(qinj_zid);
			zary.zidxs[qinj_zid] = zary.zqty++;
			zary.ztypes	[qinj_zid] = "Qi";
			zary.zsigs	[qinj_zid] = 2000.0;
			zary.znode1s[qinj_zid] = node;
			zary.znode2s[qinj_zid] = node;
			zary.zvals	[qinj_zid] = pseudoQ[node];
			zary.znew	[qinj_zid] = false;
		}
*/
		for ( auto& zid : zary.zids ) {
			cout << zid << '\t' << zary.zvals[zid] << '\n';
		}


		// --------------------------------------------------------------------
		// LISTEN FOR MEASUREMENTS
		// --------------------------------------------------------------------

		// ideally we want to compute an estimate on a thread at intervals and
		//   collect measurements in the meantime

		for ( auto& node: node_names ) cout << node+'\n';
		// measurements come from the simulation output
		string simoutTopic = "goss.gridappsd.simulation.output."+gad.simid;
		SELoopConsumer loopConsumer(gad.brokerURI,gad.username,gad.password,
			simoutTopic,"topic",gad.simid,
			zary,node_qty,node_names,node_idxs,node_vnoms,Y,A);
		Thread loopConsumerThread(&loopConsumer);
		loopConsumerThread.start();	// execute loopConsumer.run()
		loopConsumer.waitUntilReady();	// wait for the startup latch release
		
		cout << "\nListening for simulation output on "+simoutTopic+'\n';
		
//		// I'm not sure of the right way to synchronously access the message content
//		// For now, all processing will be done inside the consumer.
//		//   - messages will be dropped if processing is not complete in time.
//		//	 - at a minimum, the algorithm cares about the time between measurements
//		while ( loopConsumer.doneLatch.getCount() ) {
//			loopConsumer.waitForData();		// don't process until data write is complete
//
//		}
		
		// wait for the estimator to exit:
		loopConsumerThread.join(); loopConsumer.close();

		// now we're done
		return 0;

	} catch (...) {
		std::cerr << "Error: Unhandled Exception\n";
		throw NULL;
	}
	
}
