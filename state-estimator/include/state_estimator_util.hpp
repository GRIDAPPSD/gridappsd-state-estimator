#ifndef SE_INIT_HPP
#define SE_INIT_HPP

#include "state_estimator_gridappsd.hpp"
using state_estimator_gridappsd::gridappsd_session;

#include "gridappsd_requests.hpp"
using gridappsd_requests::sparql_query;

//#include "sparql_queries.hpp"
#include "sparql_queries_CIM100.hpp"
using sparql_queries::sparq_nodes;
using sparql_queries::sparq_transformer_end_vbase;
using sparql_queries::sparq_energy_consumer_pq;
using sparql_queries::sparq_ratio_tap_changer_nodes;
using sparql_queries::sparq_energy_source_buses;
using sparql_queries::sparq_term_bus;
using sparql_queries::sparq_cemrid_busnames;

#include <string>
#define SLIST std::list<std::string>
#define SIMAP std::unordered_map<std::string,unsigned int>
#define SDMAP std::unordered_map<std::string,double>
#define SCMAP std::unordered_map<std::string,std::complex<double>>
#define SSMAP std::unordered_map<std::string,std::string>
#define IDMAP std::unordered_map<unsigned int,double>
#define IMDMAP std::unordered_map<unsigned int,IDMAP>
#define SSLISTMAP std::unordered_map<std::string,SLIST>

namespace state_estimator_util{

	void get_nodes(gridappsd_session& gad, SSMAP& node_bmrids, SSMAP& node_phs) {
		json jnodes = sparql_query(gad,"nodes",sparq_nodes(gad.modelID)); 
		//*selog << jnodes.dump(2); 
		for ( auto& binding : jnodes["data"]["results"]["bindings"] ) { 
			string busname = binding["busname"]["value"]; 
			string busid = binding["busid"]["value"]; 
			for ( auto& c : busname ) c = toupper(c); 
	
			try { // phs contains one or more discrete phases 
				string phs = binding["phases"]["value"]; 
				// phase A 
				if (phs.find("A")!=string::npos) { 
					node_bmrids[busname+".1"] = busid; 
					node_phs[busname+".1"] = "A"; 
				}
				// phase B 
				if (phs.find("B")!=string::npos) { 
					node_bmrids[busname+".2"] = busid;
					node_phs[busname+".2"] = "B";
				}
				// phase C
				if (phs.find("C")!=string::npos) {
					node_bmrids[busname+".3"] = busid;
					node_phs[busname+".3"] = "C";
				}
				// phase s1
				if (phs.find("s1")!=string::npos) {
					node_bmrids[busname+".1"] = busid;
					node_phs[busname+".1"] = "s1";
				}
				// phase s2
				if (phs.find("s2")!=string::npos) {
					node_bmrids[busname+".2"] = busid;
					node_phs[busname+".2"] = "s2";
				}
			} catch ( ... ) {
				// phase A
				node_bmrids[busname+".1"] = busid;
				node_phs[busname+".1"] = "A";
				// phase B
				node_bmrids[busname+".2"] = busid;
				node_phs[busname+".2"] = "B";
				// phase C
				node_bmrids[busname+".3"] = busid;
				node_phs[busname+".3"] = "C";
			}
		}
	}


	void insert_pseudo_measurements(gridappsd_session& gad, SensorArray& zary,
				SLIST& node_names, SCMAP& node_vnoms, const double sbase) {
		json jpsm = sparql_query(gad,"psm",sparq_energy_consumer_pq(gad.modelID));

		// Initialize containers to hold pseudo-measurements
		SDMAP pseudoP, pseudoQ;
        double nominal_systemP, nominal_systemQ;

		// Add nominal load injections
		for ( auto& load : jpsm["data"]["results"]["bindings"] ) {
		 	string bus = load["busname"]["value"];
            for ( char& c : bus ) c = toupper(c);

			if ( !load.count("phase") ) {
				// This is a 3-phase balanced load (handle D and Y the same)
				string sptot = load["p_3p"]["value"]; double ptot = stod(sptot);
				string sqtot = load["q_3p"]["value"]; double qtot = stod(sqtot);
				// Add injection to phase A
				pseudoP[bus+".1"] -= ptot/3.0/2.0;
				pseudoQ[bus+".1"] -= qtot/3.0/2.0;
				// Add injection to phase B
				pseudoP[bus+".2"] -= ptot/3.0/2.0;
				pseudoQ[bus+".2"] -= qtot/3.0/2.0;
				// Add injection to phase C
				pseudoP[bus+".3"] -= ptot/3.0/2.0;
				pseudoQ[bus+".3"] -= qtot/3.0/2.0;

                // Add combined injection to total
                nominal_systemP += ptot/2.0;
                nominal_systemQ += qtot/2.0;
			} else {
				// This is a 1-phase load
				string spph = load["p_phase"]["value"]; double pph = stod(spph);
				string sqph = load["q_phase"]["value"]; double qph = stod(sqph);
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

                    // Add phase injection to total
                    nominal_systemP += pph/2.0;
                    nominal_systemQ += qph/2.0;
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
					// Positive load at the named node
					pseudoP[node] -= real(sload/vload*node_vnoms[node])/2.0;
					pseudoQ[node] -= imag(sload/vload*node_vnoms[node])/2.0;
					// Negative load at the second node
					pseudoP[n2] += real(sload/vload*node_vnoms[n2])/2.0;
					pseudoQ[n2] += imag(sload/vload*node_vnoms[n2])/2.0;

                    // Add net load injection to total
					nominal_systemP += real(sload/vload*node_vnoms[node])/2.0;
					nominal_systemQ += imag(sload/vload*node_vnoms[node])/2.0;
					nominal_systemP -= real(sload/vload*node_vnoms[n2])/2.0;
					nominal_systemQ -= imag(sload/vload*node_vnoms[n2])/2.0;
				}
			}
		}


		// Add these injections and sourcebus voltages to the sensor array
		json jesources = sparql_query(gad,"esources",sparq_energy_source_buses(gad.modelID));

        json source_buses = jesources["data"]["results"]["bindings"];

        if ( source_buses.size() != 1 ) {
            cerr << "ERROR: number of energy sources (" << source_buses.size() << ") is not 1\n" << std::flush;
            throw("invalid number of energy sources");
        }

        string sourcebus = source_buses[0]["bus"]["value"]; 
        for ( auto& c : sourcebus ) c = toupper(c);
        string source_node_prefix = sourcebus + ".";


		for ( auto& node : node_names ) {

			// Check for SOURCEBUS
//			if ( !node.compare(0,9,"SOURCEBUS") ) {
            if ( node.find(source_node_prefix) == 0 ) {

				// Add sourcebus voltage magnitude
				string vmag_zid = "source_V_"+node;
				zary.zids.push_back(vmag_zid);
				zary.zidxs  [vmag_zid] = zary.zqty++;
				zary.ztypes [vmag_zid] = "vi";
				zary.zsigs  [vmag_zid] = 0.001; // 1 sigma = 0.1%
				zary.znode1s[vmag_zid] = node;
				zary.znode2s[vmag_zid] = node;
				zary.zvals  [vmag_zid] = 1.00;
				zary.znew   [vmag_zid] = 0;
				zary.ztime  [vmag_zid] = 0;
				zary.znotime[vmag_zid] = true;

//                *selog << "**Source Bus node: " << node << '\n' << std::flush;
//                *selog << "\tsource_node_prefix: " << source_node_prefix << '\n' << std::flush;

				// Add sourcebus voltage phase
				string varg_zid = "source_T_"+node;
				zary.zids.push_back(varg_zid);
				zary.zidxs  [varg_zid] = zary.zqty++;
				zary.ztypes [varg_zid] = "Ti";
				zary.zsigs  [varg_zid] = 0.01;
				zary.znode1s[varg_zid] = node;
				zary.znode2s[varg_zid] = node;
				zary.zvals  [varg_zid] = 0.0;
				zary.znew   [varg_zid] = 0;
				zary.ztime  [varg_zid] = 0;
				zary.znotime[varg_zid] = true;
			}

			else {
                double loss_ratio = 0.05;

				// Add the P injection
				string pinj_zid = "pseudo_P_"+node;
				zary.zids.push_back(pinj_zid);
				zary.zidxs  [pinj_zid] = zary.zqty++;
				zary.ztypes	[pinj_zid] = "Pi";
				zary.znode1s[pinj_zid] = node;
				zary.znode2s[pinj_zid] = node;
				zary.znew   [pinj_zid] = 0;
				zary.ztime  [pinj_zid] = 0;
				zary.znotime[pinj_zid] = true;
				zary.zvals  [pinj_zid] = pseudoP[node]/sbase;
                zary.zsigs  [pinj_zid] = std::abs(pseudoP[node]/sbase) +
                    loss_ratio*(nominal_systemP/sbase)/node_names.size(); // load + leakage

//                *selog << "NON-Source Bus node: " << node << '\n' << std::flush;
//                *selog << "\tsource_node_prefix: " << source_node_prefix << '\n' << std::flush;
				
                // Add the Q injection
				string qinj_zid = "pseudo_Q_"+node;
				zary.zids.push_back(qinj_zid);
				zary.zidxs  [qinj_zid] = zary.zqty++;
				zary.ztypes	[qinj_zid] = "Qi";
				zary.znode1s[qinj_zid] = node;
				zary.znode2s[qinj_zid] = node;
				zary.znew   [qinj_zid] = 0;
				zary.ztime  [qinj_zid] = 0;
				zary.znotime[qinj_zid] = true;
				zary.zvals  [qinj_zid] = pseudoQ[node]/sbase;
                zary.zsigs  [qinj_zid] = std::abs(pseudoQ[node]/sbase) +
                    loss_ratio*(nominal_systemQ/sbase)/node_names.size(); // load + leakage
			}
		}
	}


	void build_A_matrix(gridappsd_session& gad, IMDMAP& A, SIMAP& node_idxs,
            SSMAP& reg_cemrid_primbus_map, SSMAP& reg_cemrid_regbus_map,
            SSMAP& regid_primnode_map, SSMAP& regid_regnode_map) {

		json jregs = sparql_query(gad,"regs",
                sparq_ratio_tap_changer_nodes(gad.modelID));
            
//        *selog << jregs.dump(2);

		for ( auto& reg : jregs["data"]["results"]["bindings"] ) {

			// Get the primary node
			string primbus = reg["primbus"]["value"];
			string primph = reg["primphs"]["value"];
			for ( auto& c : primbus ) c = toupper(c);
            string primnode = primbus;
			if (!primph.compare("A")) primnode += ".1";
			if (!primph.compare("B")) primnode += ".2";
			if (!primph.compare("C")) primnode += ".3";
			if (!primph.compare("s1")) primnode += ".1";
			if (!primph.compare("s2")) primnode += ".2";
			uint primidx = node_idxs[primnode];

			// get the regulation node
			string regbus = reg["regbus"]["value"];
			string regph = reg["regphs"]["value"];
			for ( auto& c : regbus ) c = toupper(c);
            string regnode = regbus;
			if (!regph.compare("A")) regnode += ".1";
			if (!regph.compare("B")) regnode += ".2";
			if (!regph.compare("C")) regnode += ".3";
			if (!regph.compare("s1")) regnode += ".1";
			if (!regph.compare("s2")) regnode += ".2";
			uint regidx = node_idxs[regnode];

//            // print
//            *selog << "reg: " << reg << "\n" << std::flush;
//            *selog << "\tprimnode: " << primnode <<
//                "\tregnode: " << regnode << "\n" << std::flush;
//            *selog << "\tprimph: " << primph << 
//                "\tregph: " << regph << "\n" << std::flush;

			// initialize the A matrix
			A[primidx][regidx] = 1;	// this will change
			A[regidx][primidx] = 1;	// this stays unity and may not be required

            // map the power transformer mrid to prim and reg nodes
            // NOTE: This is over-written when multiple single-phase regulators
            //      are attached to a single multi-phase transformer
            string cemrid = reg["cemrid"]["value"];
            reg_cemrid_primbus_map[cemrid] = primbus;
            reg_cemrid_regbus_map[cemrid] = regbus;

            // map the regulator id to prim and reg nodes
            string regid = reg["rtcid"]["value"];
            regid_primnode_map[regid] = primnode;
            regid_regnode_map[regid] = regnode;
		}
	}

    void build_term_bus_map(gridappsd_session& gad, SSMAP& term_bus_map) {
        json jterms = sparql_query(gad,"terms",sparq_term_bus(gad.modelID));
        for ( auto& item : jterms["data"]["results"]["bindings"] ) {
            string termid = item["termid"]["value"];
            string busname = item["busname"]["value"];
			for ( auto& c : busname ) c = toupper(c);
            term_bus_map[termid] = busname;
        }
    }

    void build_cemrid_busnames_map(gridappsd_session& gad,
            SSLISTMAP& cemrid_busnames_map) {
        json jbusnames = sparql_query(gad,"busnames",sparq_cemrid_busnames(gad.modelID));
        for ( auto& item : jbusnames["data"]["results"]["bindings"] ) {
            string cemrid = item["cemrid"]["value"];
            string busname = item["busname"]["value"];
			for ( auto& c : busname ) c = toupper(c);
            cemrid_busnames_map[cemrid].push_back(busname);
        }
    }

} // end namespace state_estimator_util

#endif // SE_INIT_HPP
