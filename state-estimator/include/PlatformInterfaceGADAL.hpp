//#ifndef PLATFORMINTERFACEGADAL_HPP
#define PLATFORMINTERFACEGADAL_HPP

#include <iomanip> // std::setprecision

#define FILE_INTERFACE_READ "Results"

#ifndef FILE_INTERFACE_READ
#include "OOPS, NEED TO DEFINE FILE_INTERFACE_READ!"
#endif

// whether to get node_vnoms from file or hardwire to 1
#define FILE_INTERFACE_VNOM
// the nosbase symbol is used for a model outside GridAPPS-D like the
// 4-bus MATLAB model
//#define SBASE_NONE
//dummy
//#include "helics/application_api/BrokerApp.hpp"
//#include "helics/application_api/CombinationFederate.hpp"
//#include "helics/core/helicsCLI11.hpp"
//#include "helics/core/helics_definitions.hpp"
#include <helics/cpp98/ValueFederate.hpp>
#include <helics/cpp98/helics.hpp>
#include <cmath>
#include "json.hpp"
using json = nlohmann::json;

#include "SharedQueue.hpp"

#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>


class PlatformInterface : public PlatformInterfaceBase {
public:
    PlatformInterface(int argc, char** argv, const double& sbase) : PlatformInterfaceBase(argc, argv, sbase) {
    std::string fedinitstring = "--federates=1";
    double deltat = 1;
    Sbase = sbase;

    std::filesystem::path inputMapFile = std::filesystem::current_path() / "input_mapping.json";
    std::filesystem::path staticInFile = std::filesystem::current_path() / "static_inputs.json";
    std::cout << "Reading OEDISI configurations from : " << inputMapFile << " and " << staticInFile << std::endl;

    std::ifstream f1(inputMapFile.string());
    inputMap = json::parse(f1);
    std::ifstream f2(staticInFile.string());
    staticInput = json::parse(f2);

    std::cout <<sbase << std::endl;
    std::string helicsversion = helicscpp::getHelicsVersionString();

    if (helicsversion.find("error") == std::string::npos) {
        // this has to do with tests passing on CI builds
        std::cout << "Helics version = " << helicsversion << '\n';
    }

    /* Create Federate Info object that describes the federate properties
     * Set federate name and core type from string
     */
    helicscpp::FederateInfo fi("zmq");

    /* Federate init string */
    fi.setCoreInit(fedinitstring);
    fi.setCoreName(staticInput["name"]);

    fi.setProperty(HELICS_PROPERTY_TIME_DELTA, deltat);

    fi.setProperty(HELICS_PROPERTY_INT_MAX_ITERATIONS, 100);

    fi.setProperty(HELICS_PROPERTY_INT_LOG_LEVEL, HELICS_LOG_LEVEL_WARNING);

    /* Create value federate */
    vfed = new helicscpp::ValueFederate(staticInput["name"], fi);
    std::cout << " Value federate created\n";

    // Set up subscriptions
    sub_topo = vfed->registerSubscription(inputMap["topology"],"");

    sub_V = vfed->registerSubscription(inputMap["sensor_voltage_magnitude"],"V");
    
    sub_P = vfed->registerSubscription(inputMap["sensor_power_real"],"W");
    
    sub_Q = vfed->registerSubscription(inputMap["sensor_power_imaginary"],"W");

    

    if(sub_topo.isValid()){
        std::cout << " Subscription registered for feeder Topology with port " << inputMap["topology"] << std::endl;
    }

    if(sub_P.isValid()){
        std::cout << " Subscription registered for feeder real power P with port " << inputMap["sensor_power_real"] << std::endl;
    }

    if(sub_Q.isValid()){
        std::cout << " Subscription registered for feeder reactive power Q with port " << inputMap["sensor_power_imaginary"] << std::endl;
    }

    if(sub_V.isValid()){
        std::cout << " Subscription registered for feeder voltage V with port " << inputMap["sensor_voltage_magnitude"] << std::endl;
    }

    // Set up publications
    pub_Vmag = vfed->registerPublication("Vmag_SE", HELICS_DATA_TYPE_STRING);

    pub_Vang = vfed->registerPublication("Vang_SE", HELICS_DATA_TYPE_STRING);

    if(pub_Vmag.isValid()){
        std::cout << " Vmag Publication registered\n";
    }
    if(pub_Vang.isValid()){
        std::cout << " Vang Publication registered\n";
    }

     /* Enter initialization state */
    vfed->enterInitializingMode();  // can throw helicscpp::InvalidStateTransition exception
    std::cout << " Entered initialization state" << std::endl;

    /* Enter execution state */
    vfed->enterExecutingMode();  // can throw helicscpp::InvalidStateTransition exception
    std::cout << " Entered execution state\n";

    currenttime = 0.0;
    ts=1;
    currenttime = vfed->requestTime(10000);

    // Get the topology subscription
    sub_topo.getString(topology);
    topo = json::parse(topology);

    // Get the voltage magnitude subscription
    if (sub_V.isUpdated())
    {
        sub_V.getString(voltages);
        V_meas = json::parse(voltages);
        std::cout<< V_meas<< std::endl;
        workQueue.push(V_meas);
    }

    // Get the real power subscription
    if (sub_P.isUpdated())
    {
        sub_P.getString(power_real);
        P_meas = json::parse(power_real);
        std::cout <<P_meas << std::endl;
        workQueue.push(P_meas);
    }

    // Get the reactive power subscription
    if (sub_Q.isUpdated())
    {
        sub_Q.getString(power_imag);
        Q_meas = json::parse(power_imag);
        std::cout <<Q_meas << std::endl;
        workQueue.push(Q_meas);
    }

    }


    void setupMeasurements() {               //measurement ids are loaded here (Done)
        //Adding meas ids
        std::cout << "number of meas id here" << "\n\n";
        std::cout << std::size(meas_zids) << "\n\n";

        for (int i = 0; i < V_meas["ids"].size(); i++) {
            string meas_id = V_meas["ids"][i];
            meas_id = "V_" + meas_id;
            meas_zids.push_back(meas_id);
        }
        std::cout << std::size(meas_zids) << "\n\n";
        for (int i = 0; i < P_meas["ids"].size(); i++) {
            string meas_id = P_meas["ids"][i];
            meas_id = "P_" + meas_id;
            meas_zids.push_back(meas_id);
        }
        std::cout << std::size(meas_zids) << "\n\n";
        for (int i = 0; i < Q_meas["ids"].size(); i++) {
            string meas_id = Q_meas["ids"][i];
            meas_id = "Q_" + meas_id;
            meas_zids.push_back(meas_id);
        }
        std::cout << "number of meas id" << "\n\n";
        std::cout << std::size(meas_zids) << "\n\n";

        SLIST node_names_tmp;
        for (int i = 0; i < topo["base_voltage_magnitudes"]["ids"].size(); i++) {
            string node_name_new = topo["base_voltage_magnitudes"]["ids"][i];
            //std::cout << node_name_new << "\n\n";
            node_names_tmp.push_back(node_name_new);   //a list
        }

        for ( auto& node : node_names_tmp ) {
            //std::cout << "inside" << "\n\n";
            // Check for SOURCEBUS
            if (( node ==  topo["slack_bus"][0].get<string>()) || ( node ==  topo["slack_bus"][1].get<string>()) || ( node ==  topo["slack_bus"][2].get<string>())) {
                //std::cout << "inside-1" << "\n\n";
                string meas_id = "source_V_"+node;
                meas_zids.push_back(meas_id);

                meas_id = "source_T_"+node;
                meas_zids.push_back(meas_id);
            }else{

                // Add the P and Q injection
                string meas_id = "pseudo_P_"+node;
                meas_zids.push_back(meas_id);

                meas_id = "pseudo_Q_"+node;
                meas_zids.push_back(meas_id);
            }
        }
        std::cout << "number of meas id after appending pseudo meas" << "\n\n";
        std::cout << std::size(meas_zids) << "\n\n";
        //_________________________________________________________________
    }


    void fillTopo() {    //reads nodes list and y matrix info (Done)

        //std::cout << std::setw(4) << topo["unique_ids"].size()<< "\n\n";
        for (int i = 0; i < topo["base_voltage_magnitudes"]["ids"].size(); i++) {
            string node_name_new = topo["base_voltage_magnitudes"]["ids"][i];
            //std::cout << node_name_new << "\n\n";
            node_names.push_back(node_name_new);   //a list
        }
        std::cout << node_names.size() << "\n\n";
        //___________________________________________________________________
        if (!sparse_impl)
        {
            std::cout<< "Not a sparse matrix implementation" << std::endl;
            for (int i = 0; i < topo["admittance"]["admittance_matrix"].size(); i++) {
                //std::cout<< i << std::endl;
                for (int j = 0; j < topo["admittance"]["admittance_matrix"][i].size(); j++) {
                    //std::cout<< j << std::endl;
                    double G = (topo["admittance"]["admittance_matrix"][i][j][0]);
                    double B = (topo["admittance"]["admittance_matrix"][i][j][1]);
                    //std::cout<< G << std::endl;
                    //std::cout<< B << std::endl;

                    if ((G!=0) || (B!=0)){
                    Yphys[i+1][j+1] = complex<double>(G,B);
                    if ( (i+1) != (j+1) ) Yphys[j+1][i+1] = complex<double>(G,B);
                    //std::cout<< Yphys[i+1][j+1] << std::endl;
                    }
                }
            }
        }
        else
        {
            std::cout<< "Sparse matrix implementation" << std::endl;
            //____________________ now for sparse implementation:
            for (int a = 0; a < topo["admittance"]["admittance_list"].size(); a++) {
                int i = 0;
                int i_locked = 0;
                int j = 0;
                int j_locked = 0;
                for ( auto& node : node_names ) {
                    if (node ==  topo["admittance"]["from_equipment"][a].get<string>()){
                        i_locked = i;
                    }else{
                        i++;
                    }
                }
                for ( auto& node : node_names ) {
                    if (node ==  topo["admittance"]["to_equipment"][a].get<string>()){
                        j_locked = j;
                    }else{
                        j++;
                    }
                }
                //std::cout<< "i locked is" << std::endl;
                //std::cout<< i_locked << std::endl;
                //std::cout<< "j locked is" << std::endl;
                //std::cout<< j_locked << std::endl;
                double G = (topo["admittance"]["admittance_list"][a][0]);
                double B = (topo["admittance"]["admittance_list"][a][1]);
                //std::cout<< G << std::endl;
                //std::cout<< B << std::endl;
                //need to find i and j?


                if ((G!=0) || (B!=0)){
                Yphys[i_locked+1][j_locked+1] = complex<double>(G,B);
                if ( (i_locked+1) != (j_locked+1) ) Yphys[j_locked+1][i_locked+1] = complex<double>(G,B);
                //std::cout<< Yphys[i+1][j+1] << std::endl;
                }

            }
        }
        //for (int l = 1; l < 5; l++) {
        //    for (int m = 1; m < 5; m++) {
        //        std::cout<< Yphys[l][m] << std::endl;
        //    }
        //}
        //std::cout<< "Last enteries:" << std::endl;

    }


    void fillVnoms() 
    {
        std::cout << "Filling Vnoms" << std::endl;
        //Vnoms are loaded here (Done)
        for (int i = 0; i < topo["base_voltage_magnitudes"]["ids"].size(); i++) 
        {
            //std::cout<< i << std::endl;
            string node = topo["base_voltage_magnitudes"]["ids"][i];
            //std::cout<< node << std::endl;
            double mag = (topo["base_voltage_magnitudes"]["values"][i]);
            //std::cout<< mag << std::endl;
            double arg = (topo["base_voltage_angles"]["values"][i]);
            //std::cout<< arg << std::endl;
            double vre = mag * cos( arg );
            double vim = mag * sin( arg );

            complex<double> vnom = complex<double>(vre,vim);
            //std::cout<< node << std::endl;
            //std::cout<< vnom << std::endl;
            node_vnoms[node] = vnom;      //a complex with both mag and angle
        }
        //__________________________________________________________________
    }


    void fillSensors() {           //loads regulators and initializes sensors (Done without regulators)
        //uint zctr = 0;
        uint zctr = Zary.zids.size();
        std::cout<< Zary.zids.size() << std::endl;
        for (int i = 0; i < V_meas["ids"].size(); i++) {
            string node, zid;
            node = V_meas["ids"][i];
            zid = "V_" + node;
            Zary.zids.push_back(zid);
            Zary.zidxs[zid] = zctr++;
            Zary.ztypes[zid] = "vi";
            Zary.znode1s[zid] = node;
            Zary.znode2s[zid] = node;
            Zary.zvals[zid] = (V_meas["values"][i].get<double>())/std::abs(node_vnoms[node]);
            //std::cout<< Zary.zvals[zid] << std::endl;
            Zary.zsigs[zid] = 0.001;
            Zary.zpseudos[zid] = false;
            Zary.znomvals[zid] = (V_meas["values"][i].get<double>())/std::abs(node_vnoms[node]);
        }
        for (int i = 0; i < P_meas["ids"].size(); i++) {
            string node, zid;
            node = P_meas["ids"][i];
            zid = "P_" + node;
            Zary.zids.push_back(zid);
            Zary.zidxs[zid] = zctr++;
            Zary.ztypes[zid] = "Pi";
            Zary.znode1s[zid] = node;
            Zary.znode2s[zid] = node;
            Zary.zvals[zid] = -P_meas["values"][i].get<double>()/Sbase;
            //std::cout<< Zary.zvals[zid] << std::endl;
            Zary.zsigs[zid] = 0.001;
            Zary.zpseudos[zid] = false;
            Zary.znomvals[zid] = -P_meas["values"][i].get<double>()/Sbase;
            //}
        }
        for (int i = 0; i < Q_meas["ids"].size(); i++) {
            string node, zid;
            node = Q_meas["ids"][i];
            zid = "Q_" + node;
            Zary.zids.push_back(zid);
            Zary.zidxs[zid] = zctr++;
            Zary.ztypes[zid] = "Qi";
            Zary.znode1s[zid] = node;
            Zary.znode2s[zid] = node;
            Zary.zvals[zid] = -Q_meas["values"][i].get<double>()/Sbase;
            Zary.zsigs[zid] = 0.001;
            Zary.zpseudos[zid] = false;
            Zary.znomvals[zid] = -Q_meas["values"][i].get<double>()/Sbase;
            //}
        }

        //inserting pseudo meas
        //uint zctr = Zary.zids.size();
        std::cout<< Zary.zids.size() << std::endl;
        for ( auto& node : node_names ) {

            // Check for SOURCEBUS
            if (( node ==  topo["slack_bus"][0].get<string>()) || ( node ==  topo["slack_bus"][1].get<string>()) || ( node ==  topo["slack_bus"][2].get<string>())) {
                //std::cout<< "Source bus detected" << std::endl;
                // Add sourcebus voltage magnitude
                string vmag_zid = "source_V_"+node;
                Zary.zids.push_back(vmag_zid);
                Zary.zidxs   [vmag_zid] = zctr++;
                Zary.ztypes  [vmag_zid] = "vi";
                Zary.znode1s [vmag_zid] = node;
                Zary.znode2s [vmag_zid] = node;
                Zary.zvals   [vmag_zid] = 1.00;
                Zary.zsigs   [vmag_zid] = 0.001; // 1 sigma = 0.1%
                Zary.zpseudos[vmag_zid] = true;
                Zary.znomvals[vmag_zid] = Zary.zvals[vmag_zid];

                string varg_zid = "source_T_"+node;
                Zary.zids.push_back(varg_zid);
                Zary.zidxs   [varg_zid] = zctr++;
                Zary.ztypes  [varg_zid] = "Ti";
                Zary.znode1s [varg_zid] = node;
                Zary.znode2s [varg_zid] = node;
                Zary.zvals   [varg_zid] = 0.0;
                Zary.zsigs   [varg_zid] = 0.001;
                Zary.zpseudos[varg_zid] = true;
                Zary.znomvals[varg_zid] = Zary.zvals[varg_zid];
            }else{

                // Add the P and Q injections
                string pinj_zid = "pseudo_P_"+node;
                Zary.zids.push_back(pinj_zid);
                Zary.zidxs   [pinj_zid] = zctr++;
                Zary.ztypes  [pinj_zid] = "Pi";
                Zary.znode1s [pinj_zid] = node;
                Zary.znode2s [pinj_zid] = node;
                double val=0;
                for (int i = 0; i < topo["injections"]["power_real"]["ids"].size(); i++) {
                    if (topo["injections"]["power_real"]["ids"][i] == node){
                        val = (topo["injections"]["power_real"]["values"][i].get<double>())/Sbase;
                    }
                }
                //std::cout << "value assigned is: " << std::endl;
                //std::cout << val << std::endl;
                Zary.zvals   [pinj_zid] = val;
                Zary.zsigs   [pinj_zid] = 0.001; // load + leakage
                Zary.zpseudos[pinj_zid] = true;
                Zary.znomvals[pinj_zid] = Zary.zvals[pinj_zid];

                string qinj_zid = "pseudo_Q_"+node;
                Zary.zids.push_back(qinj_zid);
                Zary.zidxs   [qinj_zid] = zctr++;
                Zary.ztypes  [qinj_zid] = "Qi";
                Zary.znode1s [qinj_zid] = node;
                Zary.znode2s [qinj_zid] = node;

                double val_q=0;
                double val_q_adj=0;
                for (int i = 0; i < topo["injections"]["power_imaginary"]["ids"].size(); i++) {
                    if (topo["injections"]["power_imaginary"]["ids"][i] == node){
                        val_q = (topo["injections"]["power_imaginary"]["values"][i].get<double>())/Sbase;
                    }
                }

                Zary.zvals   [qinj_zid] = val_q;
                Zary.zsigs   [qinj_zid] = 0.001;
                Zary.zpseudos[qinj_zid] = true;
                Zary.znomvals[qinj_zid] = Zary.zvals[qinj_zid];

            }
        }
        //_________________________________________________________________
        std::cout<< Zary.zids.size() << std::endl;
    }


    bool fillMeasurement() {         //Todo --------how to form workqueue
        //_________________________________________________________________
        bool ret = true;
		
		if (currenttime <= Total_ts) 
        {
			std::cout <<currenttime << std::endl;
			

			// currenttime = vfed->requestTime(10000);

			V_message = workQueue.pop();
			json P_message = workQueue.pop();
			json Q_message = workQueue.pop();

			std::cout<< V_message<< std::endl;
			std::cout<< P_message << std::endl;
			std::cout<< Q_message<< std::endl;
			
			json V_meas_sim, P_meas_sim, Q_meas_sim;

			sub_V.getString(voltages);
            V_meas_sim = json::parse(voltages);
			workQueue.push(V_meas_sim);

            sub_P.getString(power_real);
			P_meas_sim = json::parse(power_real);
			workQueue.push(P_meas_sim);

			sub_Q.getString(power_imag);
			Q_meas_sim = json::parse(power_imag);
			workQueue.push(Q_meas_sim);

			meas_timestamp = 0;
			meas_mrids.clear();
			meas_magnitudes.clear();

			meas_timestamp = (uint)(ts++);

			string zid;
			uint idx = 0;
			for (int i = 0; i < V_message["ids"].size(); i++) {
				string node = V_message["ids"][i];
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = (V_message["values"][i].get<double>())/std::abs(node_vnoms[node]);
				std::cout<< meas_magnitudes[zid] << std::endl;
			}
			for (int i = 0; i < P_message["ids"].size(); i++) {
				string node = P_message["ids"][i];
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(P_message["values"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
				//}
			}
			for (int i = 0; i < Q_message["ids"].size(); i++) {
				string node = Q_message["ids"][i];
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(Q_message["values"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
				//}
			}

			for (int i = idx; i < meas_zids.size(); i++) {
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				//std::cout<< zid << std::endl;
				//std::cout<< Zary.zvals[zid] << std::endl;
				meas_magnitudes[zid] = Zary.zvals[zid];
			}
			//json jmessage = {{"values": [2331.1810216005406, 2331.177966421088, 2331.1820742949385, 2331.18084349885, 2331.1835662485505, 2331.1798562677645, 2381.9393143056645, 2337.641125001467, 2336.9341773469864, 2378.2509734375412, 2396.4406617953928, 2353.331346272425, 2395.961453668316, 2381.036946740476, 2360.722474155403, 2392.648790835949, 2373.501098608352, 2379.93682210568, 2378.346000463625, 2376.967574841636, 2349.2078132322504, 2389.905247243767, 2387.085927210393, 2389.278916926626, 2346.1243153625237, 2331.1812819873217, 2331.1812819873217, 2355.127661312042, 2346.116927404546, 2342.5676565324743, 2353.034195764937, 2315.390149808388, 2331.1812819873217, 2331.1812819873217, 2341.1690212568865, 2341.8761840103325, 2352.2034613578344, 2350.121088861682, 2351.239502794415, 2312.6220172715066, 2331.1812819873217, 2376.607551417662, 2331.1812819873217, 2310.8016056755414, 2373.7324299695083, 2331.1812819873217, 2331.1812819873217, 2334.328210572143, 2331.291347150265, 2319.760834305269, 2329.4942324045337, 2327.181289554684, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.498880357235, 2311.380773777244, 2382.55630840594, 2331.897870801268, 2310.8429132271076, 2331.1812819873217, 2330.252719568887, 2308.0930968829575, 2311.0810934303654, 2379.3911972836972, 2331.0227013401636, 2311.6348237481725, 2375.154606383472, 2330.0651029875635, 2327.850383950822, 2332.7665325731077, 2329.8422719426617, 2329.426897017492, 2309.5128901020717, 2340.722307491062, 2307.5433831648274, 2369.9485091105857, 2334.436580840001, 2319.8403120168755, 2329.5323122389805, 2334.133875966737, 2305.8824763482153, 2368.569933402147, 2367.6722949955556, 2333.241074923122, 2302.936634347165, 2366.0409952543732, 2327.6097080449726, 2363.276883440126, 2300.6152529105902, 2364.633441598301, 2330.2015839820824, 2299.5056375695294, 2298.4040998758237, 2362.153551715768, 2327.9105070044598, 2298.6717141987174, 2297.7271546514407, 2361.4910923046123, 2327.36976291029, 2297.9516570683872, 2360.999310986141, 2327.1915201298752, 2297.917694558643, 2361.051141272759, 2326.4658009651544, 2297.575882738358, 2339.042170113563, 2340.0398439664186, 2312.6483368521613, 2328.7551846934102, 2327.976881839204, 2323.779486974636, 2326.0473698815535, 2330.84402812044, 2319.140239845763, 2329.6455635989705, 2330.3727197526077, 2311.3261599914467, 2378.901875917682, 2343.977703008032, 2316.32931501523, 2378.855514189581, 2345.8135099002034, 2331.174576354315, 2372.934476366311, 2331.1791320225175, 2321.541212332582, 2378.482945297808, 2347.570159524856, 2371.548900423016, 2300.939316791403, 2328.422247225463, 2338.8417923660454, 2370.8609957699914, 2331.1998430477943, 2331.215780340877, 2331.18435900845, 2299.1016662926995, 2360.2204883926465, 2326.813836475773, 2298.5255226131258, 2358.615353793654, 2324.8546670002834, 2298.833565354412, 2354.7255431274216, 2320.2946464550537, 2296.381507737209, 2347.969794484053, 2322.010310515198, 2308.542340574059, 2339.335426724242, 2310.766442970448, 2297.9555758371444, 2295.29095996434, 2331.1812819873217, 2331.1812819873217, 2303.619629238685, 2328.2454551727146, 2327.201289706514, 2300.104203736526, 2328.1125261815237, 2324.2009553128087, 2291.496435579253, 2288.806002089317, 2287.278287709761, 2322.5882568497714, 2299.077566062031, 2350.4838512581814, 2347.234670882775, 2318.6657694316787, 2316.421617618298, 2316.686864027016, 2360.900054452813, 2328.1078381219895, 2299.1419318750623, 2354.7339707348096, 2327.377209960633, 2303.9189912782244, 2365.6908972151828, 2331.429069666845, 2301.858864767967, 2335.6114380793265, 2337.322163484772, 2307.1588744089604, 2365.9955598663632, 2333.69012855327, 2308.3615344477867, 2366.5225303074462, 2334.2279094710366, 2310.7761505932217, 2332.2831762550586, 2337.968792312761, 2328.6278557703167, 2316.807635154498, 2329.574543985928, 2337.9292408814904, 2325.991055681877, 2296.3913226024415, 2353.305754679529, 2333.364677087108, 2296.3047807769394, 2294.3738152916585, 2352.8096180174734, 2337.0800277409, 2352.7364141280405, 2331.1812819873217, 2331.1812819873217, 2334.748089679497, 2334.6481401803167, 2294.323101357748, 2350.9305700143045, 2332.4266479157563, 2292.7930097882777, 2329.1311160931637, 2350.9209064773745, 2331.2917095655184, 2350.360617376423, 2295.6473967410766, 2350.11586208121, 2340.9232885356532, 2309.470394007791, 2356.5730386994114, 2328.386679124302, 2336.634130305057, 2354.5522056848545, 2324.108917351827, 2330.9243613308868, 2330.3625528246243, 2328.7218201160354, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2324.956467570193, 2321.8290904638093, 2331.1812819873217, 2357.3873412368134, 2331.1812819873217, 2318.205045009066, 2314.318669710248, 2354.890164727071, 2289.420805487797, 2331.1812819873217, 2331.1812819873217, 2351.7172152945946, 2282.136417319964, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2278.9314742042875, 2277.355625507721, 2277.9386100647716, 2275.2280912863716, 2274.3291159662945, 2319.95723739389, 2329.281047575761, 2329.7974271814787, 2321.0270358459616, 2328.960000923116, 2331.418118261922, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2300.9389908748317, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 2331.1812819873217, 268.98245561392173, 268.98245561392173, 268.98245561392173], "ids": ["150.1", "150.2", "150.3", "150R.1", "150R.2", "150R.3", "149.1", "149.2", "149.3", "1.1", "1.2", "1.3", "2.2", "3.3", "7.1", "7.2", "7.3", "4.3", "5.3", "6.3", "8.1", "8.2", "8.3", "12.2", "9.1", "13.1", "13.2", "13.3", "9R.1", "14.1", "34.3", "18.1", "18.2", "18.3", "11.1", "10.1", "15.3", "16.3", "17.3", "19.1", "21.1", "21.2", "21.3", "20.1", "22.2", "23.1", "23.2", "23.3", "24.3", "25.1", "25.2", "25.3", "25R.1", "25R.3", "26.1", "26.3", "28.1", "28.2", "28.3", "27.1", "27.3", "31.3", "33.1", "29.1", "29.2", "29.3", "30.1", "30.2", "30.3", "250.1", "250.2", "250.3", "32.3", "35.1", "35.2", "36.1", "36.2", "35.3", "40.1", "40.2", "40.3", "37.1", "38.2", "39.2", "41.3", "42.1", "42.2", "42.3", "43.2", "44.1", "44.2", "44.3", "45.1", "47.1", "47.2", "47.3", "46.1", "48.1", "48.2", "48.3", "49.1", "49.2", "49.3", "50.1", "50.2", "50.3", "51.1", "51.2", "51.3", "151.1", "151.2", "151.3", "52.1", "52.2", "52.3", "53.1", "53.2", "53.3", "54.1", "54.2", "54.3", "55.1", "55.2", "55.3", "57.1", "57.2", "57.3", "56.1", "56.2", "56.3", "58.2", "60.1", "60.2", "60.3", "59.2", "61.1", "61.2", "61.3", "62.1", "62.2", "62.3", "63.1", "63.2", "63.3", "64.1", "64.2", "64.3", "65.1", "65.2", "65.3", "66.1", "66.2", "66.3", "67.1", "68.1", "67.2", "67.3", "72.1", "72.2", "72.3", "97.1", "97.2", "97.3", "69.1", "70.1", "71.1", "73.3", "76.1", "76.2", "76.3", "74.3", "75.3", "77.1", "77.2", "77.3", "86.1", "86.2", "86.3", "78.1", "78.2", "78.3", "79.1", "79.2", "79.3", "80.1", "80.2", "80.3", "81.1", "81.2", "81.3", "82.1", "82.2", "82.3", "84.3", "83.1", "83.2", "83.3", "85.3", "87.1", "87.2", "87.3", "88.1", "89.1", "89.2", "89.3", "90.2", "91.1", "91.2", "91.3", "92.3", "93.1", "93.2", "93.3", "94.1", "95.1", "95.2", "95.3", "96.2", "98.1", "98.2", "98.3", "99.1", "99.2", "99.3", "100.1", "100.2", "100.3", "450.1", "450.2", "450.3", "197.1", "197.2", "197.3", "101.1", "101.2", "101.3", "102.3", "105.1", "105.2", "105.3", "103.3", "104.3", "106.2", "108.1", "108.2", "108.3", "107.2", "109.1", "300.1", "300.2", "300.3", "110.1", "111.1", "112.1", "113.1", "114.1", "135.1", "135.2", "135.3", "152.1", "152.2", "152.3", "160R.1", "160R.2", "160R.3", "160.1", "160.2", "160.3", "61S.1", "61S.2", "61S.3", "300_OPEN.1", "300_OPEN.2", "300_OPEN.3", "94_OPEN.1", "610.1", "610.2", "610.3"], "units": "kV", "equipment_type": null, "accuracy": null, "bad_data_threshold": null, "time": "2017-01-01T00:15:00"}};
			/*json jmessage;
			jmessage["values"] = {2331.1810216005406, 2331.177966421088, 2331.1820742949385};
			jmessage["ids"] = {"150.1", "150.2", "150.3"};
			jmessage["time"] = "2017-01-01T00:15:00";
			jmessage["units"] =  "kV";
			pub_Vmag.publish(jmessage.dump());
			
			currenttime = vfed->requestTime(10000);*/
		} 
        
        else
        {
			//if (workQueue.empty()){
			ret = false;
			vfed->finalize();
			delete vfed;
			helicsCloseLibrary();
		}
		//vfed->finalize();
		//delete vfed;
		//helicsCloseLibrary();
		//}
		return ret;
    }


    bool nextMeasurementWaiting() {
        // Always returning false tells the SE work loop to complete an
        // estimate for every measurement. Otherwise it does a single estimate
        // over the entire measurements_data.csv file!
        return false;
    }


    void setupPublishing() {
        std::cout << "Setting up publishing" << std::endl;
        string filename = FILE_INTERFACE_READ;
        filename += "/results_data.csv";
        est_fh.open(filename, std::ofstream::trunc);
        est_fh << "timestamp,";

        for ( auto& node_name : node_names )
            est_fh << "vmag_"+node_name+",";
        uint node_qty = node_names.size();
        uint nctr = 0;
        for ( auto& node_name : node_names )
            est_fh << "varg_"+node_name << ( ++nctr < node_qty ? "," : "\n" );

        est_fh.close();
    }


    void publishEstimate(const uint& timestamp,
        SDMAP& est_v, SDMAP& est_angle, SDMAP&, SDMAP&, SDMAP& est_vmagpu, SDMAP& est_vargpu) {

        string filename = FILE_INTERFACE_READ;
        filename += "/results_data.csv";
        est_fh.open(filename, std::ofstream::app);

        est_fh << timestamp << ',';
        est_fh << std::fixed;
        est_fh << std::setprecision(10);
		std::list<double> est_volt;
		std::list<double> est_ang;
        for ( auto& node_name : node_names ){
            est_fh << est_vmagpu[node_name] << ",";
			//std::cout<< "-------------------------------" << std::endl;
			//std::cout<< est_v[node_name] << std::endl;
			//std::cout<< node_name << std::endl;
			est_volt.push_back(est_v[node_name]);
			est_ang.push_back(est_angle[node_name]);
		}
        uint node_qty = node_names.size();
        uint nctr = 0;
        for ( auto& node_name : node_names )
            est_fh << est_vargpu[node_name] << ( ++nctr < node_qty ? "," : "\n" );

        est_fh.close();
		time_t now = time(0);
   
		// convert now to string form
		char* dt = ctime(&now);
		json jmessage_vmag;
		jmessage_vmag["values"] = est_volt;//{2331.1810216005406, 2331.177966421088, 2331.1820742949385};
		jmessage_vmag["ids"] = node_names;//{"150.1", "150.2", "150.3"};
		jmessage_vmag["time"] = V_message["time"];//std::to_string(currenttime);//"2017-01-01T00:15:00";
		jmessage_vmag["units"] =  "kV";
		pub_Vmag.publish(jmessage_vmag.dump());
		
		json jmessage_vang;
		jmessage_vang["values"] = est_ang;//{2331.1810216005406, 2331.177966421088, 2331.1820742949385};
		jmessage_vang["ids"] = node_names;//{"150.1", "150.2", "150.3"};
		jmessage_vang["time"] = V_message["time"];//"2017-01-01T00:15:00";
		jmessage_vang["units"] =  "deg";
		pub_Vang.publish(jmessage_vang.dump());
			
		currenttime = vfed->requestTime(10000);
    }


    std::vector<string> getZids() {
        return meas_zids;
    }


    string getOutputDir() {
        return FILE_INTERFACE_READ;
    }

private:
    std::ifstream meas_fh;
    std::ofstream est_fh;
    std::vector<string> meas_zids;
	
    json topo;
	json P_meas;
	json Q_meas;
	json V_meas;
	
    SharedQueue<json> workQueue;
	double Sbase;
	double ts;
	bool sparse_impl = false;
	
    helicscpp::Publication pub_Vmag;
	helicscpp::Publication pub_Vang;
	
    helicscpp::ValueFederate* vfed;
	
    HelicsTime currenttime;
    
    helicscpp::Input sub_topo;
	helicscpp::Input sub_P;
	helicscpp::Input sub_Q;
	helicscpp::Input sub_V;
	
    std::string topology;
    std::string power_real;
    std::string power_imag;
    std::string voltages;
	
    SLIST node_est_v;
	
    int Total_ts = 97;
	
    json V_message;

    json inputMap;
    json staticInput;
};

