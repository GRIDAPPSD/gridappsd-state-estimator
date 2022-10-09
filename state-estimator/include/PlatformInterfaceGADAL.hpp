#ifndef PLATFORMINTERFACEGADAL_HPP
#define PLATFORMINTERFACEGADAL_HPP

#include <iomanip> // std::setprecision

#define FILE_INTERFACE_READ "test_files_123"

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

#include <iostream>
#include <thread>


class PlatformInterface : public PlatformInterfaceBase {
public:
    PlatformInterface(int argc, char** argv, const double& sbase) : PlatformInterfaceBase(argc, argv, sbase) {
	std::string fedinitstring = "--federates=1";
    double deltat = 1;
	Sbase = sbase;
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
	fi.setCoreName("pnnl_state_estimator");

    fi.setProperty(HELICS_PROPERTY_TIME_DELTA, deltat);

    fi.setProperty(HELICS_PROPERTY_INT_MAX_ITERATIONS, 100);

    fi.setProperty(HELICS_PROPERTY_INT_LOG_LEVEL, HELICS_LOG_LEVEL_WARNING);

    /* Create value federate */
    helicscpp::ValueFederate* vfed = new helicscpp::ValueFederate("pnnl_state_estimator", fi);
    std::cout << " Value federate created\n";
	
	helicscpp::Input sub_topo = vfed->registerSubscription("local_feeder/topology","");
    
	//helicscpp::Input sub_P = vfed->registerSubscription("sensors/power_real","W");
	helicscpp::Input sub_P = vfed->registerSubscription("sensor_power_real/publication","W");
	
	//helicscpp::Input sub_Q = vfed->registerSubscription("sensors/power_imag","W");
	helicscpp::Input sub_Q = vfed->registerSubscription("sensor_power_imaginary/publication","W");
	
	//helicscpp::Input sub_V = vfed->registerSubscription("sensors/voltages","V");
	helicscpp::Input sub_V = vfed->registerSubscription("sensor_voltage_magnitude/publication","V");
	
    if(sub_topo.isValid()){
        std::cout << " Subscription registered for feeder Topology\n";
	}
	
	if(sub_P.isValid()){
        std::cout << " Subscription registered for feeder real power P\n";
	}
	
	if(sub_Q.isValid()){
        std::cout << " Subscription registered for feeder reactive power Q\n";
	}
	
	if(sub_V.isValid()){
        std::cout << " Subscription registered for feeder voltage V\n";
	}
	
	helicscpp::Publication pub_Vmag = vfed->registerGlobalPublication("Vmag_SE", HELICS_DATA_TYPE_DOUBLE);
	
	helicscpp::Publication pub_Vang = vfed->registerGlobalPublication("Vang_SE", HELICS_DATA_TYPE_DOUBLE);
	
    if(pub_Vmag.isValid()){
		std::cout << " Vmag Publication registered\n";
	}
    if(pub_Vang.isValid()){
		std::cout << " Vang Publication registered\n";
	}
	
	//std::ifstream ifs("input_mapping.json");
	//json jf = json::parse(ifs);
	
	//std::ifstream ifs("static_inputs.json");
	//json jf_si = json::parse(ifs);
	
	//std::cout <<  jf << std::endl;

	 /* Enter initialization state */
    vfed->enterInitializingMode();  // can throw helicscpp::InvalidStateTransition exception
    std::cout << " Entered initialization state" << std::endl;

    /* Enter execution state */
    vfed->enterExecutingMode();  // can throw helicscpp::InvalidStateTransition exception
    std::cout << " Entered execution state\n";
	
	std::string topology;
	std::string power_real;
	std::string power_imag;
	std::string voltages;
	
	HelicsTime currenttime = 0.0;
	ts=1;
	
	currenttime = vfed->requestTime(10000);
	
	sub_topo.getString(topology);
    //std::cout <<topology << std::endl;
	topo = json::parse(topology);
	std::cout <<topo["slack_bus"][0].get<string>() << std::endl;
	std::cout <<topo["slack_bus"][1].get<string>() << std::endl;
	std::cout <<topo["slack_bus"][2].get<string>() << std::endl;
	std::cout <<topo << std::endl;
	
	sub_P.getString(power_real);
	P_meas = json::parse(power_real);
	//std::cout <<P_meas << std::endl;
	
	sub_Q.getString(power_imag);
	Q_meas = json::parse(power_imag);
	//std::cout <<Q_meas << std::endl;
	
	sub_V.getString(voltages);
	V_meas = json::parse(voltages);
	//std::cout <<V_meas["array"] << std::endl;
	//std::cout <<V_meas["unique_ids"] << std::endl;
	
	workQueue.push(V_meas);
	workQueue.push(P_meas);
	workQueue.push(Q_meas);
	
	std::cout<< V_meas<< std::endl;
	std::cout<< P_meas << std::endl;
	std::cout<< Q_meas<< std::endl;

	while (currenttime < 10000) {
		std::cout <<currenttime << std::endl;
		json V_meas_sim, P_meas_sim, Q_meas_sim;
		
		//if (currenttime > 90) {
		sub_P.getString(power_real);
		P_meas_sim = json::parse(power_real);
		//std::cout <<P_meas_sim << std::endl;
	
		sub_Q.getString(power_imag);
		Q_meas_sim = json::parse(power_imag);
		//std::cout <<Q_meas_sim << std::endl;
	
		sub_V.getString(voltages);
		V_meas_sim = json::parse(voltages);
		//std::cout <<V_meas_sim << std::endl;
		//}
		
		workQueue.push(V_meas_sim);
		workQueue.push(P_meas_sim);
		workQueue.push(Q_meas_sim);
		
		currenttime = vfed->requestTime(10000);
	}
	
	vfed->finalize();
    std::cout << "NLIN2: Federate finalized" << std::endl;
    // Destructor for ValueFederate must be called before close library
    //helicscpp::cleanupHelicsLibrary();
	delete vfed;
    helicsCloseLibrary();
    std::cout << "NLIN2: Library Closed" << std::endl;
	
    }


    void setupMeasurements() {               //measurement ids are loaded here (Done)
        string filename = FILE_INTERFACE_READ;
        filename += "/measurement_data_1.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading measurements from test harness file: " << filename << "\n\n" << std::flush;
#endif
        meas_fh.open(filename);

        // header line is an ordered list of zids
        // parse measurement file header to get the zids into an STL vector
        string meas_line;
        getline(meas_fh, meas_line);
        string cell;
        std::stringstream headerStream(meas_line);
        getline(headerStream, cell, ','); // throwaway timestamp header token
        //while ( getline(headerStream, cell, ',') )
        //   meas_zids.push_back(cell);
		
		//_______________________________________________________________
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
		for (int i = 0; i < topo["admittance"]["ids"].size(); i++) {
			string node_name_new = topo["admittance"]["ids"][i];
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
        string filename = FILE_INTERFACE_READ;
        filename += "/nodelist.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading nodelist from test harness file: " << filename <<
            "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: nodelist file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        while ( getline(ifs, line) ) {
            // Extract the node name
            string node_name = regex_replace(line,regex("\""),"");
            // Store the node information
            //node_names.push_back(node_name);   //a list
        }
        ifs.close();
		
		//_______________________________________________________________
		
		//std::cout << std::setw(4) << topo["unique_ids"].size()<< "\n\n";
		for (int i = 0; i < topo["admittance"]["ids"].size(); i++) {
			string node_name_new = topo["admittance"]["ids"][i];
			//std::cout << node_name_new << "\n\n";
			node_names.push_back(node_name_new);   //a list
		}
		std::cout << node_names.size() << "\n\n";
		//___________________________________________________________________

        filename = FILE_INTERFACE_READ;
        filename += "/ysparse.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading ybus from test harness file: " << filename <<
            "\n\n" << std::flush;
#endif
        ifs.open(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: ysparse file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        getline(ifs, line);  // throwaway header line
        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell;
            getline(lineStream, cell, ','); int i = std::stoi(cell);
            getline(lineStream, cell, ','); int j = std::stoi(cell);
            getline(lineStream, cell, ','); double G = std::stod(cell);
            getline(lineStream, cell, ','); double B = std::stod(cell);

            //Yphys[i][j] = complex<double>(G,B);      //dictionary, only contains entries that are non zero. (only take non-zero entries out of y_matrix and assign to this disctionary)
            //if ( i != j ) Yphys[j][i] = complex<double>(G,B);
			//std::cout<< Yphys[i][j] << std::endl;
        }
        ifs.close();
		
		//_______________________________________________________________
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
		//__________________________________________________________________
		
		
    }


    void fillVnoms() {     //Vnoms are loaded here (Done)
#ifdef FILE_INTERFACE_VNOM
        string filename = FILE_INTERFACE_READ;
        filename += "/vnom.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading vnom from test harness file: " << filename <<
            "\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: vnom file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line);  // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string node, cell;
            getline(lineStream, node, ',');
            getline(lineStream, cell, ','); double mag = std::stod(cell);
            getline(lineStream, cell, ','); double arg = std::stod(cell);
            double vre = mag * cos( arg * M_PI/180.0 );
            double vim = mag * sin( arg * M_PI/180.0 );
            complex<double> vnom = complex<double>(vre,vim);
			//std::cout<< node << std::endl;
			//std::cout<< vnom << std::endl;
            //node_vnoms[node] = vnom;      //a complex with both mag and angle. 
        }
        ifs.close();
		
		//_______________________________________________________________
		for (int i = 0; i < topo["base_voltage_magnitudes"]["ids"].size(); i++) {
			//std::cout<< i << std::endl;
			string node = topo["base_voltage_magnitudes"]["ids"][i];
			//std::cout<< node << std::endl;
			double mag = (topo["base_voltage_magnitudes"]["values"][i]);
			//std::cout<< mag << std::endl;
			double arg = (topo["base_voltage_angles"]["values"][i]);
			//std::cout<< arg << std::endl;
			double vre = mag * cos( arg /180.0 );
            double vim = mag * sin( arg /180.0 );
			
			complex<double> vnom = complex<double>(vre,vim);
			//std::cout<< node << std::endl;
			//std::cout<< vnom << std::endl;
            node_vnoms[node] = vnom;      //a complex with both mag and angle. 
		}
		//__________________________________________________________________
#else
        for ( auto& node_name : *node_names_ref )
            node_vnoms[node_name] = 1;
#endif
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
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
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
			}
		}
		for (int i = 0; i < Q_meas["ids"].size(); i++) {
			string node, zid;
			node = Q_meas["ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
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
			}
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
                Zary.zvals   [pinj_zid] = val/Sbase;
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
				for (int i = 0; i < topo["injections"]["power_imaginary"]["ids"].size(); i++) {
					if (topo["injections"]["power_imaginary"]["ids"][i] == node){
						val_q = (topo["injections"]["power_imaginary"]["values"][i].get<double>())/Sbase;
					}
				}
				
                Zary.zvals   [qinj_zid] = val_q/Sbase;
                Zary.zsigs   [qinj_zid] = 0.001; // load + leakage
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
		
		json V_message = workQueue.pop();
		json P_message = workQueue.pop();
		json Q_message = workQueue.pop();
		
		std::cout<< V_message<< std::endl;
		std::cout<< P_message << std::endl;
		std::cout<< Q_message<< std::endl;

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
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(P_message["values"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
			}
		}
		for (int i = 0; i < Q_message["ids"].size(); i++) {
			string node = Q_message["ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(Q_message["values"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
			}
		}
		
		for (int i = idx; i < meas_zids.size(); i++) {
			zid = meas_zids[idx++];
			meas_mrids.push_back(zid);
			//std::cout<< zid << std::endl;
			//std::cout<< Zary.zvals[zid] << std::endl;
			meas_magnitudes[zid] = Zary.zvals[zid];
		}
		
		if (workQueue.empty()){
			ret = false;
        }
		return ret;
    }


    bool nextMeasurementWaiting() {
        // Always returning false tells the SE work loop to complete an
        // estimate for every measurement. Otherwise it does a single estimate
        // over the entire measurements_data.csv file!
        return false;
    }


    void setupPublishing() {
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
        SDMAP&, SDMAP&, SDMAP&, SDMAP&, SDMAP& est_vmagpu, SDMAP& est_vargpu) {

        string filename = FILE_INTERFACE_READ;
        filename += "/results_data.csv";
        est_fh.open(filename, std::ofstream::app);

        est_fh << timestamp << ',';
        est_fh << std::fixed;
        est_fh << std::setprecision(10);

        for ( auto& node_name : node_names )
            est_fh << est_vmagpu[node_name] << ",";
        uint node_qty = node_names.size();
        uint nctr = 0;
        for ( auto& node_name : node_names )
            est_fh << est_vargpu[node_name] << ( ++nctr < node_qty ? "," : "\n" );

        est_fh.close();
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
};

#endif
