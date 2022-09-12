#ifndef PLATFORMINTERFACEGADAL_HPP
#define PLATFORMINTERFACEGADAL_HPP

#include <iomanip> // std::setprecision

//#define FILE_INTERFACE_READ "test_4Ti"
//#define FILE_INTERFACE_READ "test_3p6_Ti"
//#define FILE_INTERFACE_READ "test_4PiQi"
//#define FILE_INTERFACE_READ "test_13assets_noaji"
//#define FILE_INTERFACE_READ "test_11big_jl"
//#define FILE_INTERFACE_READ "test_4"
//#define FILE_INTERFACE_READ "test_4vinj"
//#define FILE_INTERFACE_READ "test_4net"
//#define FILE_INTERFACE_READ "test_4sbase"
//#define FILE_INTERFACE_READ "test_13assets"
//#define FILE_INTERFACE_READ "test_11full"
//#define FILE_INTERFACE_READ "test_11diff"
//#define FILE_INTERFACE_READ "test_11noQ"
//#define FILE_INTERFACE_READ "test_4withB"
//#define FILE_INTERFACE_READ "test_4woB"
//#define FILE_INTERFACE_READ "test_11big"
//#define FILE_INTERFACE_READ "test_3p6"
//#define FILE_INTERFACE_READ "test_3p6pseudo"
//#define FILE_INTERFACE_READ "test_11_bus_full"
//#define FILE_INTERFACE_READ "test_11_bus_diff"
//#define FILE_INTERFACE_READ "test_11_bus_full_meas"
//#define FILE_INTERFACE_READ "test_11_bus_diff_meas"
//#define FILE_INTERFACE_READ "test_4_bus_full"
//#define FILE_INTERFACE_READ "test_4_bus_diff"
//#define FILE_INTERFACE_READ "test_3p6_bus_full"
//#define FILE_INTERFACE_READ "test_3p6_bus_diff"
//#define FILE_INTERFACE_READ "test_3p6_bus_full_meas"
//#define FILE_INTERFACE_READ "test_3p6_bus_diff_meas"
//#define FILE_INTERFACE_READ "test_files_13assets"
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

    fi.setProperty(HELICS_PROPERTY_TIME_DELTA, deltat);

    fi.setProperty(HELICS_PROPERTY_INT_MAX_ITERATIONS, 100);

    fi.setProperty(HELICS_PROPERTY_INT_LOG_LEVEL, HELICS_LOG_LEVEL_WARNING);

    /* Create value federate */
    helicscpp::ValueFederate* vfed = new helicscpp::ValueFederate("pnnl_state_estimator", fi);
    std::cout << " Value federate created\n";
	
	helicscpp::Input sub_topo = vfed->registerSubscription("local_feeder/topology","");
    
	helicscpp::Input sub_P = vfed->registerSubscription("sensors/power_real","W");
	
	helicscpp::Input sub_Q = vfed->registerSubscription("sensors/power_imag","W");
	
	helicscpp::Input sub_V = vfed->registerSubscription("sensors/voltages","V");
	
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
	
	currenttime = vfed->requestTime(10000);
	
	sub_topo.getString(topology);
    //std::cout <<topology << std::endl;
	topo = json::parse(topology);
	std::cout <<topo["slack_bus"][0].get<string>() << std::endl;
	std::cout <<topo["slack_bus"][1].get<string>() << std::endl;
	std::cout <<topo["slack_bus"][2].get<string>() << std::endl;
    //std::cout << std::setw(4) << topo["unique_ids"] << "\n\n";
	//std::cout << std::setw(4) << topo["y_matrix"] [0][0]<< "\n\n";
	
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
	
		//std::cout<< V_meas_sim<< std::endl;
		//std::cout<< P_meas_sim << std::endl;
		//std::cout<< Q_meas_sim<< std::endl;
		
		
		currenttime = vfed->requestTime(10000);
	}
	
	vfed->finalize();
    std::cout << "NLIN2: Federate finalized" << std::endl;
    // Destructor for ValueFederate must be called before close library
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
		for (int i = 0; i < V_meas["unique_ids"].size(); i++) {
			string meas_id = V_meas["unique_ids"][i];
			meas_id = "V_" + meas_id;
			meas_zids.push_back(meas_id); 
		}
		std::cout << std::size(meas_zids) << "\n\n";
		for (int i = 0; i < P_meas["unique_ids"].size(); i++) {
			string meas_id = P_meas["unique_ids"][i];
			if ((meas_id !="P1UDT942-P1UHS0_1247X.1") && (meas_id !="P1UDT942-P1UHS0_1247X.2") && (meas_id !="P1UDT942-P1UHS0_1247X.3")){
				meas_id = "P_" + meas_id;
				meas_zids.push_back(meas_id); 
			}
		}
		std::cout << std::size(meas_zids) << "\n\n";
		for (int i = 0; i < Q_meas["unique_ids"].size(); i++) {
			string meas_id = Q_meas["unique_ids"][i];
			if ((meas_id !="P1UDT942-P1UHS0_1247X.1") && (meas_id !="P1UDT942-P1UHS0_1247X.2") && (meas_id !="P1UDT942-P1UHS0_1247X.3")){
				meas_id = "Q_" + meas_id;
				meas_zids.push_back(meas_id);
			}				
		}
		std::cout << std::size(meas_zids) << "\n\n";
		meas_zids.push_back("T_" + topo["slack_bus"][0].get<string>());
		meas_zids.push_back("T_" + topo["slack_bus"][1].get<string>());
		meas_zids.push_back("T_" + topo["slack_bus"][2].get<string>());
		std::cout << "number of meas id" << "\n\n";
		std::cout << std::size(meas_zids) << "\n\n";
		//_________________________________________________________________
		/*for (int i = 0; i < topo["unique_ids"].size(); i++) {
			string meas_id = topo["unique_ids"][i];
			meas_id = "V_" + meas_id;
			meas_zids.push_back(meas_id);
		}
		for (int i = 0; i < topo["unique_ids"].size(); i++) {
			string meas_id = topo["unique_ids"][i];
			if ((meas_id !="P1UDT942-P1UHS0_1247X.1") && (meas_id !="P1UDT942-P1UHS0_1247X.2") && (meas_id !="P1UDT942-P1UHS0_1247X.3")){
				meas_id = "P_" + meas_id;
				meas_zids.push_back(meas_id);
			}
		}
		for (int i = 0; i < topo["unique_ids"].size(); i++) {
			string meas_id = topo["unique_ids"][i];
			if ((meas_id !="P1UDT942-P1UHS0_1247X.1") && (meas_id !="P1UDT942-P1UHS0_1247X.2") && (meas_id !="P1UDT942-P1UHS0_1247X.3")){
				meas_id = "Q_" + meas_id;
				meas_zids.push_back(meas_id);
			}
		}
		meas_zids.push_back("T_P1UDT942-P1UHS0_1247X.1");
		meas_zids.push_back("T_P1UDT942-P1UHS0_1247X.2");
		meas_zids.push_back("T_P1UDT942-P1UHS0_1247X.3");
		std::cout << "number of meas id" << "\n\n";
		std::cout << std::size(meas_zids) << "\n\n";*/
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
		for (int i = 0; i < topo["unique_ids"].size(); i++) {
			string node_name_new = topo["unique_ids"][i];
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
		for (int i = 0; i < topo["y_matrix"].size(); i++) {
			//std::cout<< i << std::endl;
			for (int j = 0; j < topo["y_matrix"][i].size(); j++) {
				//std::cout<< j << std::endl;
				double G = (topo["y_matrix"][i][j]["real"]);
				double B = (topo["y_matrix"][i][j]["imag"]);
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
		for (int i = 0; i < topo["base_voltages"].size(); i++) {
			string node = topo["unique_ids"][i];
			double mag = (topo["base_voltages"][i]);
			double arg = (topo["phases"][i]);
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
        /*string filename = FILE_INTERFACE_READ;
        filename += "/regid.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading regulator mappings from test harness file: " <<
            filename << "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: regid file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line); // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string regid, primnode, regnode;
            getline(lineStream, regid, ',');
            getline(lineStream, primnode, ',');
            getline(lineStream, regnode, ',');

            regid_primnode[regid] = primnode;
            regid_regnode[regid] = regnode;

            uint primidx = node_idxs[primnode];
            uint regidx = node_idxs[regnode];
            // initialize the A matrix
            Amat[primidx][regidx] = 1; // this will change
            Amat[regidx][primidx] = 1; // this stays unity and may not be needed
        }
        ifs.close();

        // For the file interface, file is read for all measurements so no need
        // to do anything for pseudo-measurements as with SensorDefConsumer
        filename = FILE_INTERFACE_READ;
        filename += "/measurements_1.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading sensor measurements from test harness file: " <<
            filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: measurements file not found: " <<
                filename << "\n\n" << std::flush;
            exit(0);
        }

        getline(ifs, line); // throwaway header line

        /*uint zctr = 0;
        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell, zid;
            getline(lineStream, cell, ','); // hold this value for ztypes
            getline(lineStream, zid, ','); Zary.zids.push_back(zid);
            Zary.zidxs[zid] = zctr++;
            Zary.ztypes[zid] = cell;
			//std::cout<< zid << std::endl;
            getline(lineStream, cell, ','); Zary.znode1s[zid] = cell;
            getline(lineStream, cell, ','); Zary.znode2s[zid] = cell;
            getline(lineStream, cell, ','); Zary.zvals[zid] = std::stod(cell);
            getline(lineStream, cell, ','); Zary.zsigs[zid] = std::stod(cell);
            getline(lineStream, cell, ','); Zary.zpseudos[zid] = cell=="1";
            getline(lineStream, cell, ','); Zary.znomvals[zid] =std::stod(cell);
        }
        ifs.close();*/
		//_______________________________________________________________
		//std::cout<< V_meas["array"] << std::endl;
		//std::cout<< P_meas["array"] << std::endl;
		//std::cout<< Q_meas["array"] << std::endl;
		uint zctr = 0;
		/*for (int i = 0; i < meas_zids.size(); i++) {
			string node, zid;
			std::cout<< meas_zids[i].substr(2) << std::endl;
			node = meas_zids[i].substr(2);
			zid = meas_zids[i];
			if (meas_zids[i].substr(0,2) == "V_"){
				Zary.zids.push_back(zid);
				Zary.zidxs[zid] = zctr++;
				Zary.znode1s[zid] = node;
				Zary.znode2s[zid] = node;
			
				Zary.ztypes[zid] = "vi";
				Zary.zvals[zid] = 1.02;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = 1.02;
			}
		}*/
		for (int i = 0; i < V_meas["unique_ids"].size(); i++) {
			string node, zid;
			node = V_meas["unique_ids"][i];
			zid = "V_" + node;
			Zary.zids.push_back(zid);
			Zary.zidxs[zid] = zctr++;
			Zary.ztypes[zid] = "vi";
			Zary.znode1s[zid] = node;
			Zary.znode2s[zid] = node;
			Zary.zvals[zid] = (V_meas["array"][i].get<double>())/std::abs(node_vnoms[node]);
			//std::cout<< Zary.zvals[zid] << std::endl;
			Zary.zsigs[zid] = 0.001;
			Zary.zpseudos[zid] = "0";
			Zary.znomvals[zid] = (V_meas["array"][i].get<double>())/std::abs(node_vnoms[node]);
		}
		for (int i = 0; i < P_meas["unique_ids"].size(); i++) {
			string node, zid;
			node = P_meas["unique_ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = "P_" + node;
				Zary.zids.push_back(zid);
				Zary.zidxs[zid] = zctr++;
				Zary.ztypes[zid] = "Pi";
				Zary.znode1s[zid] = node;
				Zary.znode2s[zid] = node;
				Zary.zvals[zid] = -P_meas["array"][i].get<double>()/Sbase;
				std::cout<< Zary.zvals[zid] << std::endl;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = -P_meas["array"][i].get<double>()/Sbase;
			}
		}
		for (int i = 0; i < Q_meas["unique_ids"].size(); i++) {
			string node, zid;
			node = Q_meas["unique_ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = "Q_" + node;
				Zary.zids.push_back(zid);
				Zary.zidxs[zid] = zctr++;
				Zary.ztypes[zid] = "Qi";
				Zary.znode1s[zid] = node;
				Zary.znode2s[zid] = node;
				Zary.zvals[zid] = -Q_meas["array"][i].get<double>()/Sbase;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = -Q_meas["array"][i].get<double>()/Sbase;
			}
		}
		//meas_zids.push_back("T_P1UDT942-P1UHS0_1247X.1");
		string node, zid;
		node = topo["slack_bus"][0].get<string>();
		//node = "P1UDT942-P1UHS0_1247X.1";
		zid = "T_" + node;
		Zary.zids.push_back(zid);
		Zary.zidxs[zid] = zctr++;
		Zary.ztypes[zid] = "Ti";
		Zary.znode1s[zid] = node;
		Zary.znode2s[zid] = node;
		Zary.zvals[zid] = 0;
		Zary.zsigs[zid] = 0.01;
		Zary.zpseudos[zid] = "1";
		Zary.znomvals[zid] = 0;
		
		//node = "P1UDT942-P1UHS0_1247X.2";
		node = topo["slack_bus"][1].get<string>();
		zid = "T_" + node;
		Zary.zids.push_back(zid);
		Zary.zidxs[zid] = zctr++;
		Zary.ztypes[zid] = "Ti";
		Zary.znode1s[zid] = node;
		Zary.znode2s[zid] = node;
		Zary.zvals[zid] = 0;
		Zary.zsigs[zid] = 0.01;
		Zary.zpseudos[zid] = "1";
		Zary.znomvals[zid] = 0;
		
		//node = "P1UDT942-P1UHS0_1247X.3";
		node = topo["slack_bus"][2].get<string>();
		zid = "T_" + node;
		Zary.zids.push_back(zid);
		Zary.zidxs[zid] = zctr++;
		Zary.ztypes[zid] = "Ti";
		Zary.znode1s[zid] = node;
		Zary.znode2s[zid] = node;
		Zary.zvals[zid] = 0;
		Zary.zsigs[zid] = 0.01;
		Zary.zpseudos[zid] = "1";
		Zary.znomvals[zid] = 0;
		//_________________________________________________________________
		//uint zctr = 0;
		/*for (int i = 0; i < meas_zids.size(); i++) {
			string node, zid;
			std::cout<< meas_zids[i].substr(2) << std::endl;
			node = meas_zids[i].substr(2);
			zid = meas_zids[i];
			Zary.zids.push_back(zid);
			Zary.zidxs[zid] = zctr++;
			Zary.znode1s[zid] = node;
			Zary.znode2s[zid] = node;
			if (meas_zids[i].substr(0,2) == "V_"){
				Zary.ztypes[zid] = "vi";
				Zary.zvals[zid] = 1.02;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = 1.02;
			}
			if (meas_zids[i].substr(0,2) == "P_"){
				Zary.ztypes[zid] = "vi";
				Zary.zvals[zid] = 1.02;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = 1.02;
			}
			if (meas_zids[i].substr(0,2) == "Q_"){
				Zary.ztypes[zid] = "vi";
				Zary.zvals[zid] = 1.02;
				Zary.zsigs[zid] = 0.001;
				Zary.zpseudos[zid] = "1";
				Zary.znomvals[zid] = 1.02;
			}
		}*/
    }


    bool fillMeasurement() {         //Todo --------how to form workqueue
        /*bool ret = true;

        meas_timestamp = 0;
        meas_mrids.clear();
        meas_magnitudes.clear();

        string meas_line;
        if ( getline(meas_fh, meas_line) ) {
			//std::cout<< "inside" << std::endl;
            std::stringstream lineStream(meas_line);
            string cell, zid;
            getline(lineStream, cell, ',');
            double doubletime = stod(cell);
            meas_timestamp = (uint)doubletime;

            uint idx = 0;
            while ( getline(lineStream, cell, ',') ) {
                zid = meas_zids[idx++];
                meas_mrids.push_back(zid);
                meas_magnitudes[zid] = stod(cell);
            }
        } else {
#ifdef DEBUG_PRIMARY
            *selog << "Reached end of measurement_data.csv file, normal exit\n" << std::flush;
#endif
            ret = false;
        }*/
		
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
		
		meas_timestamp = (uint)1000;
		
		string zid;
		uint idx = 0;
		for (int i = 0; i < V_message["unique_ids"].size(); i++) {
			string node = V_message["unique_ids"][i];
			zid = meas_zids[idx++];
			meas_mrids.push_back(zid);
			meas_magnitudes[zid] = (V_message["array"][i].get<double>())/std::abs(node_vnoms[node]);
			std::cout<< meas_magnitudes[zid] << std::endl;
		}
		for (int i = 0; i < P_message["unique_ids"].size(); i++) {
			string node = P_message["unique_ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(P_message["array"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
			}
		}
		for (int i = 0; i < Q_message["unique_ids"].size(); i++) {
			string node = Q_message["unique_ids"][i];
			if ((node !="P1UDT942-P1UHS0_1247X.1") && (node !="P1UDT942-P1UHS0_1247X.2") && (node !="P1UDT942-P1UHS0_1247X.3")){
				zid = meas_zids[idx++];
				meas_mrids.push_back(zid);
				meas_magnitudes[zid] = -(Q_message["array"][i].get<double>()/Sbase);
				//std::cout<< meas_magnitudes[zid] << std::endl;
			}
		}
		
		//zid = "T_P1UDT942-P1UHS0_1247X.1";
		zid = "T_" + topo["slack_bus"][0].get<string>();
		meas_mrids.push_back(zid);
		meas_magnitudes[zid] = 0;
		
		//zid = "T_P1UDT942-P1UHS0_1247X.2";
		zid = "T_" + topo["slack_bus"][1].get<string>();;
		meas_mrids.push_back(zid);
		meas_magnitudes[zid] = 0;
		
		//zid = "T_P1UDT942-P1UHS0_1247X.3";
		zid = "T_" + topo["slack_bus"][2].get<string>();;
		meas_mrids.push_back(zid);
		meas_magnitudes[zid] = 0;
		
		//ret = false;
        
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
};

#endif
