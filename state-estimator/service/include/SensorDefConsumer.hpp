#ifndef SENSORDEFCONSUMER_HPP
#define SENSORDEFCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "SEConsumer.hpp"

// standard data types
#include <string>
//#include <complex>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// Store node names in a linked list and hash node name to their position
// Iterate over the linked list to access all nodes or states
// Note that positions are one-indexed
// Store sensor names in a linked list and hash node names to various params
#define SLIST std::list<std::string>
//#define SIMAP std::unordered_map<std::string,unsigned int>
#define SDMAP std::unordered_map<std::string,double>
#define SSMAP std::unordered_map<std::string,std::string>

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
	private:
	SLIST sns;		// sensor name [list of strings]
	SSMAP sts;		// sensor type [sn->str]
	SDMAP ssigs;	// sensor sigma: standard deviation [sn->double]
	SSMAP snd1s;	// point node or from node for flow sensors [sn->str]
	SSMAP snd2s;	// point node or to node for flow sensors [sn->str]
	SDMAP svals;	// value of the latest measurement [sn->double]
	uint numms = 0; // number of sensors
	// to add a sensor
	//	-- sns.push_back(sn);
	//	-- sts[sn] = st;
	//	-- ssigs[sn] = ssig;
	//	-- snd1s[sn] = snd1;
	//	-- snd2s[sn] = snd2;
	//	-- svals[sn] = sval;
	
	public:
	SensorDefConsumer(const string& brokerURI, 
				const string& username,
				const string& password,
				const string& target,
				const string& mode) {
		this->brokerURI = brokerURI;
		this->username = username;
		this->password = password;
		this->target = target;
		this->mode = mode;
	}

	public:
	void fillSens(uint& numms, SLIST& sns, SSMAP& sts, SDMAP& ssigs,
			SSMAP& snd1s, SSMAP& snd2s, SDMAP& svals) {
		numms = this->numms;
		sns = this->sns;
		sts = this->sts;
		ssigs = this->ssigs;
		snd1s = this->snd1s;
		snd2s = this->snd2s;
		svals = this->svals;
	}
	
	public:
	virtual void process(const string& text) {
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND INITIALIZE SENSORS
		// --------------------------------------------------------------------
		cout << "Recieved sensor message:\n\t";

		json jtext = json::parse(text);
		cout << jtext.dump().substr(0,1000) << " ...\n\n";

		cout << "\nPress ENTER to parse:";
		while ( cin.get() != '\n' );

		json jdata = json::parse(jtext["data"].dump());
		json feeders = jdata["feeders"];
		for ( auto itr = feeders.begin() ; itr != feeders.end() ; itr++ ) {
			json feeder = *itr;
			cout << "\nFeeder name: " << feeder["name"] << '\n';
			cout << "\tmRID: " << feeder["mRID"] << '\n';
			cout << "\tsubstation: " << feeder["substation"] << '\n';
			cout << "\tsubstationID: " << feeder["substationID"] << '\n';
			cout << "\tsubregion: " << feeder["subregion"] << '\n';
			cout << "\tsubregionID: " << feeder["subregionID"] << '\n';
			cout << "\tregion: " << feeder["region"] << '\n';
			cout << "\tregionID: " << feeder["regionID"] << '\n';

			vector<string> objs = {"capacitors","switches"};
			for ( auto jtr = objs.begin() ; jtr != objs.end() ; jtr++ ) {
				string type = *jtr;

				cout << "\nPress ENTER to list " + type + ": ";
				while ( cin.get() != '\n' );

				cout << '\t' << type << ": \n";
				json objs = feeder[type];
				unsigned int ctr = 0;
				for ( auto ktr = objs.begin() ; ktr != objs.end() ; ktr++ ) {
					json obj = *ktr;
					cout << "\t\t" << obj["name"] << '\n';
				}
			}

			cout << "\nPress ENTER to list non-PNV measurements:";
			while ( cin.get()!='\n' );
			
			cout << "\tmeasurements:\n";
			unsigned int measctr = 0, pnvctr = 0;
			for ( auto jtr = feeder["measurements"].begin() ; 
					jtr != feeder["measurements"].end() ; jtr++ ) {
				measctr++;
				string meas = (*jtr)["name"];
				string node = (*jtr)["ConnectivityNode"];
				string tmeas = (*jtr)["measurementType"];
				if ( !tmeas.compare("PNV") ) pnvctr++;
				else cout<<"\t\t"<<(*jtr)["name"]<< " at node "<<node<<" is type "<<tmeas<<'\n';
			}
			cout << "\tNumber of measurements: " << measctr << '\n';
			cout << "\tNumber of PNV measurements: " << pnvctr << '\n';
		}


//		// if the message is what we are looking for, process and release
//		string fn = "demo/13Bus/static_measurements.csv";
//		cout << "Loading sensor definitions from file: " << fn << "\n";
//		cout << "\t...\n";
		
/*		// --------------------------------------------------------------------
		// LOAD THE SENSORS
		// --------------------------------------------------------------------
		// for now, read the sensor objects from a file
		std::ifstream sfs;
		sfs.open(fn,std::ifstream::in);
		if ( !sfs ) throw "failed to open static measurement file";
		std::string sfs1;
		std::getline(sfs,sfs1);
		std::string st,sn,snd1,snd2;
		double sval,ssig;
		std::cout << sfs1;
		while ( sfs >> st >> sn >> snd1 >> snd2 >> sval >> ssig ) {
			sns.push_back(sn);
			numms++;
			sts[sn] = st;
			ssigs[sn] = ssig;
			snd1s[sn] = snd1;
			snd2s[sn] = snd2;
			svals[sn] = sval;
		}
		// END read the sensor objects from a file
*/		
		
		// --------------------------------------------------------------------
		// SENSOR INITIALIZATION COMPLETE
		// --------------------------------------------------------------------
		cout << "\nSensor initialization complete.\n";
		// release latch
		doneLatch.countDown();
	}
};

#endif
