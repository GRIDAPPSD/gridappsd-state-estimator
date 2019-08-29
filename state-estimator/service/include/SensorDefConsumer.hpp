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


#include "SensorArray.hpp"
//// Store node names in a linked list and hash node name to their position
//// Iterate over the linked list to access all nodes or states
//// Note that positions are one-indexed
//// Store sensor names in a linked list and hash node names to various params
//#define SLIST std::list<std::string>
////#define SIMAP std::unordered_map<std::string,unsigned int>
//#define SBMAP std::unordered_map<std::string,bool>
//#define SDMAP std::unordered_map<std::string,double>
//#define SSMAP std::unordered_map<std::string,std::string>

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
	private:
	SensorArray zary;
//	SLIST mmids;
//	SLIST zids;		// measurement id [mrid_ztype] [list of strings]
//	SSMAP ztypes;	// measurement types [str->str]
//	SDMAP zsigs;	// measurement sigma: standard deviation [str->double]
//	SSMAP znode1s;	// point node or from node for flow measurements [str->str]
//	SSMAP znode2s;	// point node or to node for flow measurements [str->str]
//	SDMAP zvals;	// value of the latest measurement [str->double]
//	SBMAP znew;		// indicator for new measurement [str->bool]
//	uint zqty = 0;	// number of measurements
//	// to add to z
//	//	-- zids.push_back(zid);
//	//	-- ztypes[zid] = ztype;
//	//	-- ssigs[zid] = zsig;
//	//	-- snd1s[zid] = znode1;
//	//	-- snd2s[sn] = znode2;
//	//	-- svals[sn] = zval;
	
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
	void fillSens(SensorArray &zary) {
		zary = this->zary;
//		zqty = this->zqty;
//		zids = this->zids;
//		ztypes = this->ztypes;
//		zsigs = this->zsigs;
//		znode1s = this->znode1s;
//		znode2s = this->znode2s;
//		zvals = this->zvals;
	}
	
	public:
	virtual void process() {
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND INITIALIZE SENSORS
		// --------------------------------------------------------------------
		cout << "Recieved sensor message of " << text.length() << " bytes...\n\t";

		json jtext = json::parse(text);
		cout << jtext.dump().substr(0,2000) << " ...\n\n";
//		cout << jtext.dump(2);

		for ( auto& feeder : jtext["data"]["feeders"] ) {
			cout << "\nFeeder name: " << feeder["name"] << '\n';
//			cout << "\tmRID: " << feeder["mRID"] << '\n';
//			cout << "\tsubstation: " << feeder["substation"] << '\n';
//			cout << "\tsubstationID: " << feeder["substationID"] << '\n';
//			cout << "\tsubregion: " << feeder["subregion"] << '\n';
//			cout << "\tsubregionID: " << feeder["subregionID"] << '\n';
//			cout << "\tregion: " << feeder["region"] << '\n';
//			cout << "\tregionID: " << feeder["regionID"] << '\n';

			vector<string> objs = {"capacitors","switches"};
			for ( string& type : objs ) {
				cout << '\t' << type << ": \n";
				json objs = feeder[type];
				unsigned int ctr = 0;
				for ( auto& obj : objs ) {
					cout << "\t\t" << obj["name"] << '\n';
				}
			}
			

			for ( auto& reg : feeder["regulators"] ) {
				cout << reg.dump() + '\n';
			}

//			cout << "\nPress ENTER to list non-PNV measurements:";
//			while ( cin.get()!='\n' );
			
		}
/*
		// --------------------------------------------------------------------
		// LOAD THE SENSORS -- sensors will deliver measurements
		// --------------------------------------------------------------------
		// Iterate over the sensors
		for ( auto& f : jtext["data"]["feeders"] ) {
			for ( auto& m : f["measurements"] ) {
				cout << m.dump()+'\n';
				// store the necessary measurement information
				string mmrid = m["mRID"];
				string tmeas = m["measurementType"];
				zary.mmrids.push_back( mmrid );
				zary.mtypes[mmrid] = tmeas;
				cout << mmrid << " -> " << tmeas << '\n';

				// build z and supporting structures
				if ( !tmeas.compare("PNV") ) {

					// The node is [bus].[phase_num];
					string node = m["ConnectivityNode"];
					for ( auto& c : node ) c = std::toupper(c);
					string phase = m["phases"];
					if ( !phase.compare("A") ) node += ".1";
					if ( !phase.compare("B") ) node += ".2";
					if ( !phase.compare("C") ) node += ".3";
					if ( !phase.compare("s1") ) node += ".1";	// secondary
					if ( !phase.compare("s2") ) node += ".2";	// secondary

					// add the voltage magnitude measurement
					string zid = mmrid + "_Vmag";
					zary.zids.push_back( zid );
					zary.zidxs[zid] = zary.zqty++;
					zary.ztypes[zid] = "vi";
					zary.znode1s[zid] = node;
					zary.znode2s[zid] = node;
					zary.zsigs[zid] = 0.0001;		// WHERE DOES THIS COME FROM ??
					// these don't necessarily need to be initialized
					// - they will be initialized on access
//					zary.zvals[zid] = 0;		// initialize to 0
//					zary.znew[zid] = false;		// initial measurement is not "new"

					// add the voltage phase measurement
					// --- LATER ---
					// -------------

				} else {
					// we only care about PNV measurements for now
				}
			}
		}
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
