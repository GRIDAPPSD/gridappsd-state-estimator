#ifndef SENSORDEFCONSUMER_HPP
#define SENSORDEFCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "SEConsumer.hpp"

// standard data types
#include <string>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int


#include "SensorArray.hpp"

#ifndef SSMAP
#define SSMAP std::unordered_map<std::string,std::string>
#endif

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
    private:
    SSMAP term_bus_map; // terminal_mrid -> bus_name
    
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
                const SSMAP& term_bus_map,
				const string& target,
				const string& mode) {
		this->brokerURI = brokerURI;
		this->username = username;
		this->password = password;
        this->term_bus_map = term_bus_map;
		this->target = target;
		this->mode = mode;
	}

	public:
	void fillSens(SensorArray &zary) {
		zary = this->zary;
	}
	
	public:
	virtual void process() {
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND INITIALIZE SENSORS
		// --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
		cout << "Received sensor message of " << text.length() << " bytes...\n\n" << std::flush;
#endif

		json jtext = json::parse(text);

		// --------------------------------------------------------------------
		// LOAD THE SENSORS -- sensors will deliver measurements
		// --------------------------------------------------------------------
		// Iterate over the sensors
		for ( auto& f : jtext["data"]["feeders"] ) {
			for ( auto& m : f["measurements"] ) {
				// store the necessary measurement information
				string mmrid = m["mRID"];
				string tmeas = m["measurementType"];
				zary.mmrids.push_back( mmrid );
				zary.mtypes[mmrid] = tmeas;

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
					zary.zsigs[zid] = 0.01;		// WHERE DOES THIS COME FROM ??
                    // uncertanty should come from the sensor service -- in that case
                    // it won't need to be initialized

					// add the voltage phase measurement
					// --- LATER ---
					// -------------

				} else if ( !tmeas.compare("Pos") ) {
                    string ce_type = m["ConductingEquipment_type"];
                    if ( !ce_type.compare("PowerTransformer") ) {
                        // regulator tap measurement
                        string node = term_bus_map[m["Terminal_mRID"]];
                        for ( auto& c : node ) c = std::toupper(c);
                        string phase = m["phases"];
                        if ( !phase.compare("A") ) node += ".1";
                        if ( !phase.compare("B") ) node += ".2";
                        if ( !phase.compare("C") ) node += ".3";
                        if ( !phase.compare("s1") ) node += ".1";	// secondary
                        if ( !phase.compare("s2") ) node += ".2";	// secondary

                        // add the position measurement 
//                        string zid = mmrid + "_tap";
//                        zary.zids.push_back( zid );
//                        zary.zidxs[zid] = zary.zqty++;
//                        zary.ztypes[zid] = "aji";
//                        zary.znode1s[zid] = node;
//                        zary.znode2s[zid] = node;
//                        zary.zsigs[zid] = 0.0000625;
                    }
                } else {
					// we only care about PNV measurements for now
				}
			}
		}
        
		
		// --------------------------------------------------------------------
		// SENSOR INITIALIZATION COMPLETE
		// --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
		cout << "Sensor initialization complete.\n\n" << std::flush;
#endif
		// release latch
		doneLatch.countDown();
	}
};

#endif
