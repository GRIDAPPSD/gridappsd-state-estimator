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

#ifndef SLIST
#define SLIST std::list<std::string>
#endif

#ifndef SSLISTMAP
#define SSLISTMAP std::unordered_map<std::string,SLIST>
#endif

//#ifndef IDMAP
//#define IDMAP std::unordered_map<unsigned int,double>
//#endif

//#ifndef IMDMAP
//#define IMDMAP std::unordered_map<unsigned int,IDMAP>
//#endif

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
    private:
    SSMAP term_bus_map; // terminal_mrid -> bus_name
    SSLISTMAP cemrid_busnames_map; // ce_mrid -> bus_names

    private:
    SSMAP reg_cemrid_primbus_map;  // for regulator Pos measurement init
    SSMAP reg_cemrid_regbus_map;   // for regulator Pos measurement init
    
	private:
	SensorArray zary;
    SSMAP mmrid_pos_type_map;
//	SLIST mmids;
//	SLIST zids;		// measurement id [mrid_ztype] [list of strings]
//	SSMAP ztypes;	// measurement types [str->str]
//	SDMAP zsigs;	// measurement sigma: standard deviation [str->double]
//	SSMAP znode1s;	// point node or from node for flow measurements [str->str]
//	SSMAP znode2s;	// point node or to node for flow measurements [str->str]
//	SDMAP zvals;	// value of the latest measurement [str->double]
//	SIMAP znew;		// counter for new measurement [str->uint]
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
//                const SSMAP& term_bus_map,
                const SSLISTMAP& cemrid_busnames_map,
                const SSMAP& reg_cemrid_primbus_map,
                const SSMAP& reg_cemrid_regbus_map,
				const string& target,
				const string& mode) {
		this->brokerURI = brokerURI;
		this->username = username;
		this->password = password;
//        this->term_bus_map = term_bus_map;
        this->cemrid_busnames_map = cemrid_busnames_map;
        this->reg_cemrid_primbus_map = reg_cemrid_primbus_map;
        this->reg_cemrid_regbus_map = reg_cemrid_regbus_map;
		this->target = target;
		this->mode = mode;
	}

	public:
	void fillSens(SensorArray &zary, SSMAP& mmrid_pos_type_map) {
		zary = this->zary;
        mmrid_pos_type_map = this->mmrid_pos_type_map;
	}
	
	public:
	virtual void process() {
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND INITIALIZE SENSORS
		// --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
		*selog << "Received sensor message of " << text.length() << " bytes\n\n" << std::flush;
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
                    // TODO switch to sensor service uncertainty
					zary.zsigs[zid] = 0.01;	// 1 sigma = 1%
                    zary.zvals[zid] = 1.0;

					// add the voltage phase measurement
					// --- LATER ---
					// -------------

				} else if ( !tmeas.compare("Pos") ) {
                    string ce_type = m["ConductingEquipment_type"];
                    if ( !ce_type.compare("PowerTransformer") ) {
                        // regulator tap measurement
                        mmrid_pos_type_map[mmrid] = "regulator_tap";

                        // look up the prim and reg nodes
                        string cemrid = m["ConductingEquipment_mRID"];
                        string primbus = reg_cemrid_primbus_map[cemrid];
                        string regbus = reg_cemrid_regbus_map[cemrid];
                        //string primnode = regid_primnode_map[cemrid];
                        //string regnode = regid_regnode_map[cemrid];

                        string phase = m["phases"];
                        string primnode = primbus;
                        string regnode = regbus;
			            if (!phase.compare("A")) { primnode += ".1"; regnode += ".1"; }
			            if (!phase.compare("B")) { primnode += ".2"; regnode += ".2"; }
			            if (!phase.compare("C")) { primnode += ".3"; regnode += ".3"; }
			            if (!phase.compare("s1")) { primnode += ".1"; regnode += ".1"; }
			            if (!phase.compare("s2")) { primnode += ".2"; regnode += ".2"; }

                        // add the position measurement 
                        string zid = mmrid + "_tap";
                        zary.zids.push_back( zid );
                        zary.zidxs[zid] = zary.zqty++;
                        zary.ztypes[zid] = "aji";
                        zary.znode1s[zid] = primnode;
                        zary.znode2s[zid] = regnode;
                        //zary.zsigs[zid] = 0.0000625; // 1% of 1 tap
                        zary.zsigs[zid] = 0.001; // 1% of span
                        zary.zvals[zid] = 1.0;

//                        *selog << m.dump(2);
//                        *selog << "primnode: " << primnode << std::endl;
//                        *selog << "regnode: " << regnode << std::endl;
#if 000
                    } else if ( !ce_type.compare("LoadBreakSwitch") ) {
                        // START HERE see TODO below and get rid of debug logging
#ifdef DEBUG_PRIMARY
                        *selog << "Dumping Switch Ybus measurement\n";
                        *selog << m.dump(2);
                        *selog << "\n" << std::flush;
#endif

                        // regulator tap measurement
                        mmrid_pos_type_map[mmrid] = "load_break_switch";

                        // look up the prim and reg nodes
                        string cemrid = m["ConductingEquipment_mRID"];

                        // add the switch measurement
                        string zid = mmrid + "_switch";
                        zary.zids.push_back( zid );
                        zary.zidxs[zid] = zary.zqty++;
                        zary.ztypes[zid] = "switch_ij";
                        // TODO figure out better confidence value for switches
                        zary.zsigs[zid] = 0.001; // 1% of span
                        zary.zvals[zid] = 1.0;

                        // cemrid_busnames_map[cemrid] contains 2 buses
                        // adjacent to a switch for cemrid
                        string phase = m["phases"];
                        uint node_count = 0;
                        for (auto it=cemrid_busnames_map[cemrid].begin();
                                it!=cemrid_busnames_map[cemrid].end(); ++it) {
                            string node = *it;
                            if (!phase.compare("A")) node += ".1";
                            if (!phase.compare("B")) node += ".2";
                            if (!phase.compare("C")) node += ".3";
                            if (!phase.compare("s1")) node += ".1";
                            if (!phase.compare("s2")) node += ".2";
                            node_count++;
                            if (node_count == 1) {
                                zary.znode1s[zid] = node;
#ifdef DEBUG_PRIMARY
                               *selog << "switch cemrid: " << cemrid << ", zid: " << zid << ", znode1s: " << node << "\n" << std::endl;
#endif
                            } else if (node_count == 2) {
                                zary.znode2s[zid] = node;
#ifdef DEBUG_PRIMARY
                               *selog << "switch cemrid: " << cemrid << ", zid: " << zid << ", znode2s: " << node << "\n" << std::endl;
#endif
                            }
                        }
#endif
                    } else {
                        mmrid_pos_type_map[mmrid] = "other";
                    }
                } else {
					// we only care about PNV and Pos measurements for now
				}
			}
		}
        
		
		// --------------------------------------------------------------------
		// SENSOR INITIALIZATION COMPLETE
		// --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
		*selog << "Sensor initialization complete.\n\n" << std::flush;
#endif
		// release latch
		doneLatch.countDown();
	}
};

#endif
