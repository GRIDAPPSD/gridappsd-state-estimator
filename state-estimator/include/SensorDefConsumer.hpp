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
    SDMAP node_nominal_Pinj_map;
    SDMAP node_nominal_Qinj_map;
    
    private:
    SensorArray zary;
    SSMAP mmrid_pos_type_map;
    SSMAP switch_node1s;
    SSMAP switch_node2s;

    private:
    double sbase;
    
    public:
    SensorDefConsumer(const string& brokerURI, 
                const string& username,
                const string& password,
//                const SSMAP& term_bus_map,
                const SSLISTMAP& cemrid_busnames_map,
                const SSMAP& reg_cemrid_primbus_map,
                const SSMAP& reg_cemrid_regbus_map,
                const SDMAP& node_nominal_Pinj_map,
                const SDMAP& node_nominal_Qinj_map,
                const double& sbase,
                const string& target,
                const string& mode) {
        this->brokerURI = brokerURI;
        this->username = username;
        this->password = password;
//        this->term_bus_map = term_bus_map;
        this->cemrid_busnames_map = cemrid_busnames_map;
        this->reg_cemrid_primbus_map = reg_cemrid_primbus_map;
        this->reg_cemrid_regbus_map = reg_cemrid_regbus_map;
        this->node_nominal_Pinj_map = node_nominal_Pinj_map;
        this->node_nominal_Qinj_map = node_nominal_Qinj_map;
        this->sbase = sbase;
        this->target = target;
        this->mode = mode;
    }

    public:
    void fillSens(SensorArray &zary, SSMAP& mmrid_pos_type_map,
                  SSMAP& switch_node1s, SSMAP& switch_node2s) {
        zary = this->zary;
        mmrid_pos_type_map = this->mmrid_pos_type_map;
        switch_node1s = this->switch_node1s;
        switch_node2s = this->switch_node2s;
    }
    
    public:
    virtual void process() {
        // --------------------------------------------------------------------
        // PARSE THE MESSAGE AND INITIALIZE SENSORS
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        *selog << "Received sensor message of " << text.length() << " bytes\n\n" << std::flush;
#endif

#ifdef FILES_INTERFACE_WRITE
        std::ofstream ofs("test_files/measurements.csv", ofstream::out);
        ofs << "ztype,zid,znode1,znode2,zval,zsig,zpseudo,znomval\n";
#endif

        json jtext = json::parse(text);
        //cout << jtext.dump(2) << endl;

#ifdef DEBUG_PRIMARY
        *selog << "Parsing sensors -- " << std::flush;
#endif

        // --------------------------------------------------------------------
        // LOAD THE SENSORS -- sensors will deliver measurements
        // --------------------------------------------------------------------
        // Iterate over the sensors
        for ( auto& f : jtext["data"]["feeders"] ) {
            for ( auto& m : f["measurements"] ) {

                // store the necessary measurement information
                string mmrid = m["mRID"];
                string tmeas = m["measurementType"];
                string ce_type = m["ConductingEquipment_type"];
                zary.mmrids.push_back( mmrid );
                zary.mtypes[mmrid] = tmeas;
                zary.mcetypes[mmrid] = ce_type;

                // The node is [bus].[phase_num];
                string meas_node = m["ConnectivityNode"];
                for ( auto& c : meas_node ) c = std::toupper(c);
                string phase = m["phases"];
                if ( !phase.compare("A") ) meas_node += ".1";
                if ( !phase.compare("B") ) meas_node += ".2";
                if ( !phase.compare("C") ) meas_node += ".3";
                if ( !phase.compare("s1") ) meas_node += ".1";    // secondary
                if ( !phase.compare("s2") ) meas_node += ".2";    // secondary
                zary.mnodes[mmrid] = meas_node;

                // build z and supporting structures
                if ( !tmeas.compare("PNV") ) {

                    // add the voltage magnitude measurement
                    string zid = mmrid + "_Vmag";
                    zary.zids.push_back( zid );
                    zary.zidxs[zid] = zary.zqty++;
                    zary.ztypes[zid] = "vi";
                    zary.znode1s[zid] = meas_node;
                    zary.znode2s[zid] = meas_node;
                    // TODO use sensor service uncertainty when implemented
                    zary.zsigs[zid] = 0.01;    // 1 sigma = 1%
                    zary.zvals[zid] = 1.0;
                    zary.znomvals[zid] = zary.zvals[zid];
#ifdef FILES_INTERFACE_WRITE
                    ofs << zary.ztypes[zid] << "," << zid << "," << zary.znode1s[zid] << "," << zary.znode2s[zid] << "," << zary.zvals[zid] << "," << zary.zsigs[zid] << ",0," << zary.znomvals[zid] << "\n";
#endif


                    // add the voltage phase measurement
                    // --- LATER ---
                    // -------------

                } else if ( !tmeas.compare("Pos") ) {
                    if ( !ce_type.compare("PowerTransformer") ) {
                        // regulator tap measurement
                        // TODO: use zary.mcetypes instead of mmrid_pos_type_map
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
                        zary.znomvals[zid] = zary.zvals[zid];
#ifdef FILES_INTERFACE_WRITE
                        ofs << zary.ztypes[zid] << "," << zid << "," << zary.znode1s[zid] << "," << zary.znode2s[zid] << "," << zary.zvals[zid] << "," << zary.zsigs[zid] << ",0," << zary.znomvals[zid] << "\n";
#endif

//                        *selog << m.dump(2);
//                        *selog << "primnode: " << primnode << std::endl;
//                        *selog << "regnode: " << regnode << std::endl;
                    } else if ( !ce_type.compare("LoadBreakSwitch") ) {
                        // TODO: use zary.mcetypes instead of mmrid_pos_type_map
                        mmrid_pos_type_map[mmrid] = "load_break_switch";
                        string cemrid = m["ConductingEquipment_mRID"];
                        string zid = mmrid + "_switch";

                        // cemrid_busnames_map[cemrid] contains 2 buses
                        // adjacent to a switch for cemrid
                        string phase = m["phases"];
                        uint switch_node_count = 0;
                        for (auto it=cemrid_busnames_map[cemrid].begin();
                                it!=cemrid_busnames_map[cemrid].end(); ++it) {
                            string switch_node = *it;
                            if (!phase.compare("A")) switch_node += ".1";
                            if (!phase.compare("B")) switch_node += ".2";
                            if (!phase.compare("C")) switch_node += ".3";
                            if (!phase.compare("s1")) switch_node += ".1";
                            if (!phase.compare("s2")) switch_node += ".2";
                            switch_node_count++;
                            if (switch_node_count == 1) {
                                switch_node1s[zid] = switch_node;
                                //*selog << "switch cemrid: " << cemrid << ", zid: " << zid << ", znode1s: " << node << "\n" << std::endl;
                            } else if (switch_node_count == 2) {
                                switch_node2s[zid] = switch_node;
                                //*selog << "switch cemrid: " << cemrid << ", zid: " << zid << ", znode2s: " << node << "\n" << std::endl;
                                break; // no reason to keep checking
                            }
                        }
                    } else {
                        mmrid_pos_type_map[mmrid] = "other";
                    }
                } else if ( !tmeas.compare("VA") ) {
#ifdef NET_INJECTION
                    // TODO: figure out whether to create another structure to
                    // track many physical measurement to one state measurement
                    // mapping
                    // Relevant component types in CIM dictionary:
                    //     energyconsumers
                    //     synchronousmachines
                    //     solarpanels

                    if (!ce_type.compare("EnergyConsumer")) {
                        string pinj_zid = meas_node + "_Pinj";
                         string qinj_zid = meas_node + "_Qinj";

                        // instead of poor performing n^2 complexity find,
                        // we could create a map from node name to aggregate
                        // injection while processing CIM dictionary and then
                        // after processing add these to zary
                        if (std::find(zary.zids.begin(),zary.zids.end(),pinj_zid) == zary.zids.end()) {
                            // add the real power injection measurement
                            zary.zids.push_back( pinj_zid );
                            zary.zidxs[pinj_zid] = zary.zqty++;
                            zary.ztypes[pinj_zid] = "Pi";
                            zary.znode1s[pinj_zid] = meas_node;
                            zary.znode2s[pinj_zid] = meas_node;

                            // assumes pinj and qinj are only added together
                            // allowing a second find() call to be eliminated
                            // add the reactive power injection measurement
                            zary.zids.push_back( qinj_zid );
                            zary.zidxs[qinj_zid] = zary.zqty++;
                            zary.ztypes[qinj_zid] = "Qi";
                            zary.znode1s[qinj_zid] = meas_node;
                            zary.znode2s[qinj_zid] = meas_node;
                        }

                        // use nominal load for node from SPARQL query for zvals

                        zary.zvals[pinj_zid] -= node_nominal_Pinj_map[meas_node]/(2.0*sbase);
                        double zsig_Pinj = node_nominal_Pinj_map[meas_node]*0.01/sbase;
                        zary.zsigs[pinj_zid] = sqrt(zary.zsigs[pinj_zid]*zary.zsigs[pinj_zid] + zsig_Pinj*zsig_Pinj);    // 1 sigma = 1% of nominal
                        zary.znomvals[pinj_zid] += zary.zvals[pinj_zid];

                        zary.zvals[qinj_zid] -= node_nominal_Qinj_map[meas_node]/(2.0*sbase);
                        double zsig_Qinj = node_nominal_Qinj_map[meas_node]*0.01/sbase;
                        zary.zsigs[qinj_zid] = sqrt(zary.zsigs[qinj_zid]*zary.zsigs[qinj_zid] + zsig_Qinj*zsig_Qinj);    // 1 sigma = 1% of nominal
                        zary.znomvals[qinj_zid] += zary.zvals[qinj_zid];

#ifdef FILES_INTERFACE_WRITE
                        ofs << zary.ztypes[pinj_zid] << "," << pinj_zid << "," << zary.znode1s[pinj_zid] << "," << zary.znode2s[pinj_zid] << "," << zary.zvals[pinj_zid] << "," << zary.zsigs[pinj_zid] << ",0," << zary.znomvals[pinj_zid] << "\n";
                        ofs << zary.ztypes[qinj_zid] << "," << qinj_zid << "," << zary.znode1s[qinj_zid] << "," << zary.znode2s[qinj_zid] << "," << zary.zvals[qinj_zid] << "," << zary.zsigs[qinj_zid] << ",0," << zary.znomvals[qinj_zid] << "\n";
#endif
#ifdef COMPARE_INJ_MEAS
                        zary.injnodes.push_back( meas_node );
#endif
                    } else if (!ce_type.compare("LinearShuntCompensator")) {
                    }
                    // other injection equipment types we will handle include
                    // PV systems and synchronous machines
                    // there are also flow types include ACLineSegment (and
                    // probably also transformers)
                    else {
                    }
#endif
                } else {
                    // we only care about PNV, Pos, and VA measurements for now
                }
            }
        }
#ifdef FILES_INTERFACE_WRITE
        ofs.close();
#endif

        // --------------------------------------------------------------------
        // SENSOR INITIALIZATION COMPLETE
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        *selog << "complete.\n\n" << std::flush;
#endif
        // release latch
        doneLatch.countDown();
    }
};

#endif
