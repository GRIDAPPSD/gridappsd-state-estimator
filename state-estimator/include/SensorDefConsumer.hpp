#ifndef SENSORDEFCONSUMER_HPP
#define SENSORDEFCONSUMER_HPP
#ifdef GRIDAPPSD_INTERFACE

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
    private:
    SSMAP term_bus; // terminal_mrid -> bus_name
    SSLISTMAP cemrid_busnames; // ce_mrid -> bus_names

    private:
    SSMAP reg_cemrid_primbus;  // for regulator Pos measurement init
    SSMAP reg_cemrid_regbus;   // for regulator Pos measurement init

    private:
    SDMAP node_nominal_Pinj;
    SDMAP node_nominal_Qinj;
    
    private:
    SensorArray Zary;
    SSMAP mmrid_pos_type;
    SSMAP switch_node1s;
    SSMAP switch_node2s;

    private:
    double sbase;
    
    public:
    SensorDefConsumer(const string& brokerURI, 
                const string& username,
                const string& password,
//                const SSMAP& term_bus,
                const SSLISTMAP& cemrid_busnames,
                const SSMAP& reg_cemrid_primbus,
                const SSMAP& reg_cemrid_regbus,
                const SDMAP& node_nominal_Pinj,
                const SDMAP& node_nominal_Qinj,
                const double& sbase,
                const string& target,
                const string& mode) {
        this->brokerURI = brokerURI;
        this->username = username;
        this->password = password;
//        this->term_bus = term_bus;
        this->cemrid_busnames = cemrid_busnames;
        this->reg_cemrid_primbus = reg_cemrid_primbus;
        this->reg_cemrid_regbus = reg_cemrid_regbus;
        this->node_nominal_Pinj = node_nominal_Pinj;
        this->node_nominal_Qinj = node_nominal_Qinj;
        this->sbase = sbase;
        this->target = target;
        this->mode = mode;
    }

    public:
    void fillSens(SensorArray &Zary, SSMAP& mmrid_pos_type,
                  SSMAP& switch_node1s, SSMAP& switch_node2s) {
        Zary = this->Zary;
        mmrid_pos_type = this->mmrid_pos_type;
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

#ifdef WRITE_FILES
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
                Zary.mmrids.push_back( mmrid );
                Zary.mtypes[mmrid] = tmeas;
                Zary.mcetypes[mmrid] = ce_type;

                // The node is [bus].[phase_num];
                string meas_node = m["ConnectivityNode"];
                for ( auto& c : meas_node ) c = std::toupper(c);
                string phase = m["phases"];
                if ( !phase.compare("A") ) meas_node += ".1";
                if ( !phase.compare("B") ) meas_node += ".2";
                if ( !phase.compare("C") ) meas_node += ".3";
                if ( !phase.compare("s1") ) meas_node += ".1";    // secondary
                if ( !phase.compare("s2") ) meas_node += ".2";    // secondary
                Zary.mnodes[mmrid] = meas_node;

                // build z and supporting structures
                if ( !tmeas.compare("PNV") ) {

                    // add the voltage magnitude measurement
                    string zid = mmrid + "_Vmag";
                    Zary.zids.push_back( zid );
                    Zary.zidxs[zid] = Zary.zqty++;
                    Zary.ztypes[zid] = "vi";
                    Zary.znode1s[zid] = meas_node;
                    Zary.znode2s[zid] = meas_node;
                    // TODO use sensor service uncertainty when implemented
                    Zary.zsigs[zid] = 0.01;    // 1 sigma = 1%
                    Zary.zvals[zid] = 1.0;
                    Zary.znomvals[zid] = Zary.zvals[zid];
#ifdef WRITE_FILES
                    ofs << Zary.ztypes[zid] << "," << zid << "," << Zary.znode1s[zid] << "," << Zary.znode2s[zid] << "," << Zary.zvals[zid] << "," << Zary.zsigs[zid] << ",0," << Zary.znomvals[zid] << "\n";
#endif


                    // add the voltage phase measurement
                    // --- LATER ---
                    // -------------

                } else if ( !tmeas.compare("Pos") ) {
                    if ( !ce_type.compare("PowerTransformer") ) {
                        // regulator tap measurement
                        // TODO: use Zary.mcetypes instead of mmrid_pos_type
                        mmrid_pos_type[mmrid] = "regulator_tap";

                        // look up the prim and reg nodes
                        string cemrid = m["ConductingEquipment_mRID"];
                        string primbus = reg_cemrid_primbus[cemrid];
                        string regbus = reg_cemrid_regbus[cemrid];
                        //string primnode = regid_primnode[cemrid];
                        //string regnode = regid_regnode[cemrid];

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
                        Zary.zids.push_back( zid );
                        Zary.zidxs[zid] = Zary.zqty++;
                        Zary.ztypes[zid] = "aji";
                        Zary.znode1s[zid] = primnode;
                        Zary.znode2s[zid] = regnode;
                        //Zary.zsigs[zid] = 0.0000625; // 1% of 1 tap
                        Zary.zsigs[zid] = 0.001; // 1% of span
                        Zary.zvals[zid] = 1.0;
                        Zary.znomvals[zid] = Zary.zvals[zid];
#ifdef WRITE_FILES
                        ofs << Zary.ztypes[zid] << "," << zid << "," << Zary.znode1s[zid] << "," << Zary.znode2s[zid] << "," << Zary.zvals[zid] << "," << Zary.zsigs[zid] << ",0," << Zary.znomvals[zid] << "\n";
#endif

//                        *selog << m.dump(2);
//                        *selog << "primnode: " << primnode << std::endl;
//                        *selog << "regnode: " << regnode << std::endl;
                    } else if ( !ce_type.compare("LoadBreakSwitch") ) {
                        // TODO: use Zary.mcetypes instead of mmrid_pos_type
                        mmrid_pos_type[mmrid] = "load_break_switch";
                        string cemrid = m["ConductingEquipment_mRID"];
                        string zid = mmrid + "_switch";

                        // cemrid_busnames[cemrid] contains 2 buses
                        // adjacent to a switch for cemrid
                        string phase = m["phases"];
                        uint switch_node_count = 0;
                        for (auto it=cemrid_busnames[cemrid].begin();
                                it!=cemrid_busnames[cemrid].end(); ++it) {
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
                        mmrid_pos_type[mmrid] = "other";
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
                        // after processing add these to Zary
                        if (std::find(Zary.zids.begin(),Zary.zids.end(),pinj_zid) == Zary.zids.end()) {
                            // add the real power injection measurement
                            Zary.zids.push_back( pinj_zid );
                            Zary.zidxs[pinj_zid] = Zary.zqty++;
                            Zary.ztypes[pinj_zid] = "Pi";
                            Zary.znode1s[pinj_zid] = meas_node;
                            Zary.znode2s[pinj_zid] = meas_node;

                            // assumes pinj and qinj are only added together
                            // allowing a second find() call to be eliminated
                            // add the reactive power injection measurement
                            Zary.zids.push_back( qinj_zid );
                            Zary.zidxs[qinj_zid] = Zary.zqty++;
                            Zary.ztypes[qinj_zid] = "Qi";
                            Zary.znode1s[qinj_zid] = meas_node;
                            Zary.znode2s[qinj_zid] = meas_node;
                        }

                        // use nominal load for node from SPARQL query for zvals

                        Zary.zvals[pinj_zid] -= node_nominal_Pinj[meas_node]/(2.0*sbase);
                        double zsig_Pinj = node_nominal_Pinj[meas_node]*0.01/sbase;
                        Zary.zsigs[pinj_zid] = sqrt(Zary.zsigs[pinj_zid]*Zary.zsigs[pinj_zid] + zsig_Pinj*zsig_Pinj);    // 1 sigma = 1% of nominal
                        Zary.znomvals[pinj_zid] += Zary.zvals[pinj_zid];

                        Zary.zvals[qinj_zid] -= node_nominal_Qinj[meas_node]/(2.0*sbase);
                        double zsig_Qinj = node_nominal_Qinj[meas_node]*0.01/sbase;
                        Zary.zsigs[qinj_zid] = sqrt(Zary.zsigs[qinj_zid]*Zary.zsigs[qinj_zid] + zsig_Qinj*zsig_Qinj);    // 1 sigma = 1% of nominal
                        Zary.znomvals[qinj_zid] += Zary.zvals[qinj_zid];

#ifdef WRITE_FILES
                        ofs << Zary.ztypes[pinj_zid] << "," << pinj_zid << "," << Zary.znode1s[pinj_zid] << "," << Zary.znode2s[pinj_zid] << "," << Zary.zvals[pinj_zid] << "," << Zary.zsigs[pinj_zid] << ",0," << Zary.znomvals[pinj_zid] << "\n";
                        ofs << Zary.ztypes[qinj_zid] << "," << qinj_zid << "," << Zary.znode1s[qinj_zid] << "," << Zary.znode2s[qinj_zid] << "," << Zary.zvals[qinj_zid] << "," << Zary.zsigs[qinj_zid] << ",0," << Zary.znomvals[qinj_zid] << "\n";
#endif
#ifdef COMPARE_INJ_MEAS
                        Zary.injnodes.push_back( meas_node );
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
#ifdef WRITE_FILES
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
#endif
