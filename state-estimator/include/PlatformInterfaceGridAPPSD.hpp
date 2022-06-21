#ifndef PLATFORMINTERFACEGRIDAPPSD_HPP
#define PLATFORMINTERFACEGRIDAPPSD_HPP

#ifdef DEBUG_PRIMARY
// temporary flag to hold up initialization until the platform has finished
// its own initialization for the simulation based on sending a STARTED message
bool blockedFlag = true;
#endif

class PlatformInterface : public PlatformInterfaceCommon {
public:
    PlatformInterface(int argc, char** argv, const double& sbase) {
        // --------------------------------------------------------------------
        // INITIALIZE THE STATE ESTIMATOR SESSION WITH RUNTIME ARGS
        // --------------------------------------------------------------------
        state_estimator_gridappsd::state_estimator_session se;
        if ( !se.init(argc, argv) ) return;

        // --------------------------------------------------------------------
        // INITIALIZE THE GRIDAPPS-D SESSION
        // --------------------------------------------------------------------
        gad_ref = new state_estimator_gridappsd::gridappsd_session(se);

#ifdef DEBUG_PRIMARY
        // determine whether to write to a log file or stdout based on whether
        // this is a platform vs. command line invocation
        static std::ofstream logfile;
        if (gad_ref->stateEstimatorFromPlatformFlag) {
            logfile.open("/tmp/state-estimator.log");
            selog = &logfile;
        }
#endif

        // --------------------------------------------------------------------
        // START THE AMQ INTERFACE
        // --------------------------------------------------------------------
        activemq::library::ActiveMQCPP::initializeLibrary();

        // Set up the producer shared by ybus, vnom, and sensor requests
        string requestTopic = "goss.gridappsd.process.request.config";
        requester_ref = new SEProducer(gad_ref->brokerURI, gad_ref->username,
            gad_ref->password, requestTopic, "queue");

        sbase_ref = &sbase;
    }


    void fillTopologyMinimal(IMMAP& Yphys, SLIST& node_names,
        SSMAP& node_bmrids, SSMAP& node_phs) {
#ifdef DEBUG_PRIMARY
        extern bool blockedFlag;
        // only block initialization for command line invocations
        //if (false) {
        if (!gad_ref->stateEstimatorFromPlatformFlag) {
            *selog << "\nWaiting for simulation to start before continuing with initialization\n" << std::flush;
            while (blockedFlag) sleep(1);
            *selog << "\nSimulation started--continuing with initialization\n" << std::flush;
        } else {
            *selog << "\nNOT waiting before continuing with initialization\n" << std::flush;
        }
#endif

        // --------------------------------------------------------------------
        // MAKE SOME SPARQL QUERIES
        // --------------------------------------------------------------------
        // get node bus mRIDs and phases needed to publish results
        state_estimator_util::get_nodes(*gad_ref, node_bmrids, node_phs);

        // Set up the ybus consumer
        string ybusTopic = "goss.gridappsd.se.response."+gad_ref->simid+".ybus";
        TopoProcConsumer ybusConsumer(gad_ref->brokerURI, gad_ref->username,
            gad_ref->password, ybusTopic, "queue");
        Thread ybusConsumerThread(&ybusConsumer);
        ybusConsumerThread.start();       // execute ybusConsumer.run()
        ybusConsumer.waitUntilReady();    // wait for latch release

        // Request ybus with previously created producer
        string ybusRequestText =
            "{\"configurationType\":\"YBus Export\",\"parameters\":{\"simulation_id\":\""
            + gad_ref->simid + "\"}}";
        requester_ref->send(ybusRequestText, ybusTopic);

        // Wait for topology processor and retrieve topology (ybus, node info)
        ybusConsumerThread.join();
        ybusConsumer.fillTopo(node_names, Yphys);
        ybusConsumer.close();
    }


    void fillVnom(SCMAP& node_vnoms) {
        // Set up the vnom consumer
        string vnomTopic = "goss.gridappsd.se.response."+gad_ref->simid+".vnom";
        VnomConsumer vnomConsumer(gad_ref->brokerURI, gad_ref->username,
            gad_ref->password, vnomTopic, "queue");
        Thread vnomConsumerThread(&vnomConsumer);
        vnomConsumerThread.start();       // execute vnomConsumer.run()
        vnomConsumer.waitUntilReady();    // wait for latch release

        // Request vnom with previously created producer
        string vnomRequestText =
            "{\"configurationType\":\"Vnom Export\",\"parameters\":{\"simulation_id\":\""
            + gad_ref->simid + "\"}}";
        requester_ref->send(vnomRequestText, vnomTopic);

        // Wait for the vnom processor and retrive vnom
        vnomConsumerThread.join();
        vnomConsumer.fillVnom(node_vnoms);
        vnomConsumer.close();

        node_vnoms_ref = &node_vnoms;
    }


    void fillMeasurements(SensorArray& zary, IMDMAP& Amat,
        SSMAP& regid_primnode_map, SSMAP& regid_regnode_map,
        SSMAP& mmrid_pos_type_map, SSMAP& switch_node1s, SSMAP& switch_node2s) {

        SSMAP reg_cemrid_primbus_map;
        SSMAP reg_cemrid_regbus_map;
        state_estimator_util::build_A_matrix(*gad_ref, Amat, *node_idxs_ref,
            reg_cemrid_primbus_map, reg_cemrid_regbus_map,
            regid_primnode_map, regid_regnode_map);

        // map conducting equipment to bus names
        SSLISTMAP cemrid_busnames_map;
        state_estimator_util::build_cemrid_busnames_map(*gad_ref,
            cemrid_busnames_map);

        // Adds nominal load injections
        SDMAP node_nominal_Pinj_map;
        SDMAP node_nominal_Qinj_map;
        state_estimator_util::get_nominal_energy_consumer_injections(*gad_ref,
            *node_vnoms_ref, node_nominal_Pinj_map, node_nominal_Qinj_map);

        // Set up the sensors consumer
        string sensTopic = "goss.gridappsd.se.response."+gad_ref->simid+
            ".cimdict";
        SensorDefConsumer sensConsumer(gad_ref->brokerURI,
            gad_ref->username, gad_ref->password,
            cemrid_busnames_map, reg_cemrid_primbus_map, reg_cemrid_regbus_map,
            node_nominal_Pinj_map, node_nominal_Qinj_map,
            *sbase_ref, sensTopic, "queue");
        Thread sensConsumerThread(&sensConsumer);
        sensConsumerThread.start();       // execute sensConsumer.run()
        sensConsumer.waitUntilReady();    // wait for latch release

        // Request sensor data with previously created producer
        string sensRequestTopic = "goss.gridappsd.process.request.config";
        string sensRequestText = "{\"configurationType\":\"CIM Dictionary\",\"parameters\":{\"simulation_id\":\""
            + gad_ref->simid + "\"}}";
        requester_ref->send(sensRequestText, sensTopic);
        requester_ref->close(); // this is the last request so close it off

        // Wait for sensor initializer and retrieve sensors
        sensConsumerThread.join();
        sensConsumer.fillSens(zary, mmrid_pos_type_map,
            switch_node1s, switch_node2s);
        sensConsumer.close();

        // Add Pseudo-Measurements
        state_estimator_util::insert_pseudo_measurements(*gad_ref, zary,
            *node_names_ref, *node_vnoms_ref, *sbase_ref);
#ifdef DEBUG_PRIMARY
        //*selog << "\nzsigs/zvals after adding pseudo-measurements:\n" << std::flush;
        //for ( auto& zid : zary.zids ) {
        //    *selog << "\tzid: " << zid << ", ztype: " << zary.ztypes[zid] << ", zsig: " << zary.zsigs[zid] << ", zvals: " << zary.zvals[zid] << "\n" << std::flush;
        //}
#endif
    }


    state_estimator_gridappsd::gridappsd_session* getGad() {
        return gad_ref;
    }


private:
    state_estimator_gridappsd::gridappsd_session* gad_ref;
    SEProducer* requester_ref;
    SCMAP* node_vnoms_ref;
    const double* sbase_ref;

};

#endif
