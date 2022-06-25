#ifndef PLATFORMINTERFACEGRIDAPPSD_HPP
#define PLATFORMINTERFACEGRIDAPPSD_HPP

// if defined, produces an estimate for every simulation timestamp rather than
// catching up by doing measurement average to drain the queue which leads to
// state estimator falling way behind with complex models
//#define DONT_DRAIN_WORKQUEUE

#ifdef DEBUG_PRIMARY
// temporary flag to hold up initialization until the platform has finished
// its own initialization for the simulation based on sending a STARTED message
bool blockedFlag = true;
#endif

class PlatformInterface : public PlatformInterfaceBase {
public:
    PlatformInterface(int argc, char** argv, const double& sbase) : PlatformInterfaceBase(argc, argv, sbase) {
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
    }


    void setupMeasurements() {
        // --------------------------------------------------------------------
        // LISTEN FOR SIMULATION LOG MESSAGES
        // --------------------------------------------------------------------
        // simulation status (running, complete) comes from log messages
        string simLogTopic = "goss.gridappsd.simulation.log."+gad_ref->simid;

        SELoopConsumer* simLogConsumer = new SELoopConsumer(&workQueue,
            gad_ref->brokerURI, gad_ref->username, gad_ref->password,
            simLogTopic, "topic");
        Thread* simLogConsumerThread = new Thread(simLogConsumer);
        simLogConsumerThread->start();    // execute simLogConsumer->run
        simLogConsumer->waitUntilReady(); // wait for the startup latch release
#ifdef DEBUG_PRIMARY
        *selog << "Listening for simulation log messages on "+simLogTopic+'\n'
        << std::flush;
#endif

        // --------------------------------------------------------------------
        // LISTEN FOR SIMULATION OUTPUT MESSAGES
        // --------------------------------------------------------------------
        // measurements come from either simulation output or sensor-simulator
        string simOutTopic = gad_ref->useSensorsForEstimatesFlag?
            "goss.gridappsd.simulation.gridappsd-sensor-simulator."+
            gad_ref->simid+".output":
            "goss.gridappsd.simulation.output."+gad_ref->simid;

        SELoopConsumer* simOutConsumer = new SELoopConsumer(&workQueue,
            gad_ref->brokerURI, gad_ref->username, gad_ref->password,
            simOutTopic, "topic");
        Thread* simOutConsumerThread = new Thread(simOutConsumer);
        simOutConsumerThread->start();    // execute simOutConsumer->run
        simOutConsumer->waitUntilReady(); // wait for the startup latch release
#ifdef DEBUG_PRIMARY
        if (gad_ref->useSensorsForEstimatesFlag)
            *selog << "Listening for sensor-simulator output on "+simOutTopic+
            '\n' << std::flush;
        else
            *selog << "Listening for simulation output on "+simOutTopic+
            '\n' << std::flush;
#endif
    }


    void fillTopo() {
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


    void fillVnoms() {
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
    }


    void fillSensors() {
        SSMAP reg_cemrid_primbus;
        SSMAP reg_cemrid_regbus;
        state_estimator_util::build_A_matrix(*gad_ref, Amat, node_idxs,
            reg_cemrid_primbus, reg_cemrid_regbus,
            regid_primnode, regid_regnode);

        // map conducting equipment to bus names
        SSLISTMAP cemrid_busnames;
        state_estimator_util::build_cemrid_busnames(*gad_ref,
            cemrid_busnames);

        // Adds nominal load injections
        SDMAP node_nominal_Pinj;
        SDMAP node_nominal_Qinj;
        state_estimator_util::get_nominal_energy_consumer_injections(*gad_ref,
            node_vnoms, node_nominal_Pinj, node_nominal_Qinj);

        // Set up the sensors consumer
        string sensTopic = "goss.gridappsd.se.response."+gad_ref->simid+
            ".cimdict";
        SensorDefConsumer sensConsumer(gad_ref->brokerURI,
            gad_ref->username, gad_ref->password,
            cemrid_busnames, reg_cemrid_primbus, reg_cemrid_regbus,
            node_nominal_Pinj, node_nominal_Qinj,
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
        sensConsumer.fillSens(Zary, mmrid_pos_type,
            switch_node1s, switch_node2s);
        sensConsumer.close();

        // Add Pseudo-Measurements
        state_estimator_util::insert_pseudo_measurements(*gad_ref, Zary,
            node_names, node_vnoms, *sbase_ref);
#ifdef DEBUG_PRIMARY
        //*selog << "\nzsigs/zvals after adding pseudo-measurements:\n" << std::flush;
        //for ( auto& zid : Zary.zids ) {
        //    *selog << "\tzid: " << zid << ", ztype: " << Zary.ztypes[zid] << ", zsig: " << Zary.zsigs[zid] << ", zvals: " << Zary.zvals[zid] << "\n" << std::flush;
        //}
#endif
    }


    bool fillMeasurement() {
        bool ret = true;

        json jmessage = workQueue.pop();

        meas_timestamp = 0;
        meas_mrids.clear();
        meas_magnitudes.clear();
        meas_angles.clear();
        meas_values.clear();

        if (jmessage.find("message") != jmessage.end()) {
            meas_timestamp = jmessage["message"]["timestamp"];

            string mmrid;
            for ( auto& m : jmessage["message"]["measurements"] ) {
                mmrid = m["measurement_mrid"];
                meas_mrids.push_back(mmrid);
                if (m.find("magnitude") != m.end())
                    meas_magnitudes[mmrid] = m["magnitude"];
                if (m.find("angle") != m.end())
                    meas_angles[mmrid] = m["angle"];
                if (m.find("value") != m.end())
                    meas_values[mmrid] = m["value"];
            }
        } else if (jmessage.find("processStatus") != jmessage.end()) {
            // only COMPLETE/CLOSED log messages are put on the queue
            // so we know right away to return false
            ret = false;
        }

        return ret;
    }


    bool nextMeasurementWaiting() {
#ifndef DONT_DRAIN_WORKQUEUE
        return !workQueue.empty();
#else
        return false;
#endif
    }


    void setupPublishing() {
        // get node bus mRIDs and phases needed to publish results
        state_estimator_util::get_nodes(*gad_ref, node_bmrids, node_phs);

        string publisherTopic = "goss.gridappsd.simulation.state-estimator."+
            gad_ref->simid+".output";
        publisher_ref = new SEProducer(gad_ref->brokerURI, gad_ref->username,
            gad_ref->password, publisherTopic, "topic");
    }


    void publishEstimate(const uint& timestamp,
        SDMAP& est_v, SDMAP& est_angle, SDMAP& est_vvar, SDMAP& est_anglevar,
        SDMAP&,  SDMAP&) {

        json jmessage;
        jmessage["simulation_id"] = gad_ref->simid;
        jmessage["message"] = json::object();
        jmessage["message"]["timestamp"] = timestamp;
        jmessage["message"]["Estimate"] = json::object();
        jmessage["message"]["Estimate"]["timeStamp"] = timestamp;
        jmessage["message"]["Estimate"]["SvEstVoltages"] = json::array();

        for ( auto& node_name : node_names ) {
            // build a json object for each node
            json node_state;
            node_state["ConnectivityNode"] = node_bmrids[node_name];
            node_state["phase"] = node_phs[node_name];

            // add the state values
            node_state["v"] = est_v[node_name];
            node_state["angle"] = est_angle[node_name];

            // Add v and angle variance values
            node_state["vVariance"] = est_vvar[node_name];
            node_state["angleVariance"] = est_anglevar[node_name];

            // append this state to the measurement array
            jmessage["message"]["Estimate"]["SvEstVoltages"].
                push_back(node_state);
        }

        // Publish the message
        publisher_ref->send(jmessage.dump());
    }


    state_estimator_gridappsd::gridappsd_session* getGad() {
        return gad_ref;
    }


    string getOutputDir() {
        return "sim_" + gad_ref->simid;
    }

private:
    state_estimator_gridappsd::gridappsd_session* gad_ref;
    SEProducer* requester_ref;
    SEProducer* publisher_ref;
    SharedQueue<json> workQueue;
};

#endif
