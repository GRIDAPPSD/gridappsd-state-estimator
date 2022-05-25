#ifndef SE_GAD_HPP
#define SE_GAD_HPP

#include "json.hpp"
using nlohmann::json;

namespace state_estimator_gridappsd {
    class state_estimator_session {
        public:
        string simid;
        string simreq;
        string addr;
        string port;
        string username;
        string password;
        
        public:
        state_estimator_session(void) {}

        public:
        int init(int argc, char** argv) {
            // Required first input simulation ID
            if ( argc > 1 ) {
                simid = argv[1];
            } else { usage(argv); return 0; }

            // Required second input: simulation request from viz
            if ( argc > 2 ) {
                simreq = argv[2];
            } else { usage(argv); return 0; }

#ifdef SBASE_TESTING
            addr = "127.0.0.1";
            port = "61616";
            username = "system";
            password = "manager";
#else
            // Host address
            if ( argc > 3 ) addr = argv[3];
            else addr = "127.0.0.1";

            // Host port
            if ( argc > 4 ) port = argv[4];
            else port = "61616";

            // Username
            if ( argc > 5 ) username = argv[5];
            else username = "system";

            // Password
            if ( argc > 6 ) password = argv[6];
            else password = "manager";

            // Check for extraneous inputs
            if ( argc > 7 ) {
                std::cerr << "Unrecognized input: " << argv[7] << '\n' << std::flush;
                usage(argv); return 0;
            }
#endif

            // Success
            return 1;
        }

        private:
        void usage(char** argv) {
            std::cerr << "Usage: " << *argv
                << " simid simreq ipaddr port username password\n" << std::flush;
        }

        public:
        ~state_estimator_session(void) {}
    };

    class gridappsd_session {
        public:
        string simid;
        string brokerURI;
        string username;
        string password;
        string modelID;
        bool stateEstimatorFromPlatformFlag;
        bool useSensorsForEstimatesFlag;
        
        public:
        gridappsd_session(void) {}

        public:
        gridappsd_session(const state_estimator_session& se) {
            simid = se.simid;
            brokerURI = "tcp://" + se.addr + ':' + se.port;
            username = se.username;
            password = se.password;

            json jsimreq = json::parse(se.simreq);
            modelID = jsimreq["power_system_config"]["Line_name"];

            // as long as the key used below isn't set when running from the
            // command line, this check will be sufficient for distinguishing
            // platfrom from command-line invocations
            stateEstimatorFromPlatformFlag = se.simreq.find("simulation_request_type")!=string::npos;

            // process simreq service_configs entries to set whether the
            // sensor-simulator has been configured and whether the
            // state-estimator should use the measurements from
            // sensor-simulator
            bool sensorSimulatorRunningFlag = false;
            useSensorsForEstimatesFlag = false;
            for (auto const& jsc : jsimreq["service_configs"]) {
                if (jsc["id"] == "gridappsd-sensor-simulator")
                    // if sensor-simulator is in the list of configured
                    // services, it is running
                    sensorSimulatorRunningFlag = true;
                else if (jsc["id"] == "state-estimator")
                    useSensorsForEstimatesFlag = jsc["user_options"]["use-sensors-for-estimates"];
            }
            // override the use of sensors for estimates if the service
            // isn't running
            if (!sensorSimulatorRunningFlag)
                useSensorsForEstimatesFlag = false;
        }

        public:
        ~gridappsd_session(void) {}
    };
}

#endif
