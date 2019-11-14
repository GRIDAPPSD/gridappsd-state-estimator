#ifndef SE_GAD_HPP
#define SE_GAD_HPP

#include "json.hpp"
using nlohmann::json;

#include<string>
using std::string;

#include<iostream>
using std::cerr;

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

			// Success
			return 1;
		}

		private:
		void usage(char** argv) {
			cerr << "Usage: " << *argv
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
		
		public:
		gridappsd_session(void) {}

		public:
		gridappsd_session(const state_estimator_session& se) {
			simid = se.simid;
			brokerURI = "tcp://" + se.addr + ':' + se.port;
			username = se.username;
			password = se.password;
			modelID = json::parse(se.simreq)["power_system_config"]["Line_name"];
		}

		public:
		~gridappsd_session(void) {}
	};
}

#endif
