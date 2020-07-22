#ifndef GAD_REQUESTS_HPP
#define GAD_REQUESTS_HPP

#include "state_estimator_gridappsd.hpp"
using session = state_estimator_gridappsd::gridappsd_session;

#include "SEProducer.hpp"
#include "GenericConsumer.hpp"

#include "json.hpp"
using nlohmann::json;

#include<string>
using std::string;

namespace gridappsd_requests {
	json sparql_query(const session& s, const string& t, const string& q) {
		// set up the consumer
		string responseTopic = "goss.gridappsd.se.response."+s.simid+"."+t;
		GenericConsumer activeConsumer(s.brokerURI,s.username,s.password,responseTopic,"queue");
		Thread activeThread(&activeConsumer);
		activeThread.start();
		activeConsumer.waitUntilReady();

		// set up the producer
		string requestTopic = "goss.gridappsd.process.request.data.powergridmodel";
		SEProducer activeProducer(s.brokerURI,s.username,s.password,requestTopic,"queue");
		string requestText = "{\"requestType\":\"QUERY\","
			"\"resultFormat\":\"JSON\","
			"\"queryString\":\"" + q + "\"}";
		activeProducer.send(requestText,responseTopic);

		// retrive the result
		activeThread.join();
		string resultText;
		activeConsumer.get(resultText);

		// return json
		return json::parse(resultText);
	}

	// GDB 7/22/20 generates a compiler warning with no return statement
	// on my Ubuntu 20 VM, probably a newer g++ compiler
	//json config_request(const session&s, const string&t) {}
}


#endif
