#ifndef SELOOPCONSUMER_HPP
#define SELOOPCONSUMER_HPP

#include "SEConsumer.hpp"

// This class listens for system state messages
class SELoopConsumer : public SEConsumer {
	public:
	SELoopConsumer(const string& brokerURI, 
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

	private:
	virtual void init() {
		// do unique stuff
//		std::cout << "Unique SELoopConsumer run stuff...\n";
	}

	public:
	virtual void process(const string& text) {
		// stuff -- need to preserve the loop
		cout << "\nMessage recieved on measurement topic:\n\t"+text+"\n";

		// I'm not sure what will happen if SE falls behind
		//	-- skipping messages is bad
		//	-- increassing lag is bad



		// When we recieve a message to terminate:
		if ( text == "stop" ) {
			cout << "TIME TO STOP!\n";
			doneLatch.countDown();
		}
	}
};

#endif
