#ifndef SECONSUMER_HPP
#define SECONSUMER_HPP

#include <activemq/library/ActiveMQCPP.h>
#include <decaf/lang/Thread.h>
#include <decaf/lang/Runnable.h>
#include <decaf/util/concurrent/CountDownLatch.h>
#include <decaf/lang/Integer.h>
#include <decaf/lang/Long.h>
#include <decaf/lang/System.h>
#include <activemq/core/ActiveMQConnectionFactory.h>
#include <activemq/util/Config.h>
#include <cms/Connection.h>
#include <cms/Session.h>
#include <cms/TextMessage.h>
#include <cms/BytesMessage.h>
#include <cms/MapMessage.h>
#include <cms/ExceptionListener.h>
#include <cms/MessageListener.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <string>
#include <regex>

using namespace activemq::core;
using namespace decaf::util::concurrent;
using namespace decaf::util;
using namespace decaf::lang;
using namespace cms;
using namespace std;


// This class is the ancestor of various state estimator listeners
class SEConsumer : public ExceptionListener,
				   public MessageListener,
				   public Runnable {
	protected:
	CountDownLatch latch;
	CountDownLatch doneLatch;
	Connection* connection;
	Session* session;
	Destination* destination;
	MessageConsumer* consumer;
	std::string brokerURI;
	std::string username;
	std::string password;
	std::string target;
	std::string mode;
	
	protected:
	SEConsumer(const SEConsumer&);
	SEConsumer& operator=(const SEConsumer&);
	
	public:
	SEConsumer() :
		latch(1),
		doneLatch(1),
		connection(NULL),
		session(NULL),
		destination(NULL),
		consumer(NULL),
		brokerURI((string)""),
		username((string)""),
		password((string)""),
		target((string)""),
		mode((string)"") {}

	public:
	virtual ~SEConsumer() {
		cleanup();
	}
	
	public:
	void close() {
		this->cleanup();
	}
	
	public:
	void waitUntilReady() {
		latch.await();
	}
	
	protected:
	virtual void run() {
		// Subscribe to the topic
		try {
			// Create a ConnectionFactory
			auto_ptr<ConnectionFactory> connectionFactory(
				ConnectionFactory::createCMSConnectionFactory(brokerURI));
			// Create a Connection
			this->connection = connectionFactory->createConnection(username,password);
			this->connection->start();
			this->connection->setExceptionListener(this);
			// Create a Session
			this->session = connection->createSession(Session::AUTO_ACKNOWLEDGE);

			// Check the mode and create the destination Topic or Queue
			if ( mode == "topic" )
				destination = session->createTopic(target);
			else if ( mode == "queue" )
				destination = session->createQueue(target);
			else
				throw "unrecognized mode \"" + mode + "\"";

			// Create a MessageConsumer from the Session to the Topic or Queue
			this->consumer = session->createConsumer(destination);
			this->consumer->setMessageListener(this);
			std::cout.flush();
			std::cerr.flush();

			// Allow implementation-specific actions:
			this->init();

			// Indicate we are ready for messages.
			this->latch.countDown();
			// Wait for the response
			this->doneLatch.await();
		} catch (CMSException& e) {
			// Indicate we are ready for messages.
			this->latch.countDown();
			e.printStackTrace();
		}
	}
	
	protected:
	virtual void init() {
		// implementation-specific actions - default is nothing
	}

	protected:
	virtual void onMessage(const Message* message) {
		// What to do when a message is recieved
		try {
			const BytesMessage* bytesMessage = 
					dynamic_cast<const BytesMessage*> (message);
			string text = "";
			if (bytesMessage != NULL) {
				for ( int idx = 0 ; idx < bytesMessage->getBodyLength() ; idx++ )
					text.push_back(bytesMessage->readChar());
			} else {
				text = "NOT A BYTESMESSAGE!";
			}
			
			// Print for debugging
//			cout << "Recieved message:\n\t" + text + "\n\n";
//			cout << "Removing escape characters:\n";
//			text = regex_replace(text,(regex)R"(\\)","");
//			cout << "\t" + text + "\n\n";

			// implementation-specific actions:
			process(text);

		} catch (CMSException& e) {
			e.printStackTrace();
		}
	}

	public:
	virtual void process(const string& text) {
		cout << "Recieved message:\n\t" << text << "\n\n";

		// Default action: release the latch
		doneLatch.countDown();
	}
		
	public:
	// If something bad happens you see it here as this class is also been
	// registered as an ExceptionListener with the connection.
	virtual void onException(const CMSException& ex AMQCPP_UNUSED) {
		printf("CMS Exception occurred.  Shutting down client.\n");
		ex.printStackTrace();
		exit(1);
	}
	
	protected:
	void cleanup() {
		if (connection != NULL) {
			try {
				connection->close();
			} catch (cms::CMSException& ex) {
				ex.printStackTrace();
			}
		}
		// Destroy resources.
		try {
			delete destination;
			destination = NULL;
			delete consumer;
			consumer = NULL;
			delete session;
			session = NULL;
			delete connection;
			connection = NULL;
		} catch (CMSException& e) {
			e.printStackTrace();
		}
	}
};

#endif
