#ifndef SEPRODUCER_HPP
#define SEPRODUCER_HPP

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

using namespace activemq::core;
using namespace decaf::util::concurrent;
using namespace decaf::util;
using namespace decaf::lang;
using namespace cms;
using namespace std;

// This class is used to publish messages to a specified topic or queue
class SEProducer {
	private:
	Connection* connection;
	Session* session;
	Destination* destination;
	MessageProducer* producer;
	std::string brokerURI;
	std::string username;
	std::string password;
	std::string target;
	std::string mode;
	int started;
	
	private:
	SEProducer(const SEProducer&);
	SEProducer& operator=(const SEProducer&);
	
	public:
	SEProducer(const std::string& brokerURI, 
					const std::string& username,
					const std::string& password,
				   const std::string& target,
				   const std::string& mode) :
		connection(NULL),
		session(NULL),
		destination(NULL),
		producer(NULL),
		brokerURI(brokerURI),
		username(username),
		password(password),
		target(target),
		mode(mode)
		{ started = 0; }
	
	public:
	virtual ~SEProducer(){
		cleanup();
	}
	
	public:
	void close() {
		this->cleanup();
	}

	virtual void init() {
		if ( !started ) {
			try {
				// Create a ConnectionFactory
				auto_ptr<ConnectionFactory> connectionFactory(
					ConnectionFactory::createCMSConnectionFactory(brokerURI));
				// Create a Connection
				connection = connectionFactory->createConnection(username,password);
				connection->start();
				// Create a Session
				session = connection->createSession(Session::AUTO_ACKNOWLEDGE);
				// Check the mode and create the destination Topic or Queue
				if ( mode == "topic" )
					destination = session->createTopic(target);
				else if ( mode == "queue" )
					destination = session->createQueue(target);
				else
					throw "unrecognized mode \"" + mode + "\"";
				// Create a MessageProducer from the Session to the Topic
				producer = session->createProducer(destination);
				producer->setDeliveryMode(DeliveryMode::NON_PERSISTENT);
			} catch (CMSException& e) {
				e.printStackTrace();
			}
			started = 1;
		}
	}
	
	virtual void send(const string& text) {
		try {
			// Start the interface if necessary
			if ( !started )	this->init();
			// Create the message
			auto_ptr<TextMessage> msg(session->createTextMessage(text));
#ifdef DEBUG_PRIMARY
			// Report
//			*selog << "Publishing to "+target+":\n\t"+text+"\n\n" << std::flush;
			*selog << "Publishing to "+target+"\n\n" << std::flush;
#endif
			// Send the message
			producer->send(msg.get());
		} catch (CMSException& e) {
			e.printStackTrace();
		}
	}
	
	virtual void send(const string& text,const string& replytopic) {
		try {
			// Start the interface if necessary
			if ( !started )	this->init();
			// Create the message
			auto_ptr<TextMessage> msg(session->createTextMessage(text));
			// Set the reply-to topic
			msg->setStringProperty("reply-to",replytopic);
#ifdef DEBUG_PRIMARY
            *selog << "Publishing to "+target+"\n\n" << std::flush;
#endif

			// Send the message
			producer->send(msg.get());
		} catch (CMSException& e) {
			e.printStackTrace();
		}
	}
		
	private:
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
			delete producer;
			producer = NULL;
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
