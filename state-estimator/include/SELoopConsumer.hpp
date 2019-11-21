#ifndef SELOOPCONSUMER_HPP
#define SELOOPCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "SEConsumer.hpp"

// This class listens for system state messages
class SELoopConsumer : public SEConsumer {
    // system state
    private:
    json jtext;     // object holding the input message
    SharedQueue<json>* workQueue;

    public:
    SELoopConsumer(SharedQueue<json>* workQueue,
            const string& brokerURI,
            const string& username,
            const string& password,
            const string& target,
            const string& mode) {
        this->workQueue = workQueue;
        this->brokerURI = brokerURI;
        this->username = username;
        this->password = password;
        this->target = target;
        this->mode = mode;
    }

    // ------------------------------------------------------------------------
    // PROCESS ()
    // ------------------------------------------------------------------------
    public:
    virtual void process() {
        jtext = json::parse(text);

#ifdef DEBUG_PRIMARY
        uint timestamp = jtext["message"]["timestamp"];
        cout << "\nSELoopConsumer received mesasurement message of " << text.length() 
            << " bytes for timestamp: " << timestamp << "\n" << std::flush;
#endif

        workQueue->push(jtext["message"]);
    }
};

#endif
