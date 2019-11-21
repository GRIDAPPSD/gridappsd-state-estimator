#ifndef SELOOPCONSUMER_HPP
#define SELOOPCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "SEConsumer.hpp"

#include "SharedQueue.hpp"
// Global definition of workQueue to make sharing easy
SharedQueue<json> workQueue;

// This class listens for system state messages
class SELoopConsumer : public SEConsumer {
    // system state
    private:
    json jtext;     // object holding the input message

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

    // ------------------------------------------------------------------------
    // PROCESS ()
    // ------------------------------------------------------------------------
    public:
    virtual void process() {
        jtext = json::parse(text);
        uint timestamp = jtext["message"]["timestamp"];
#ifdef DEBUG_PRIMARY
        cout << "\nSELoopConsumer received mesasurement message of " << text.length() 
            << " bytes for timestamp: " << timestamp << "\n" << std::flush;
#endif

        workQueue.push(jtext["message"]);
    }
};

#endif
