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

        if (jtext.find("message") != jtext.end()) {
#ifdef DEBUG_PRIMARY
            cout << "\nSELoopConsumer received measurement message of " << text.length() 
                << " bytes\n" << std::flush;
            //cout << "MESSAGE START\n" << std::flush;
            //for (uint ibuff=0; ibuff<text.length(); ibuff+=4095) {
            //    cout << text.substr(ibuff,4095) << "\n" << std::flush;
            //}
            //cout << "MESSAGE END\n" << std::flush;
#endif
            workQueue->push(jtext);
        } else if (jtext.find("processStatus") != jtext.end()) {
            string status = jtext["processStatus"];
            if (!status.compare("COMPLETE") || !status.compare("CLOSED")) {
#ifdef DEBUG_PRIMARY
                cout << "\nSELoopConsumer received COMPLETE/CLOSED log message\n" << std::flush;
#endif
                workQueue->push(jtext);
            }
        }
    }
};

#endif
