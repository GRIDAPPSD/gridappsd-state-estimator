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
            *selog << "." << std::flush;
//            *selog << "\nSELoopConsumer received measurement message of " << text.length() 
//                << " bytes\n" << std::flush;
            //*selog << "MESSAGE START\n" << std::flush;
            //for (uint ibuff=0; ibuff<text.length(); ibuff+=4095) {
            //    *selog << text.substr(ibuff,4095) << "\n" << std::flush;
            //}
            //*selog << "MESSAGE END\n" << std::flush;
#endif
            workQueue->push(jtext);

#ifdef DEBUG_PRIMARY
            // this is only needed for command line invocations, but
            // easier to just do it than check if it's needed
            extern bool blockedFlag;
            blockedFlag = false;
#endif
        } else if (jtext.find("processStatus") != jtext.end()) {
            string status = jtext["processStatus"];
            if (!status.compare("COMPLETE") || !status.compare("CLOSED")) {
#ifdef DEBUG_PRIMARY
                *selog << "\nSELoopConsumer received COMPLETE/CLOSED log message\n" << std::flush;
#endif
                workQueue->push(jtext);
            }
        }
    }
};

#endif
