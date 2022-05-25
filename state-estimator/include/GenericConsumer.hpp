#ifndef GENERICCONSUMER_HPP
#define GENERICCONSUMER_HPP
#ifdef GRIDAPPSD_INTERFACE

// This class is a generic listener
class GenericConsumer: public SEConsumer {
    public:
    GenericConsumer(const string& brokerURI, 
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
};

#endif
#endif
