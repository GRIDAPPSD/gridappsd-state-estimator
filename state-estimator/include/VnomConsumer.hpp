#ifndef VNOMCONSUMER_HPP
#define VNOMCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "SEConsumer.hpp"

// for side-loading test cases
#include <fstream>

// standard data types
#include <string>
#include <complex>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// Store node names in a linked list and hash node name to their position
// Iterate over the linked list to access all nodes or states
// Note that positions are one-indexed
// Store sensor names in a linked list and hash node names to various params
#define SLIST std::list<std::string>
#define SIMAP std::unordered_map<std::string,unsigned int>
#define SCMAP std::unordered_map<std::string,std::complex<double>>

// Hash address (i,j) to the index of a sparse matrix vector
#define ICMAP std::unordered_map<unsigned int,std::complex<double>>
#define IMMAP std::unordered_map<unsigned int,ICMAP>

using namespace std;

// This class listens for the Y-bus message and constructs the topology
class VnomConsumer: public SEConsumer {

    // this needs to hold a map from nodes to complex nominal voltages
    private:
    SCMAP node_vnoms;

    public:
    VnomConsumer(const string& brokerURI, 
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

    public:
    void fillVnom(SCMAP& node_vnoms) {
        node_vnoms = this->node_vnoms;
    }
    
    public:
//    virtual void init() {
//        // WHEN THE API CALL EXISTS, WE CAN DELETE init()
//        //  - normally, a message populates this->text and calls process()
//        //  - here, we'll populate this->text from a file and call process()
//        json jtext;
//        jtext["data"] = json::object();
//        jtext["data"]["vnomFile"] = json::array();
//
//        // for each line
//        ifstream ifh("base_nominal_voltages.csv");
//
//        string line;
//        while ( getline(ifh,line) )
//            jtext["data"]["vnomFile"].push_back(line);
//
//        text = jtext.dump();
//
//        *selog << text + "\n" << std::flush;
//
//        process();
//    }


    public:
    virtual void process() {
        
        // --------------------------------------------------------------------
        // PARSE THE MESSAGE AND PROCESS THE TOPOLOGY
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        *selog << "Received vnom message of " << text.length() << " bytes\n\n" << std::flush;
#endif

#ifdef FILE_INTERFACE_WRITE
        std::ofstream ofs("test_files/vnom.csv", ofstream::out);
        ofs << "Nodename,Mag,Arg\n";
#endif

        json jtext = json::parse(text);

#ifdef DEBUG_PRIMARY
        *selog << "Parsing vnom -- " << std::flush;
#endif

        bool firstline = true;
        for ( auto& jline : jtext["data"]["vnom"] ) {
            if (firstline) firstline = false;
            else {
                string s = jline;
                // strip out white space
                s.erase( remove( s.begin(), s.end(), ' ' ), s.end() );

                // split the line {Bus, BasekV,
                //    Node1, Mag1, Arg1, pu1,
                //    Node2, Mag2, Arg2, pu2,
                //    Node3, Mag3, Arg3, pu3 }
                size_t pos = s.find(",");
                string bus = s.substr(0,pos);
                bus.erase( remove ( bus.begin(), bus.end(), '"' ), bus.end() );
                    s.erase(0,pos+1); pos = s.find(",");
                string basekv_str = s.substr(0,pos);
                double basekv = stod(basekv_str);
                    s.erase(0,pos+1); pos = s.find(",");

                string node1_str = s.substr(0,pos);
                int node1 = stoi(node1_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string mag1_str = s.substr(0,pos);
                double mag1 = stod(mag1_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string arg1_str = s.substr(0,pos);
                double arg1 = stod(arg1_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string vpu1_str = s.substr(0,pos);
                double vpu1 = stod(vpu1_str);
                    s.erase(0,pos+1); pos = s.find(",");

                string node2_str = s.substr(0,pos);
                int node2 = stoi(node2_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string mag2_str = s.substr(0,pos);
                double mag2 = stod(mag2_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string arg2_str = s.substr(0,pos);
                double arg2 = stod(arg2_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string vpu2_str = s.substr(0,pos);
                double vpu2 = stod(vpu2_str);
                    s.erase(0,pos+1); pos = s.find(",");

                string node3_str = s.substr(0,pos);
                int node3 = stoi(node3_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string mag3_str = s.substr(0,pos);
                double mag3 = stod(mag3_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string arg3_str = s.substr(0,pos);
                double arg3 = stod(arg3_str);
                    s.erase(0,pos+1); pos = s.find(",");
                string vpu3_str = s.substr(0,pos);
                double vpu3 = stod(vpu3_str);
                    s.erase(0,pos+1); pos = s.find(",");

                // Each of the the three nodes is a potential entry
                string node;
                double vre, vim;
                complex<double> vnom;

                // check node1
                if ( node1 ) {
                    node = bus + '.' + node1_str;
                    vre = mag1 * cos( arg1 * PI/180.0 );
                    vim = mag1 * sin( arg1 * PI/180.0 );
                    vnom = complex<double>(vre,vim);
                    node_vnoms[node] = vnom;
#ifdef FILE_INTERFACE_WRITE
                    ofs << node << "," << mag1_str << "," << arg1_str << "\n";
#endif
                }
                
                // check node 2
                if ( node2 ) {
                    node = bus + '.' + node2_str;
                    vre = mag2 * cos( arg2 * PI/180.0 );
                    vim = mag2 * sin( arg2 * PI/180.0 );
                    vnom = complex<double>(vre,vim);
                    node_vnoms[node] = vnom;
#ifdef FILE_INTERFACE_WRITE
                    ofs << node << "," << mag2_str << "," << arg2_str << "\n";
#endif
                }
                
                // check node 3
                if ( node3 ) {
                    node = bus + '.' + node3_str;
                    vre = mag3 * cos( arg3 * PI/180.0 );
                    vim = mag3 * sin( arg3 * PI/180.0 );
                    vnom = complex<double>(vre,vim);
                    node_vnoms[node] = vnom;
#ifdef FILE_INTERFACE_WRITE
                    ofs << node << "," << mag3_str << "," << arg3_str << "\n";
#endif
                }

            }
        }
#ifdef FILE_INTERFACE_WRITE
        ofs.close();
#endif

#ifdef DEBUG_PRIMARY
        *selog << "complete.\n\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // TOPOLOGY PROCESSING COMPLETE
        // --------------------------------------------------------------------
        // release latch
        doneLatch.countDown();
    }
};

#endif
