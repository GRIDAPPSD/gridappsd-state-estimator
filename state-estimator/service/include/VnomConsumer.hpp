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
//	virtual void init() {
//		// WHEN THE API CALL EXISTS, WE CAN DELETE init()
//		//  - normally, a message populates this->text and calls process()
//		//  - here, we'll populate this->text from a file and call process()
//		json jtext;
//		jtext["data"] = json::object();
//		jtext["data"]["vnomFile"] = json::array();
//
//		// for each line
//		ifstream ifh("base_nominal_voltages.csv");
//
//		string line;
//		while ( getline(ifh,line) )
//			jtext["data"]["vnomFile"].push_back(line);
//
//		text = jtext.dump();
//
//		cout << text + '\n';
//
//		process();
//	}


	public:
	virtual void process() {
		
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND PROCESS THE TOPOLOGY
		// --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
		cout << "\nReceived vnom message of " << text.length() << " bytes...\n\n";
#endif


		json jtext = json::parse(text);

#ifdef DEBUG_SECONDARY
		// This is actually a list of lines from a dss voltage export file
		cout << "Vnom Message:\n" + jtext.dump(2) + "\n\n";
#endif

		bool firstline = true;
		for ( auto& jline : jtext["data"]["vnom"] ) {
			if (firstline) firstline = false;
			else {
				string s = jline;
#ifdef DEBUG_SECONDARY
				cout << s + '\n';
#endif
				
				// strip out white space
				s.erase( remove( s.begin(), s.end(), ' ' ), s.end() );
#ifdef DEBUG_SECONDARY
				cout<< s + '\n';
#endif

				// split the line {Bus, BasekV,
				//    Node1, Mag1, Arg1, pu1,
				//    Node2, Mag2, Arg2, pu2,
				//    Node3, Mag3, Arg3, pu3 }
				size_t pos = s.find(",");
				string bus = s.substr(0,pos);
				bus.erase( remove ( bus.begin(), bus.end(), '"' ), bus.end() );
					s.erase(0,pos+1); pos = s.find(",");
				double basekv = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");

				int node1 = stoi( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double mag1 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double arg1 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double vpu1 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");

				int node2 = stoi( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double mag2 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double arg2 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double vpu2 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");

				int node3 = stoi( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double mag3 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double arg3 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");
				double vpu3 = stod( s.substr(0,pos) );
					s.erase(0,pos+1); pos = s.find(",");

#ifdef DEBUG_SECONDARY
				cout <<'\t'<< bus << '\t' << basekv << "\n\t"
					<< node1 << '\t' << mag1 << '\t' << arg1 << '\t' << vpu1 << "\n\t"
					<< node2 << '\t' << mag2 << '\t' << arg2 << '\t' << vpu2 << "\n\t"
					<< node3 << '\t' << mag3 << '\t' << arg3 << '\t' << vpu3 << "\n";
#endif
			
				// Each of the the three nodes is a potential entry
				string node;
				double vre, vim;
				complex<double> vnom;

				// check node1
				if ( node1 ) {
					node = bus + '.' + to_string(node1);
					vre = mag1 * cos( arg1 * PI/180 );
					vim = mag1 * sin( arg1 * PI/180);
					vnom = complex<double>(vre,vim);
#ifdef DEBUG_SECONDARY
					cout << vnom << endl;
#endif
					node_vnoms[node] = vnom;
				}
				
				// check node 2
				if ( node2 ) {
					node = bus + '.' + to_string(node2);
					vre = mag2 * cos( arg2 * PI/180 );
					vim = mag2 * sin( arg2 * PI/180 );
					vnom = complex<double>(vre,vim);
#ifdef DEBUG_SECONDARY
					cout << vnom << endl;
#endif
					node_vnoms[node] = vnom;
				}
				
				// check node 3
				if ( node3 ) {
					node = bus + '.' + to_string(node3);
					vre = mag3 * cos( arg3 * PI/180 );
					vim = mag3 * sin( arg3 * PI/180 );
					vnom = complex<double>(vre,vim);
#ifdef DEBUG_SECONDARY
					cout << vnom << endl;
#endif
					node_vnoms[node] = vnom;
				}

			}
		}


		// --------------------------------------------------------------------
		// TOPOLOGY PROCESSING COMPLETE
		// --------------------------------------------------------------------
		// release latch
		doneLatch.countDown();
	}
};

#endif
