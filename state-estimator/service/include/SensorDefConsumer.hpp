#ifndef SENSORDEFCONSUMER_HPP
#define SENSORDEFCONSUMER_HPP

#include "SEConsumer.hpp"

// standard data types
#include <string>
//#include <complex>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// Store node names in a linked list and hash node name to their position
// Iterate over the linked list to access all nodes or states
// Note that positions are one-indexed
// Store sensor names in a linked list and hash node names to various params
#define SLIST std::list<std::string>
//#define SIMAP std::unordered_map<std::string,unsigned int>
#define SDMAP std::unordered_map<std::string,double>
#define SSMAP std::unordered_map<std::string,std::string>

// This class listens for sensor definitions and constructs the sensors
class SensorDefConsumer : public SEConsumer {
	private:
	SLIST sns;		// sensor name [list of strings]
	SSMAP sts;		// sensor type [sn->str]
	SDMAP ssigs;	// sensor sigma: standard deviation [sn->double]
	SSMAP snd1s;	// point node or from node for flow sensors [sn->str]
	SSMAP snd2s;	// point node or to node for flow sensors [sn->str]
	SDMAP svals;	// value of the latest measurement [sn->double]
	uint numms = 0; // number of sensors
	// to add a sensor
	//	-- sns.push_back(sn);
	//	-- sts[sn] = st;
	//	-- ssigs[sn] = ssig;
	//	-- snd1s[sn] = snd1;
	//	-- snd2s[sn] = snd2;
	//	-- svals[sn] = sval;
	
	public:
	SensorDefConsumer(const string& brokerURI, 
				const string& username,
				const string& password,
				const string& target,
				const string& mode) {
		this->brokerURI = brokerURI;
		this->username = username;
		this->password = password;
		this->mode = mode;
	}

	public:
	void fillSens(uint& numms, SLIST& sns, SSMAP& sts, SDMAP& ssigs,
			SSMAP& snd1s, SSMAP& snd2s, SDMAP& svals) {
		numms = this->numms;
		sns = this->sns;
		sts = this->sts;
		ssigs = this->ssigs;
		snd1s = this->snd1s;
		snd2s = this->snd2s;
		svals = this->svals;
	}
	
	public:
	virtual void process(const string& text) {
		// --------------------------------------------------------------------
		// PARSE THE MESSAGE AND INITIALIZE SENSORS
		// --------------------------------------------------------------------
		cout << ">Recieved sensor message: \n\t\"" << text << "\"\n";

		// if the message is what we are looking for, process and release
		string fn = "demo/13Bus/static_measurements.csv";
		cout << "Loading sensor definitions from file: " << fn << "\n";
		cout << "\t...\n";
		
		// --------------------------------------------------------------------
		// LOAD THE SENSORS
		// --------------------------------------------------------------------
		// for now, read the sensor objects from a file
		std::ifstream sfs;
		sfs.open(fn,std::ifstream::in);
		if ( !sfs ) throw "failed to open static measurement file";
		std::string sfs1;
		std::getline(sfs,sfs1);
		std::string st,sn,snd1,snd2;
		double sval,ssig;
		std::cout << sfs1;
		while ( sfs >> st >> sn >> snd1 >> snd2 >> sval >> ssig ) {
			sns.push_back(sn);
			numms++;
			sts[sn] = st;
			ssigs[sn] = ssig;
			snd1s[sn] = snd1;
			snd2s[sn] = snd2;
			svals[sn] = sval;
		}
		// END read the sensor objects from a file
		
		cout << "\tSensors loaded.\n";
		
		// --------------------------------------------------------------------
		// SENSOR INITIALIZATION COMPLETE
		// --------------------------------------------------------------------
		// release latch
		doneLatch.countDown();
	}
};

#endif
