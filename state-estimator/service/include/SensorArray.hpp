#ifndef SENSORARRAY_HPP
#define SENSORARRAY_HPP

#include <string>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// store object names and hash names to different pieces of information
#define SLIST std::list<std::string>
#define SBMAP std::unordered_map<std::string,bool>
#define SDMAP std::unordered_map<std::string,double>
#define SSMAP std::unordered_map<std::string,std::string>

class SensorArray {
	public:
	SLIST mmrids;	// list of measurement mrids
	SSMAP mtypes;	// measurement types (e.g. "PNV")
	SLIST zids;		// list of measurement component IDs mmrid+"_"+ztype
	SIMAP zidxs;	// indexes for vectors like z and h
	SSMAP ztypes;	// measurement component types (e.g. "Vmag")
	SDMAP zsigs;	// standard deviations of measurements
	SSMAP znode1s;	// point node or from node for flow measurements
	SSMAP znode2s;	// point node or to node for flow measurements
	SDMAP zvals;	// value of the latest measurement
	SBMAP znew;		// indicator for new measurements
	uint zqty = 0;	// number of measurement components
};


#endif
