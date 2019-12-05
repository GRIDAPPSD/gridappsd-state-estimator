#ifndef STATE_HPP
#define STATE_HPP

#include <string>
#include <list>
#include <unordered_map>

// macro for unsigned int
#define uint unsigned int

// store object names and hash names to different pieces of information
#define SLIST std::list<std::string>
#define SDMAP std::unordered_map<std::string,double>
#define SSMAP std::unordered_map<std::string,std::string>

class State {
	public:
	SLIST xids;		// list of state IDs nmrid+"_"+[phase]+"_"+type
	SSMAP xidxs;	// indexes for vectors like x
	SSMAP xmrids;	// bus (or object) mirds
	SSMAP xphs;		// phases
	SSMAP xtypes;	// types (e.g. "Vmag", "Varg")
	SDMAP xvals;	// value of the estimate
	uint xqty = 0;	// number of states
};

#endif
