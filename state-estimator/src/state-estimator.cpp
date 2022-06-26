#define DIAGONAL_P
#define SWITCHES
//#define NET_INJECTION
#ifdef NET_INJECTION
//#define COMPARE_INJ_MEAS
#endif

#define DEBUG_PRIMARY
//#define DEBUG_FILES
//#define DEBUG_SIZES

//#define SBASE_TESTING

#define GS_OPTIMIZE
// whether to store negligable values in sparse matrices or not
// defining USE_NEGL indicates not to store negligable values
//#define USE_NEGL

//#define WRITE_FILES

#include <iostream>
#include <fstream>
#include <regex>
  using std::regex;
#include <string>
  using std::string;
#include <complex>
  using std::complex;
#include <list>
#include <unordered_map>
#ifdef DEBUG_PRIMARY
#include <unistd.h>
#endif

// global logging stream--either a file or stdout based on invocation
std::ostream* selog = &std::cout;

// abstract platform interface class that specific interaces must implement
#include "PlatformInterfaceBase.hpp"

// platform-specific interface implementations
#ifdef FILE_INTERFACE
#include "PlatformInterfaceFile.hpp"
#endif
#ifdef GRIDAPPSD_INTERFACE
#include "PlatformInterfaceGridAPPSD.hpp"
#endif

// include files for all interfaces
#include "SELoopWorker.hpp"


int main(int argc, char** argv) {
    // system base power, functionally arbitrary -- can be tweaked
    // for better numerical stability if needed
    // all values in the approximate range 1e-140 to 1e+150 converge
    // and only numeric overflow/underflow results in failures for the
    // 3 models tested (ieee13nodecktassets, ieee123, test9500new)
    // values in the 1e+6 to 1e+12 range result in minimum Supd condition
    // numbers with the range for lowest condition varying somewhat between
    // the 3 models tested
#ifdef SBASE_TESTING
    double spower = (double)std::stoi(argv[3]);
    const double sbase = pow(10.0, spower);
#else
#ifdef SBASE_NONE
    const double sbase = 1;
#else
    const double sbase = 1.0e+6;
#endif
#endif

    PlatformInterface plint(argc, argv, sbase);

    plint.setupMeasurements();

    plint.fillTopology();

    plint.fillVnoms();

    plint.fillSensors();

    // Initialize class that does the state estimates
    SELoopWorker loopWorker(&plint);

#ifdef DEBUG_PRIMARY
    *selog << "Starting the State Estimator work loop...\n" << std::flush;
#endif
    loopWorker.workLoop();

    // we'll never get here
    return 0;
}

