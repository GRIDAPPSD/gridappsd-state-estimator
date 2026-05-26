#define DIAGONAL_P
#define SWITCHES
//#define NET_INJECTION
#ifdef NET_INJECTION
//#define COMPARE_INJ_MEAS
#endif

#define DEBUG_PRIMARY
#define DEBUG_STATS
#if defined(DEBUG_PRIMARY) && defined(DEBUG_STATS)
#define TEST_SUITE
#endif
//#define DEBUG_FILES
//#define DEBUG_SIZES

//#define SBASE_TESTING

#define GS_OPTIMIZE
// whether to store negligable values in sparse matrices or not
// defining USE_NEGL indicates not to store negligable values
//#define USE_NEGL // be careful using this as it can throw away needed values

// GDB 12/18/25: Can't set WRITE_FILES for a file interface build because
// that is designed to consume file and not generate them--chaos results if
// they are both set
#ifndef FILE_INTERFACE
#define WRITE_FILES
#endif

// This PLATFORM_OLD define is used for old SPARQL queries such as for the
// v2023.07.0 platform. It should not be set for the v2025.01.0 platform
// and newer.
//#define PLATFORM_OLD
// This PLATFORM_NEW define is used for platorms after v2025.01.0 where
// multiple simulations were supported
#define PLATFORM_NEW
// different sparql queries are needed if the GridAPPS-D platform version
// pre-dates CIM-Graph. v2023.07.0 requires OLD_PLATFORM to be defined while
// v2025.01.0 does not.

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
#ifdef GADAL_INTERFACE
#include "PlatformInterfaceGADAL.hpp"
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
    //const double sbase = 1.0e+6; // my go-to sbase for a long time
    //const double sbase = 1.0e+7; // for 9500 model, lowest condition #
    const double sbase = 1.0e+8; // best 9500 model % error results
#endif
#endif

    // GDB 12/18/25: Cut down on default 6 digits of precision when helpful for
    // comparing logs between runs
    *selog << std::setprecision(16);

    PlatformInterface plint(argc, argv, sbase);

    plint.setupMeasurements();

    plint.fillTopology();

    plint.fillVnoms();

    plint.fillSensors();

    plint.setupPublishing();

    // Initialize class that does the state estimates
    SELoopWorker loopWorker(&plint);

#ifdef DEBUG_PRIMARY
    *selog << "Starting the State Estimator work loop...\n" << std::flush;
#endif
    loopWorker.workLoop();

    // we'll never get here
    return 0;
}

