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

#define SLIST std::list<std::string>
#define SIMAP std::unordered_map<std::string,uint>
#define SDMAP std::unordered_map<std::string,double>
#define SCMAP std::unordered_map<std::string,std::complex<double>>
#define SBMAP std::unordered_map<std::string,bool>
#define SSMAP std::unordered_map<std::string,std::string>
#define ISMAP std::unordered_map<uint,std::string>
#define IDMAP std::unordered_map<uint,double>
#define IMDMAP std::unordered_map<uint,IDMAP>
#define ICMAP std::unordered_map<uint,std::complex<double>>
#define IMMAP std::unordered_map<uint,ICMAP>
#define SSLISTMAP std::unordered_map<std::string,SLIST>

// global logging stream--either a file or stdout based on invocation
std::ostream* selog = &std::cout;

// include files for all interfaces
#include "SensorArray.hpp"

#ifdef GRIDAPPSD_INTERFACE
// include files for the GridAPPS-D interface
#include "SharedQueue.hpp"
#include "json.hpp"
  using json = nlohmann::json;
#include "SEConsumer.hpp"
#include "GenericConsumer.hpp"
#include "TopoProcConsumer.hpp"
#include "VnomConsumer.hpp"
#include "SensorDefConsumer.hpp"
#include "SEProducer.hpp"
#include "state_estimator_gridappsd.hpp"
  using state_estimator_gridappsd::gridappsd_session;
#include "gridappsd_requests.hpp"
  using gridappsd_requests::sparql_query;
#include "sparql_queries_CIM100.hpp"
#include "state_estimator_util.hpp"
#include "SELoopConsumer.hpp"
#endif

#include "PlatformInterfaceBase.hpp"
#ifdef FILE_INTERFACE
#include "PlatformInterfaceFile.hpp"
#endif
#ifdef GRIDAPPSD_INTERFACE
#include "PlatformInterfaceGridAPPSD.hpp"
#endif

// more include files for all interfaces
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
#ifdef FILE_INTERFACE_NOSBASE
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
    *selog << "Starting the SE work loop\n" << std::flush;
#endif
    loopWorker.workLoop();

    // we'll never get here
    return 0;
}
