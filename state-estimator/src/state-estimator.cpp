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

// files-based interface related conditional compilation values
#ifdef FILE_INTERFACE
//#define FILE_INTERFACE_READ "test_4Ti"
//#define FILE_INTERFACE_READ "test_3p6_Ti"
//#define FILE_INTERFACE_READ "test_4PiQi"
//#define FILE_INTERFACE_READ "test_13assets_noaji"
//#define FILE_INTERFACE_READ "test_11big_jl"
//#define FILE_INTERFACE_READ "test_4"
//#define FILE_INTERFACE_READ "test_4vinj"
//#define FILE_INTERFACE_READ "test_4net"
//#define FILE_INTERFACE_READ "test_4sbase"
//#define FILE_INTERFACE_READ "test_13assets"
//#define FILE_INTERFACE_READ "test_11full"
//#define FILE_INTERFACE_READ "test_11diff"
//#define FILE_INTERFACE_READ "test_11noQ"
//#define FILE_INTERFACE_READ "test_4withB"
//#define FILE_INTERFACE_READ "test_4woB"
//#define FILE_INTERFACE_READ "test_11big"
//#define FILE_INTERFACE_READ "test_3p6"
//#define FILE_INTERFACE_READ "test_3p6pseudo"
//#define FILE_INTERFACE_READ "test_11_bus_full"
//#define FILE_INTERFACE_READ "test_11_bus_diff"
//#define FILE_INTERFACE_READ "test_11_bus_full_meas"
//#define FILE_INTERFACE_READ "test_11_bus_diff_meas"
//#define FILE_INTERFACE_READ "test_4_bus_full"
//#define FILE_INTERFACE_READ "test_4_bus_diff"
//#define FILE_INTERFACE_READ "test_3p6_bus_full"
//#define FILE_INTERFACE_READ "test_3p6_bus_diff"
//#define FILE_INTERFACE_READ "test_3p6_bus_full_meas"
//#define FILE_INTERFACE_READ "test_3p6_bus_diff_meas"
#define FILE_INTERFACE_READ "test_files_123"
//#define FILE_INTERFACE_READ "test_files_13assets"

#ifndef FILE_INTERFACE_READ
#include "OOPS, NEED TO DEFINE FILE_INTERFACE_READ!"
#endif

// whether to get node_vnoms from file or hardwire to 1
#define FILE_INTERFACE_VNOM
// the nosbase symbol is used for a model outside GridAPPS-D like the
// 4-bus MATLAB model
//#define FILE_INTERFACE_NOSBASE
#endif

//#define FILE_INTERFACE_WRITE

#define PI 3.141592653589793

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
#include "SharedQueue.hpp"
#include "json.hpp"
  using json = nlohmann::json;

#ifdef GRIDAPPSD_INTERFACE
// include files for the GridAPPS-D interface
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
    // Common/shared interface code

    // Node bus mRIDs and phases data structures
    SSMAP node_bmrids;
    SSMAP node_phs;

    // Topology data structures
    uint node_qty;      // number of nodes
    SLIST node_names;   // list of node names
    SIMAP node_idxs;    // map from node name to unit-indexed position
    ISMAP node_name_lookup;
    IMMAP Yphys;        // double map from indices to complex admittance
    // G, B, g, and b are derived from Yphys:
    //    -- Gij = std::real(Yphys[i][j]);
    //    -- Bij = std::imag(Yphys[i][j]);
    //    -- gij = std::real(-1.0*Yphys[i][j]);
    //    -- bij = std::imag(-1.0*Yphys[i][j]);

    // Node nominal voltages data structure
    SCMAP node_vnoms;

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

    // --------------------------------------------------------------------
    // SENSOR INITIALIZATION
    // --------------------------------------------------------------------

    // A matrix data structures
    IMDMAP Amat;
    SSMAP regid_primnode;
    SSMAP regid_regnode;

    // Sensors data structures
    SensorArray zary;
    SSMAP mmrid_pos_type;
    SSMAP switch_node1s;
    SSMAP switch_node2s;

    // declare the thread-safe queue shared between SELoopConsumer (writer)
    // and SELoopWorker (reader)
    SharedQueue<json> measQueue;

    PlatformInterface plint(argc, argv, sbase);

    plint.setupMeasurements(measQueue);

    plint.fillTopology(Yphys, node_qty, node_names, node_idxs, node_name_lookup,
        node_bmrids, node_phs);

    plint.fillVnom(node_vnoms);

    plint.fillSensors(zary, Amat, regid_primnode, regid_regnode,
        mmrid_pos_type, switch_node1s, switch_node2s);

    // Initialize class that does the state estimates
    SELoopWorker loopWorker(plint, zary, node_qty, node_names,
        node_idxs, node_vnoms, node_bmrids, node_phs, node_name_lookup,
        sbase, Yphys, Amat, regid_primnode, regid_regnode,
        mmrid_pos_type, switch_node1s, switch_node2s);

#ifdef DEBUG_PRIMARY
    *selog << "Starting the SE work loop\n" << std::flush;
#endif
    loopWorker.workLoop();

    // we'll never get here
    return 0;
}
