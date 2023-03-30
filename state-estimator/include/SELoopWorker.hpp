#ifndef SELOOPWORKER_HPP
#define SELOOPWORKER_HPP
#include <cfloat>
#include <iomanip>

#include "cs.h"
#include "klu.h"

// standard data types
#include <array>

// for mkdir and opendir
#if defined(DEBUG_FILES) || defined(TEST_SUITE)
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#endif

#ifdef DEBUG_PRIMARY
#include <sys/time.h>
#endif

#ifndef A5MAP
#define A5MAP std::multimap<uint,std::array<uint, 5>>
#define A5PAIR std::pair<uint, std::array<uint, 5>>
#endif

// macros defining the negligable value versions of C-sparse functions
#ifdef USE_NEGL
#define NEGL 1.0e-16
//#define NEGL 1.0e-8 // this can throw away some needed values so be careful
#ifdef GS_OPTIMIZE
//#define gs_entry_diagonal_negl(A,ij,val) if (val>NEGL || -val>NEGL) gs_entry_diagonal(A,ij,val); else gs_entry_diagonal(A,ij,0.0)
//#define gs_entry_firstcol_negl(A,i,val) if (val>NEGL || -val>NEGL) gs_entry_firstcol(A,i,val); else gs_entry_firstcol(A,i,0.0)
//#define gs_entry_fullsquare_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_fullsquare(A,i,j,val); else gs_entry_fullsquare(A,i,j,0.0)
//#define gs_entry_colorder_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_colorder(A,i,j,val); else gs_entry_colorder(A,i,j,0.0)
#define gs_entry_diagonal_negl(A,ij,val) if (val>NEGL || -val>NEGL) gs_entry_diagonal(A,ij,val)
#define gs_entry_firstcol_negl(A,i,val) if (val>NEGL || -val>NEGL) gs_entry_firstcol(A,i,val)
#define gs_entry_fullsquare_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_fullsquare(A,i,j,val)
#define gs_entry_colorder_negl(A,i,j,val) if (val>NEGL || -val>NEGL) gs_entry_colorder(A,i,j,val)
#else
//#define cs_entry_negl(A,i,j,val) if (val>NEGL || -val>NEGL) cs_entry(A,i,j,val); else cs_entry(A,i,j,0.0)
#define cs_entry_negl(A,i,j,val) if (val>NEGL || -val>NEGL) cs_entry(A,i,j,val)
#endif
#else
#ifdef GS_OPTIMIZE
#define gs_entry_diagonal_negl(A,ij,val) gs_entry_diagonal(A,ij,val)
#define gs_entry_firstcol_negl(A,i,val) gs_entry_firstcol(A,i,val)
#define gs_entry_fullsquare_negl(A,i,j,val) gs_entry_fullsquare(A,i,j,val)
#define gs_entry_colorder_negl(A,i,j,val) gs_entry_colorder(A,i,j,val)
#else
#define cs_entry_negl(A,i,j,val) cs_entry(A,i,j,val)
#endif
#endif

// macro to set precision of value to a fixed number of decimal digits
//#define SET_PRECISION12(val) round(val*1e+12)/1e+12
//#define SET_PRECISION8(val) round(val*1e+8)/1e+8
//#define SET_PRECISION6(val) round(val*1e+6)/1e+6

//double SET_SIGNIFICANT(double value, uint digits) {
//    if (value == 0.0)
//        return 0.0;

//    double factor = pow(10.0, digits - ceil(log10(fabs(value))));
//    return round(value*factor)/factor;
//}

// This class listens for system state messages
class SELoopWorker {
    private:

    // passed in from constructor
    PlatformInterface* plint;
    SensorArray        Zary;
    SLIST              node_names;   // node names [list of strings]
    uint               node_qty;     // number of nodes
    SIMAP              node_idxs;    // node positional indices [node->int]
    SCMAP              node_vnoms;   // complex nominal voltages of nodes
    SSMAP              node_bmrids;  // string mRIDs of the associated buses
    SSMAP              node_phs;     // string phases
    ISMAP              node_name_lookup;
    double             sbase;
    IMMAP              Yphys;        // Ybus [node->[row->col]] [physical units]
    // G, B, g, and b are derived from Yphys:
    //    -- Gij = std::real(Yphys[i][j]);
    //    -- Bij = std::imag(Yphys[i][j]);
    //    -- gij = std::real(-1.0*Yphys[i][j]);
    //    -- bij = std::imag(-1.0*Yphys[i][j]);
    IMDMAP             Amat;         // regulator tap ratios
    SSMAP              regid_primnode;
    SSMAP              regid_regnode;
    SSMAP              mmrid_pos_type; // type of position measurement
    SSMAP              switch_node1s;
    SSMAP              switch_node2s;

    // system topology definition
    uint xqty;          // number of states
    uint zqty;          // number of measurements

    // establish jacobian entry types
    enum dydx_type : uint {
        dPi_dvi, dPi_dvj, dQi_dvi, dQi_dvj, dvi_dvi,
        dPi_dTi, dPi_dTj, dQi_dTi, dQi_dTj, dTi_dTi,
        daji_dvj, daji_dvi,
    };

    A5MAP Jshapemap;   // column ordered map of J entries: {zidx,xidx,i,j,dydx_type}

    // system state
    IMMAP Ypu;
    ICMAP Vpu;          // voltage state in per-unit
#ifdef DIAGONAL_P
    IDMAP Uvmag;        // variance of voltage magnitudes (per-unit)
    IDMAP Uvarg;        // variance of voltage angles (per-unit)
#else
    cs *Pmat=NULL;      // x comes from V and A but P is persistent 
#endif
    IMDMAP Bmat;         // binary switch state matrix

    cs *Fmat;           // process model
    cs *Rmat;           // measurement covariance (diagonal)
    cs *eyex;           // identity matrix of dimension x

    bool firstEstimateFlag = true;
#ifdef SBASE_TESTING
    uint estimateExitCount = 0;
#endif

#if defined(DEBUG_FILES) || defined(TEST_SUITE)
    std::ofstream state_fh;  // file to record states
#endif

#ifdef WRITE_FILES
    std::ofstream results_fh;  // file to record results
#endif

#ifdef TEST_SUITE
    std::ofstream testinit_accy_fh;  // file to record accuracy before estimate
    std::ofstream testest_accy_fh;   // file to record accuracy for estimates
    std::ofstream testest_perf_fh;   // file to record performance for estimates
#endif

    public:
    SELoopWorker(PlatformInterface* plint) {
        this->plint = plint;
        this->Zary = plint->getZary();
        this->node_names = plint->getnode_names();
        this->node_qty = this->node_names.size();
        this->node_idxs = plint->getnode_idxs();
        this->node_vnoms = plint->getVnoms();
        this->node_bmrids = plint->getnode_bmrids();
        this->node_phs = plint->getnode_phs();
        this->node_name_lookup = plint->getnode_name_lookup();
        this->sbase = plint->getsbase();
        this->Yphys = plint->getYphys();
        this->Amat = plint->getAmat();
        this->regid_primnode = plint->getregid_primnode();
        this->regid_regnode = plint->getregid_regnode();
        this->mmrid_pos_type = plint->getmmrid_pos_type();
        this->switch_node1s = plint->getswitch_node1s();
        this->switch_node2s = plint->getswitch_node2s();
    }


    public:
    void workLoop() {
        uint timestamp, timestampLastEstimate, timeZero;
        bool exitAfterEstimateFlag = false;
        bool doEstimateFlag;
        bool reclosedFlag;
        bool keepZvalsFlag = false;
        uint estimatesSinceReset = 0;

        // do one-time-only processing
        init();

        // initialize "last timestamp" map to a "no measurements" flag value
        for ( auto& zid : Zary.zids )
            Zary.ztimes[zid] = UINT_MAX;

        // initialize what's updated during processing
        // (if things go bad we'll need to reset these)
        initVoltagesAndCovariance();

        for (;;) {
            // ----------------------------------------------------------------
            // Reset the new measurement counter for each node
            // ----------------------------------------------------------------
            doEstimateFlag = false;
            reclosedFlag = false;

            // check whether to preserve zvals from last queue draining
            if ( !keepZvalsFlag )
                for ( auto& zid : Zary.zids ) Zary.znews[zid] = 0;
            else
                keepZvalsFlag = false;

            // drain the queue with quick z-averaging
            do {
                if ( plint->fillMeasurement() ) {
                    timestamp = plint->getmeas_timestamp();
                    if (firstEstimateFlag) {
                        timeZero = timestamp;
                        // set flag value to increase uncertainty in first
                        // estimate call
                        timestampLastEstimate = UINT_MAX;
                        firstEstimateFlag = false;
                    }

                    if (add_zvals(timestamp, timeZero))
                        reclosedFlag = true;

                    // set flag to indicate a full estimate can be done
                    // if a COMPLETED/CLOSED log message is received
                    doEstimateFlag = true;

                } else {
                    if (doEstimateFlag) {
#ifdef DEBUG_PRIMARY
                        *selog << "Got COMPLETED for simulation status, doing full estimate with previous measurement\n" << std::flush;
#endif
                        // set flag to exit after completing full estimate below
                        exitAfterEstimateFlag = true;

                        // we've already done the add_zvals call for the last
                        // measurement so proceed to estimate z-averaging + estimate
                        break;
                    } else {
#ifdef DEBUG_PRIMARY
                        *selog << "Got COMPLETED for simulation status, normal exit because full estimate just done\n" << std::flush;
#endif
                        exit(0);
                    }
                }
            } while (plint->nextMeasurementWaiting());

            // do z averaging here by dividing sum by # of items
// #ifdef DEBUG_PRIMARY
//             *selog << "===========> z-averaging being done after draining queue\n" << std::flush;
// #endif

            for ( auto& zid : Zary.zids ) {
                if ( Zary.znews[zid] > 1 )
                    Zary.zvals[zid] /= Zary.znews[zid];
            }

// #ifdef DEBUG_PRIMARY
//            *selog << "zvals before estimate\n" << std::flush;
//            for ( auto& zid : Zary.zids ) {
//                *selog << "measurement of type: " << Zary.ztypes[zid] << "\t" << zid << ": " << Zary.zvals[zid] << "\t(" << Zary.znews[zid] << ")\n" << std::flush;
//            }
// #endif

            if (reclosedFlag) {
#ifdef DEBUG_PRIMARY
                // friendly reset of selected matrices analogous to klu_error
                *selog << "\nSwitch reclosed--soft reset for state estimation\n" << std::flush;
#endif

                // things went bad so reset what was previously updated
                softReset();

                // set flag value to increase uncertainty in next estimate call
                timestampLastEstimate = UINT_MAX;
            }

            // do the core "estimate" processing here since the queue is,
            // for the moment, empty
            // ----------------------------------------------------------------
            // Estimate the state
            // ----------------------------------------------------------------
#ifdef DEBUG_PRIMARY
            *selog << "\nEstimating state for timestep: " << timestamp-timeZero << "\n" << std::flush;
#endif
            try {
                if ( estimate(timestamp, timestampLastEstimate, timeZero,
                              estimatesSinceReset) ) {
                    // successful estimate call so increment counter
                    estimatesSinceReset++;

                    // not the first estimate call so set to actual value
                    timestampLastEstimate = timestamp;
                    //sleep(30); // delay to let queue refill for testing
                    publish(timestamp);
                } else {
                    // unsuccessful estimate call so reset counter
                    estimatesSinceReset = 0;

#ifdef DEBUG_PRIMARY
                    // friendly reset of selected matrices
                    *selog << "\nResidual exceeded threshold--soft reset for state estimation\n" << std::flush;
#endif
                    // things went bad so reset what was previously updated
                    softReset();

                    // set flag value to increase uncertainty in next estimate
                    timestampLastEstimate = 0;

                    // undo the zvals averaging because we'll drain what's been
                    // added before the next estimate call and average those
                    // with what's been drained already
                    for ( auto& zid : Zary.zids ) {
                        if ( Zary.znews[zid] > 1 )
                            Zary.zvals[zid] *= Zary.znews[zid];
                    }

                    // set flag to indicate not to clear previous zvals
                    keepZvalsFlag = true;
                }
            } catch(const char* msg) {
#ifdef DEBUG_PRIMARY
                *selog << "\nCaught klu_error exception--soft reset for state estimation\n" << std::flush;
#endif

                // things went bad so reset what was previously updated
                softReset();

                // set flag value to increase uncertainty in first estimate call
                timestampLastEstimate = 0;

                // start fresh with new estimates from the top of the loop
            }

            if (exitAfterEstimateFlag) {
#ifdef DEBUG_PRIMARY
                *selog << "Normal exit after COMPLETE/CLOSED log message and full estimate\n" << std::flush;
#endif
                exit(0);
            }
        }
    }


    private:
    void init() {
#if defined(DEBUG_FILES) || defined(TEST_SUITE)
        // create the output directory if needed
        if (!opendir("output")) {
            // if output is already a symbolic link to a shared
            // folder as intended, this won't be needed
            mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }

        // create simulation parent directory
        string simpath = "output/" + plint->getOutputDir() + "/";
        mkdir(simpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

#ifdef DEBUG_FILES
        // create init directory
        string initpath = simpath + "init/";
        mkdir(initpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
#ifdef TEST_SUITE
        // create test_suite directory
        string testpath = simpath + "test_suite/";
        mkdir(testpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        testinit_accy_fh.open(simpath+"test_suite/init_accy.csv",std::ofstream::out);
        testinit_accy_fh << "nodeqty,xqty,zqty,Jacobian_elements,Yphys_scaled_terms,F_width,F_height,F_entries,F_min,F_max,F_mean,eyex_width,eyex_height,eyex_entries,eyex_min,eyex_max,eyex_mean,R_width,R_height,R_entries,R_min,R_max,R_mean\n";
        testinit_accy_fh.close();
#endif
#endif

        // --------------------------------------------------------------------
        // Establish Dimension of State Space and Measurement Space
        // --------------------------------------------------------------------
        xqty = 2*node_qty;
        zqty = Zary.zids.size();

#ifdef DEBUG_PRIMARY
        *selog << "node_qty is " << node_qty << "; " << std::flush;
        *selog << "xqty is " << xqty << "; " << std::flush;
        *selog << "zqty is " << zqty << "\n" << std::flush;
#endif
#ifdef TEST_SUITE
        testinit_accy_fh.open(simpath+"test_suite/init_accy.csv",std::ofstream::app);
        testinit_accy_fh << node_qty << "," << xqty << "," << zqty;
        testinit_accy_fh.close();
#endif

        // --------------------------------------------------------------------
        // Determine possible non-zero Jacobian elements
        // --------------------------------------------------------------------
        for ( auto& zid: Zary.zids ) {
            uint zidx = Zary.zidxs[zid];            // row index of J
            string ztype = Zary.ztypes[zid];        // measurement type
            uint i = node_idxs[Zary.znode1s[zid]];  // one-indexed

            // Real Power Injection Measurements
            if ( !ztype.compare("Pi") ) {
                for ( auto& row_pair : Yphys[i] ) {
                    uint j = row_pair.first;
                    if ( j == i ) {
                        // dPi/dvi and dPi/dTi exist for node i

                        Jshapemap.insert(A5PAIR(j-1, {zidx,j-1,i,i,dPi_dvi}));
                        Jshapemap.insert(A5PAIR(node_qty+j-1, {zidx,node_qty+j-1,i,i,dPi_dTi}));
                    } else {
                        // dPi/dvj and dPi/dTj exist for nodes adjacent to i
                        Jshapemap.insert(A5PAIR(j-1, {zidx,j-1,i,j,dPi_dvj}));
                        Jshapemap.insert(A5PAIR(node_qty+j-1, {zidx,node_qty+j-1,i,j,dPi_dTj}));
                    }
                }
            } else if ( !ztype.compare("Qi") ) {
                for ( auto& row_pair : Yphys[i] ) {
                    uint j = row_pair.first;
                    if ( j == i ) {
                        // dPi/dvi and dPi/dTi exist for node i
                        Jshapemap.insert(A5PAIR(j-1, {zidx,j-1,i,i,dQi_dvi}));
                        Jshapemap.insert(A5PAIR(node_qty+j-1, {zidx,node_qty+j-1,i,i,dQi_dTi}));
                    } else {
                        // dPi/dvj and dPi/dTj exists for nodes adjacent to i
                        Jshapemap.insert(A5PAIR(j-1, {zidx,j-1,i,j,dQi_dvj}));
                        Jshapemap.insert(A5PAIR(node_qty+j-1, {zidx,node_qty+j-1,i,j,dQi_dTj}));
                    }
                }
            } else if ( !ztype.compare("aji") ) {
                // note: the regulation node, j, is assigned to znode2s
                //       the primary node, i, is assigned to znode1s
                uint j = node_idxs[Zary.znode2s[zid]];

                // daji/dvj exists: 1/vi
                Jshapemap.insert(A5PAIR(j-1, {zidx, j-1, i, j, daji_dvj}));

                // daji/dvi exists: -vj/vi^2
                Jshapemap.insert(A5PAIR(i-1, {zidx, i-1, i, j, daji_dvi}));

            } else if ( !ztype.compare("vi") ) {
                // dvi/dvi is the only partial that exists
                Jshapemap.insert(A5PAIR(i-1, {zidx,i-1,i,i,dvi_dvi}));
            } else if ( !ztype.compare("Ti") ) {
                // dTi/dTi is the only partial that exists
                Jshapemap.insert(A5PAIR(node_qty+i-1, {zidx,node_qty+i-1,i,i,dTi_dTi}));
            } else if ( !ztype.compare("switch_ij") ) {
                // if switch is open, possibility of distributed generation
                // means the relationship between vi and vj is indeterminant
                // this piecewise relationship is not differentiable for calc_J
            } else { 
                *selog << "\tERROR: Jshapemap unrecognized measurement type: " <<
                          ztype << "\n" << std::flush;
                exit(1);
            }
        }
#ifdef DEBUG_PRIMARY
        *selog << "Jshapemap Jacobian elements: " << Jshapemap.size() << "\n" << std::flush;
#endif
#ifdef TEST_SUITE
        testinit_accy_fh.open(simpath+"test_suite/init_accy.csv",std::ofstream::app);
        testinit_accy_fh << "," << Jshapemap.size();
        testinit_accy_fh.close();
#endif

        // Cap Yphys with magnitude greater than 1e3 threshold
        double thresh = 1e+3;
#ifdef DEBUG_PRIMARY
        uint ctr = 0;
        //*selog << "Yphys scaling started...\n" << std::flush;
#endif
        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            for ( auto& jpair : Yphys[i] ) {
                uint j = jpair.first;
                if ( j == i )
                    continue;

                complex<double> term_val = jpair.second;
                double term_mag = abs(term_val);

                if ( term_mag <= thresh )
                    continue;
#ifdef DEBUG_PRIMARY
                ctr++;
                //*selog << "Yphys scaling down row: " << i << ", col: " << j << "\n" << std::flush;
#endif
                double scaler = thresh / term_mag;
                // update the term
                complex<double> new_term_val = term_val * scaler;
#ifdef DEBUG_PRIMARY
                //*selog << "\tYphys scaler: " << scaler << "\n" << std::flush;
                //*selog << "\tYphys term_val: " << abs(term_val) << "(" << 180/M_PI*arg(term_val) << ")\n" << std::flush;
                //*selog << "\tYphys new_term_val: " << abs(new_term_val) << "(" << 180/M_PI*arg(new_term_val) << ")\n" << std::flush;
#endif
                Yphys[i][j] = new_term_val;

                // update the diagonal
                complex<double> diag_term_val = Yphys[i][i];
                complex<double> delta_term_val = new_term_val - term_val;
                string jnode = node_name_lookup[j];
                complex<double> delta_diag_val = -1.0 * delta_term_val * node_vnoms[jnode]/node_vnoms[inode];
                complex<double> new_diag_term_val = diag_term_val + delta_diag_val;
                Yphys[i][i] = new_diag_term_val;
            }
        }
#ifdef DEBUG_PRIMARY
        *selog << "Yphys # of scaled terms: " << ctr << "\n" << std::flush;
#endif
#ifdef TEST_SUITE
        testinit_accy_fh.open(simpath+"test_suite/init_accy.csv",std::ofstream::app);
        testinit_accy_fh << "," << ctr;
        testinit_accy_fh.close();
#endif

        // Overwrite Yphys entries for switches to (-500,500)
        complex<double> Yover = complex<double>(-500,500);
        for (auto& switch_node1_pair : switch_node1s) {
            string zid = switch_node1_pair.first;
            uint i = node_idxs[switch_node1_pair.second];
            uint j = node_idxs[switch_node2s[zid]];

            try {
                auto& Yrow = Yphys.at(i);
                complex<double> yij = Yrow.at(j);

            } catch ( const std::out_of_range& oor ) {
                Yphys[i][j] = Yphys[j][i] = Yover;
                complex<double> new_term_val = Yover;
                complex<double> term_val = 0;

                // common terms for updating diagonals
                string inode = node_name_lookup[i];
                string jnode = node_name_lookup[j];
                complex<double> delta_term_val = new_term_val - term_val;
                complex<double> diag_term_val, delta_diag_val,new_diag_term_val;

                // update the i,i diagonal
                diag_term_val = Yphys[i][i];
                delta_diag_val = -1.0 * delta_term_val * node_vnoms[jnode]/node_vnoms[inode];
                new_diag_term_val = diag_term_val + delta_diag_val;
                Yphys[i][i] = new_diag_term_val;

                // update the j,j diagonal
                diag_term_val = Yphys[j][j];
                delta_diag_val = -1.0 * delta_term_val * node_vnoms[inode]/node_vnoms[jnode];
                new_diag_term_val = diag_term_val + delta_diag_val;
                Yphys[j][j] = new_diag_term_val;
            }
        }

        // --------------------------------------------------------------------
        // Compute Ypu
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        *selog << "Computing Ypu time -- " << std::flush;
        double startTime = getWallTime();
#endif
        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            complex<double> conjterm = conj(node_vnoms[inode]/sbase);
            auto& row = Yphys.at(i);
            for ( auto& jpair : row ) {
                // jpair consists of <j><yij>
                uint j = jpair.first;
                complex<double> yij = jpair.second;
                string jnode = node_name_lookup[j];
                Ypu[i][j] = conjterm * yij * node_vnoms[jnode];
            }
        }
#ifdef DEBUG_PRIMARY
        *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef WRITE_FILES
        std::ofstream wofh;
        wofh.open("test_files/ypu.csv", std::ofstream::out);
        wofh << std::setprecision(10);  // match OpenDSS ysparse
        wofh << "Row,Col,G,B\n";
        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            try {
                auto& row = Ypu.at(i);
                for ( auto& jnode : node_names ) {
                    uint j = node_idxs[jnode];
                    try {
                        complex<double> tmp = row.at(j);
                        double tmpre = tmp.real();
                        double tmpim = tmp.imag();
                        if (j >= i) // only output the lower diagonal
                            wofh << j << "," << i << "," << tmpre << "," << tmpim << "\n";
                    } catch ( const std::out_of_range& oor ) {
                    }
                }
            } catch ( const std::out_of_range& oor ) {
            }
        } wofh.close();

        wofh.open("test_files/yphys.csv", std::ofstream::out);
        wofh << std::setprecision(10);  // match OpenDSS ysparse
        wofh << "Row,Col,G,B\n";
        for ( uint i = 1 ; i <= node_qty ; i++ ) {
            try {
                auto& row = Yphys.at(i);
                for ( uint j = 1 ; j <= node_qty ; j++ ) {
                    try {
                        complex<double> tmp = row.at(j);
                        double tmpre = tmp.real();
                        double tmpim = tmp.imag();
                        if (j >= i) // only output the lower diagonal
                            wofh << j << "," << i << "," << tmpre << "," << tmpim << "\n";
                    } catch ( const std::out_of_range& oor ) {
                    }
                }
            } catch ( const std::out_of_range& oor ) {
            }
        } wofh.close();
#endif
#ifdef DEBUG_FILES
        // write to file
        std::ofstream ofh;
        ofh.open(initpath+"Ypu.csv",std::ofstream::out);
        ofh << std::setprecision(16);

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            try {
                auto& row = Ypu.at(i);
                uint jctr = 0;
                for ( auto& jnode : node_names ) {
                    uint j = node_idxs[jnode];
                    try {
                        complex<double> tmp = row.at(j);
                        double tmpre = tmp.real();
                        double tmpim = tmp.imag();
                        ofh << tmpre 
                            << ( tmpim >= 0 ? "+" : "-" )
                            << std::abs(tmpim) << "i" 
                            << ( ++jctr < node_qty ? "," : "\n" );
                    } catch ( const std::out_of_range& oor ) {
                        ofh << "0+0i" << ( ++jctr < node_qty ? "," : "\n" );
                    }
                }
            } catch ( const std::out_of_range& oor ) {
                uint jctr = 0;
                for ( auto& jnode : node_names )
                    ofh << "0+0i" << ( ++jctr < node_qty ? "," : "\n" );
            }
        } ofh.close();
#endif

#ifdef DEBUG_FILES
        // write Y to file
        ofh.open(initpath+"Yphys.csv",std::ofstream::out);
        ofh << std::setprecision(16);
        *selog << "writing " << initpath+"Yphys.csv\n" << std::flush;
        for ( uint i = 1 ; i <= node_qty ; i++ ) {
            try {
                auto& row = Yphys.at(i);
                for ( uint j = 1 ; j <= node_qty ; j++ ) {
                    try {
                        complex<double> tmp = row.at(j);
                        double tmpre = tmp.real();
                        double tmpim = tmp.imag();
                        ofh << tmpre
                            << ( tmpim >= 0 ? "+" : "-" )
                            << std::abs(tmpim) << "i"
                            << ( j < node_qty ? "," : "\n" );
                    } catch ( const std::out_of_range& oor ) {
                        ofh << "0+0i" << ( j < node_qty ? "," : "\n" );
                    }
                }
            } catch ( const std::out_of_range& oor ) {
                for ( uint j = 0 ; j < node_qty ; j++ )
                    ofh << "0+0i" << ( j < node_qty ? "," : "\n" );
            }
        } ofh.close();
#endif

#ifdef DEBUG_FILES
        // write Vbase to file
        ofh.open(initpath+"vnoms.csv",std::ofstream::out);
        ofh << std::setprecision(16);
        *selog << "writing " << initpath+"vnoms.csv\n" << std::flush;
        std::vector<complex<double>> vnoms(node_qty);
        for ( auto& node_name : node_names ) {
            *selog << node_name << "\n" << std::flush;
            *selog << "\tidx is " << node_idxs[node_name] << "\n" << std::flush;
            *selog << "\tvnom is " << node_vnoms[node_name] << "\n" << std::flush;
            vnoms[node_idxs[node_name]-1] = node_vnoms[node_name];
        }
        for ( complex<double>& vnom : vnoms ) {
            double re = vnom.real();
            double im = vnom.imag();
            ofh << re
                << ( im >= 0 ? "+" : "-" )
                << std::abs(im) << "i"
                << "\n";
        } ofh.close();
#endif
            
#ifdef DEBUG_FILES
        // write the node map to file
        ofh.open(initpath+"nodem.csv",std::ofstream::out);
        *selog << "writing " << initpath+"nodem.csv\n" << std::flush;
        for ( auto& node_name : node_names )
            ofh << node_name << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor information to file
        ofh.open(initpath+"meas.txt",std::ofstream::out);
        *selog << "writing output/init/meas.txt\n" << std::flush;
        ofh << "sensor_type\tsensor_name\tnode1\tnode2\tvalue\tsigma\n";
        for ( auto& zid : Zary.zids ) {
            ofh << Zary.ztypes[zid] << "\t"
                << zid << "\t"
                << Zary.znode1s[zid] << "\t"
                << Zary.znode2s[zid] << "\t"
                << Zary.zvals[zid] << "\t"
                << Zary.zsigs[zid] << "\n";
        } ofh.close();

        *selog << "done writing\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        // write sensor types to file
        ofh.open(initpath+"ztypes.csv");
        for ( auto& zid : Zary.zids )
            ofh << Zary.ztypes[zid] << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor node 1s
        ofh.open(initpath+"znode1s.csv");
        for ( auto& zid : Zary.zids )
            ofh << Zary.znode1s[zid] << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor node 2s
        ofh.open(initpath+"znode2s.csv");
        for ( auto& zid : Zary.zids )
            ofh << Zary.znode2s[zid] << "\n";
        ofh.close();
#endif

        // --------------------------------------------------------------------
        // Initialize cs variables for state estimation
        // --------------------------------------------------------------------
        // state transition matrix (constant)
#ifdef DEBUG_PRIMARY
        *selog << "Initializing F -- " << std::flush;
#endif
#ifdef GS_OPTIMIZE
        Fmat = gs_singleval_diagonal(xqty, 1.0);
#else
        cs* Fraw = cs_spalloc(xqty, xqty, xqty, 1, 1);
        for (uint i=0; i < xqty; i++)
            cs_entry_negl(Fraw, i, i, 1.0);
        Fmat = cs_compress(Fraw);
        cs_spfree(Fraw);
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Fmat,initpath+"F.csv");
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(Fmat, "F", true);
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Fmat, "F", true);
#endif

        // identity matrix of dimension x (constant)
#ifdef DEBUG_PRIMARY
        *selog << "Initializing eyex -- " << std::flush;
#endif
#ifdef GS_OPTIMIZE
        eyex = gs_singleval_diagonal(xqty, 1.0);
#else
        cs* eyexraw = cs_spalloc(xqty, xqty, xqty, 1, 1);
        for (uint i=0; i < xqty; i++)
            cs_entry_negl(eyexraw, i, i, 1.0);
        eyex = cs_compress(eyexraw);
        cs_spfree(eyexraw);
#endif
#ifdef DEBUG_FILES
        print_cs_compress(eyex,initpath+"eyex.csv");
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(eyex, "eyex", true);
#endif
#ifdef DEBUG_STATS
        print_cs_stats(eyex, "eyex", true);
#endif

        // R is the measurement covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        *selog << "Initializing R -- " << std::flush;
#endif
#ifdef GS_OPTIMIZE
        Rmat = gs_spalloc_diagonal(zqty);
        for ( auto& zid : Zary.zids ) {
            // variance of R[i,i] is sigma[i]^2
            // Originally we had this as just sigma[i], which resulted in
            // different sbase producing different estimate results and took
            // a couple weeks to debug working through many matrices to figure
            // out if they were different for different sbase values
            gs_entry_diagonal_negl(Rmat,Zary.zidxs[zid],
                                   Zary.zsigs[zid]*Zary.zsigs[zid]);
            //*selog << zid << "," << Zary.zidxs[zid] << "," << node_idxs[Zary.znode1s[zid]] << "," << Zary.zsigs[zid]*Zary.zsigs[zid] << "\n" << std::flush;
        }
#else
        cs* Rraw = cs_spalloc(zqty, zqty, zqty, 1, 1);
        for ( auto& zid : Zary.zids )
            // variance of R[i,i] is sigma[i]^2
            // Originally we had this as just sigma[i], which resulted in
            // different sbase producing different estimate results and took
            // a couple weeks to debug working through many matrices to figure
            // out if they were different for different sbase values
            cs_entry_negl(Rraw,Zary.zidxs[zid],Zary.zidxs[zid],
                          Zary.zsigs[zid]*Zary.zsigs[zid]);
        Rmat = cs_compress(Rraw);
        cs_spfree(Rraw);
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Rmat,initpath+"R.csv");
        //print_cs_compress_triples(Rmat,"R_sbase1e6_trip.csv", 8);
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(Rmat, "R", true);
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Rmat, "R", true);
#endif

#ifdef DEBUG_FILES
        // --------------------------------------------------------------------
        // Initialize the state recorder file
        // --------------------------------------------------------------------
        state_fh.open(simpath+"vmag_pu.csv",std::ofstream::out);
        state_fh << "timestamp,";
        uint nctr = 0;
        for ( auto& node_name : node_names )
            state_fh << "\'"+node_name+"\'" << ( ++nctr < node_qty ? "," : "\n" );
        state_fh.close();
#endif
#ifdef TEST_SUITE
        testest_accy_fh.open(simpath+"test_suite/est_accy.csv",std::ofstream::out);
        testest_accy_fh << "timestamp,";
        testest_accy_fh << "P_width,P_height,P_entries,P_min,P_max,P_mean,";
        testest_accy_fh << "P1_width,P1_height,P1_entries,P1_min,P1_max,P1_mean,";
        testest_accy_fh << "P2_width,P2_height,P2_entries,P2_min,P2_max,P2_mean,";
        testest_accy_fh << "P3_width,P3_height,P3_entries,P3_min,P3_max,P3_mean,";
        testest_accy_fh << "Q_width,Q_height,Q_entries,Q_min,Q_max,Q_mean,";
        testest_accy_fh << "Ppre_width,Ppre_height,Ppre_entries,Ppre_min,Ppre_max,Ppre_mean,";
        testest_accy_fh << "x_width,x_height,x_entries,x_min,x_max,x_mean,";
        testest_accy_fh << "xpre_width,xpre_height,xpre_entries,xpre_min,xpre_max,xpre_mean,";
        testest_accy_fh << "J_width,J_height,J_entries,J_min,J_max,J_mean,";
        testest_accy_fh << "S1_width,S1_height,S1_entries,S1_min,S1_max,S1_mean,";
        testest_accy_fh << "S2_width,S2_height,S2_entries,S2_min,S2_max,S2_mean,";
        testest_accy_fh << "S3_width,S3_height,S3_entries,S3_min,S3_max,S3_mean,";
        testest_accy_fh << "R_min,R_max,R_mean,";
        testest_accy_fh << "Supd_width,Supd_height,Supd_entries,Supd_min,Supd_max,Supd_mean,";
        testest_accy_fh << "Supd_condnum,";
        testest_accy_fh << "K3_width,K3_height,K3_entries,K3_min,K3_max,K3_mean,";
        testest_accy_fh << "K2_width,K2_height,K2_entries,K2_min,K2_max,K2_mean,";
        testest_accy_fh << "Kupd_width,Kupd_height,Kupd_entries,Kupd_min,Kupd_max,Kupd_mean,";
        testest_accy_fh << "z_width,z_height,z_entries,z_min,z_max,z_mean,";
        testest_accy_fh << "h_width,h_height,h_entries,h_min,h_max,h_mean,";
        testest_accy_fh << "yupd_width,yupd_height,yupd_entries,yupd_min,yupd_max,yupd_mean,";
        testest_accy_fh << "x1_width,x1_height,x1_entries,x1_min,x1_max,x1_mean,";
        testest_accy_fh << "xupd_width,xupd_height,xupd_entries,xupd_min,xupd_max,xupd_mean,";
        testest_accy_fh << "P4_width,P4_height,P4_entries,P4_min,P4_max,P4_mean,";
        testest_accy_fh << "P5_width,P5_height,P5_entries,P5_min,P5_max,P5_mean,";
        testest_accy_fh << "Pupd_width,Pupd_height,Pupd_entries,Pupd_min,Pupd_max,Pupd_mean,";
        testest_accy_fh << "meas_min,meas_max,meas_mean,est_min,est_max,est_mean,est_pererr\n";
        testest_accy_fh.close();

        testest_perf_fh.open(simpath+"test_suite/est_perf.csv",std::ofstream::out);
        testest_perf_fh << "timestamp,Supd_time,Supd_mem,Kupd_time,Kupd_mem,est_time,est_mem\n";
        testest_perf_fh.close();

        state_fh.open(simpath+"test_suite/vmag_pu.csv",std::ofstream::out);
        state_fh << "timestamp,";
        uint tctr = 0;
        for ( auto& node_name : node_names )
            state_fh << "\'"+node_name+"\'" << ( ++tctr < node_qty ? "," : "\n" );
        state_fh.close();
#endif
#ifdef WRITE_FILE
        string filename = "test_files/results_data.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Writing results to test harness file: " << filename << "\n\n" << std::flush;
#endif
        results_fh.open(filename,std::ofstream::out);
        results_fh << "timestamp,";
        nctr = 0;
        for ( auto& node_name : node_names )
            results_fh << "vmag_"+node_name+",";
        for ( auto& node_name : node_names )
            results_fh << "varg_"+node_name << ( ++nctr < node_qty ? "," : "\n" );
        results_fh.close();
#endif
    }


    private:
    void initVoltagesAndCovariance() {
        // --------------------------------------------------------------------
        // Initialize Voltages (complex per-unit)
        // --------------------------------------------------------------------
        // clear previous values if this was called before
        if ( Vpu.size() > 0 ) Vpu.clear();

        for ( auto& node_name : node_names ) {
            // Important: V is indexed by node index like A and Y
            // V[node_idxs[node_name]] = node_vnoms[node_name];
            Vpu[node_idxs[node_name]] = 1.0;
        }
#ifdef DEBUG_PRIMARY
        *selog << "Voltages Initialized.\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        // print initial state vector
        std::ofstream ofh;
        string simpath = "output/" + plint->getOutputDir() + "/";
        string initpath = simpath + "init/";
        ofh.open(initpath+"Vpu.csv",std::ofstream::out);
        ofh << std::setprecision(16);
        *selog << "writing " << initpath+"Vpu.csv\n\n" << std::flush;
        *selog << "node_qty is " << node_qty << "\n" << std::flush;

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            *selog << inode << " idx is " << i << "\n" << std::flush;
            try {
                complex<double> tmp = Vpu.at(i);
                double tmpre = tmp.real();
                double tmpim = tmp.imag();
                ofh << tmpre
                    << ( tmpim >= 0 ? "+" : "-" )
                    << std::abs(tmpim) << "i"
                    << "\n";
            } catch ( const std::out_of_range& oor ) {
                ofh << "0+0i" << "\n";
            }
        } ofh.close();
#endif

        // --------------------------------------------------------------------
        // Initialize State Covariance Matrix
        // --------------------------------------------------------------------
        double span_vmag = 1.0;
        double span_varg = 1.0/3.0*M_PI;
        double span_taps = 0.2;

        // scaling factor for state uncertainty initialization
        double span_multiplier = 0.2*0.2;
#ifdef DIAGONAL_P
        // clear previous values if this was called before
        if ( Uvmag.size() > 0 ) Uvmag.clear();
        if ( Uvarg.size() > 0 ) Uvarg.clear();

        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            Uvmag[idx] = span_multiplier*span_vmag;
            Uvarg[idx] = span_multiplier*span_varg;
        }
#else
        // clear previous values if this was called before
        cs_spfree(Pmat);
#ifdef GS_OPTIMIZE
        Pmat = gs_doubleval_diagonal(node_qty, span_multiplier*span_vmag, span_multiplier*span_varg);
#else
        cs *Praw = cs_spalloc (2*node_qty, 2*node_qty, 2*node_qty, 1, 1);
        for (uint i = 0; i < node_qty; i++) {
            cs_entry_negl(Praw, i, i, span_multiplier*span_vmag);
            cs_entry_negl(Praw, node_qty+i, node_qty+i, span_multiplier*span_varg);
        }
        Pmat = cs_compress(Praw);
        cs_spfree(Praw);
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P,initpath+"Pinit.csv");
#endif
#endif

#ifdef DEBUG_PRIMARY
        *selog << "State Covariance Matrix Initialized.\n\n" << std::flush;
#endif
    }


    private:
    void softReset() {
        // reset Amat to where it started
        // iterate over map of maps setting all entries back to 1
        for (auto& ent1 : Amat)
            for (auto& ent2 : ent1.second)
                ent2.second = 1;

        initVoltagesAndCovariance();
    }


    private:
    bool add_zvals(const uint& timestamp, const uint& timeZero) {
        // --------------------------------------------------------------------
        // Use the simulation output to update the states
        // --------------------------------------------------------------------
        // This needs to translate simulation output into Zary.zvals[zid]
        //  - We need to iterate over the "measurements" in the simoutput
        //  - As in SensorDefConsumer.hpp, measurements can have multiple z's

        bool ret = false;

        SLIST meas_mrids = plint->getmeas_mrids();
        SDMAP meas_magnitudes = plint->getmeas_magnitudes();
        SDMAP meas_angles = plint->getmeas_angles();
        SDMAP meas_values = plint->getmeas_values();
#ifdef WRITE_FILES
        SDMAP node_mag, node_ang;
#endif

        for ( auto& mmrid : meas_mrids ) {
            if ( !Zary.mtypes[mmrid].compare("") ) {
                if (Zary.znews[mmrid] == 0)
                    Zary.zvals[mmrid] = meas_magnitudes[mmrid];
                else
                    Zary.zvals[mmrid] += meas_magnitudes[mmrid];
                Zary.znews[mmrid]++;
                Zary.ztimes[mmrid] = timestamp;

                if (mmrid.substr(mmrid.length()-4, string::npos) == "_tap") {
                    // Update the A matrix with the latest tap ratio measurement
                    double tap_ratio = meas_magnitudes[mmrid];

                    string primnode = Zary.znode1s[mmrid];
                    uint i = node_idxs[primnode];

                    string regnode = Zary.znode2s[mmrid];
                    uint j = node_idxs[regnode];

                    if ( ( Amat[j][i] - tap_ratio ) > 0.00625 )
                        Amat[j][i] -= 0.00625;
                    else if ( ( Amat[j][i] - tap_ratio ) < -0.00625 )
                        Amat[j][i] += 0.00625;
                    else
                        Amat[j][i] = tap_ratio;
                }

            } else if ( !Zary.mtypes[mmrid].compare("PNV") ) {
                if (meas_magnitudes.find(mmrid) != meas_magnitudes.end()) {
                    // update the voltage magnitude (in per-unit)
                    string zid = mmrid+"_Vmag";
                    double vmag_phys = meas_magnitudes[mmrid];
                    double vmag_nom = vmag_phys / abs(node_vnoms[Zary.znode1s[zid]]);
#ifdef WRITE_FILES
                    string meas_node = Zary.mnodes[mmrid];
                    node_mag[meas_node] = vmag_nom;
                    // assumes there is always an angle with the magnitude,
                    // which isn't the case with sensor simulator measurements
                    double vang_phys = meas_angles[mmrid];
                    double vang_rad = vang_phys*M_PI/180.0;
                    node_ang[meas_node] = vang_rad;
#endif
                    // TODO: This uses vnom filled from OpenDSS values, but
                    // needs to use GridLAB-D values
                    if (Zary.znews[zid] == 0)
                        Zary.zvals[zid] = vmag_nom;
                    else
                        Zary.zvals[zid] += vmag_nom;
                    Zary.znews[zid]++;
                    Zary.ztimes[zid] = timestamp;
                }

                // update the voltage phase
                // --- LATER ---
                // -------------

            } else if ( !Zary.mtypes[mmrid].compare("Pos") ) {
                if ( !mmrid_pos_type[mmrid].compare("regulator_tap") ) {
                    // update the tap ratio
                    string zid = mmrid+"_tap";
                    double tap_position = meas_values[mmrid];
                    double tap_ratio = 1.0 + 0.1*tap_position/16.0;

                    if (Zary.znews[zid] == 0)
                        Zary.zvals[zid] = tap_ratio;
                    else
                        Zary.zvals[zid] += tap_ratio;
                    Zary.znews[zid]++;
                    Zary.ztimes[zid] = timestamp;

                    // Update the A matrix with the latest tap ratio measurement
                    // TODO: Consider averaging for queued measurements
                    string primnode = Zary.znode1s[zid];
                    uint i = node_idxs[primnode];

                    string regnode = Zary.znode2s[zid];
                    uint j = node_idxs[regnode];
                    
                    if ( ( Amat[j][i] - tap_ratio ) > 0.00625 ) {
//                        *selog << "\t***Setting Amat[" << regnode << "][" << primnode 
//                                << "] to " << Amat[j][i] - 0.00625 << " (rate limited)" 
//                                << '\n' << std::flush;
                        Amat[j][i] -= 0.00625;
                    }
                    else if ( ( Amat[j][i] - tap_ratio ) < -0.00625 ) {
//                        *selog << "\t***Setting Amat[" << regnode << "][" << primnode 
//                                << "] to " << Amat[j][i] + 0.00625 << " (rate limited)" 
//                                << '\n' << std::flush;
                        Amat[j][i] += 0.00625;
                    }
                    else {
//                        *selog << "\t***Setting Amat[" << regnode << "][" << primnode 
//                                << "] to " << tap_ratio << '\n' << std::flush;
                        Amat[j][i] = tap_ratio;
                    }

                } else if ( !mmrid_pos_type[mmrid].compare("load_break_switch") ) {
                    string zid = mmrid+"_switch";
                    double switch_state = meas_values[mmrid];
                    uint i = node_idxs[switch_node1s[zid]];
                    uint j = node_idxs[switch_node2s[zid]];

                    // detect switch reclosing to trigger soft reset
                    if (switch_state > 0) {
                        try {
                            auto Brow = Bmat.at(i);
                            try {
                                double bval = std::real(Brow.at(j));
                                if (bval < 1) {
#ifdef DEBUG_PRIMARY
//                                    *selog << "\tJUST CLOSED SWITCH Bmat[" << i << "][" << j << "] = " << switch_state << ", switch_node1s: " << switch_node1s[zid] << ", switch_node2s: " << switch_node2s[zid] << "\n" << std::flush;
#endif
                                    ret = true;
                                }
                            } catch ( const std::out_of_range& oor ) {}
                        } catch ( const std::out_of_range& oor ) {}
                    }
#if 000
                    else {
                        try {
                            auto Brow = Bmat.at(i);
                            try {
                                double beeij = std::real(Brow.at(j));
                                if (beeij > 0) {
#ifdef DEBUG_PRIMARY
                                    *selog << "\tJUST OPENED SWITCH Bmat[" << i << "][" << j << "] = " << switch_state << ", switch_node1s: " << switch_node1s[zid] << ", switch_node2s: " << switch_node2s[zid] << "\n" << std::flush;
#endif
                                }
                            } catch ( const std::out_of_range& oor ) {}
                        } catch ( const std::out_of_range& oor ) {}
                    }
#endif

                    Bmat[i][j] = Bmat[j][i] = switch_state;
//                    *selog << "\t***Setting Bmat[" << i << "][" << j << "] = " << Bmat[i][j] << ", switch_node1s: " << switch_node1s[zid] << ", switch_node2s: " << switch_node2s[zid] << "\n" << std::flush;
#if 000
                    string match="";
                    if (switch_state>0 && abs(Yphys[i][j])==0)
                        match = "MISMATCH: ";

                    *selog << match << "measurement load_break_switch zid: " << zid << ", i: " << i << ", j: " << j << "\n";
                    *selog << match << "measurement load_break_switch value: " << switch_state << "\n";
                    *selog << match << "measurement load_break_switch node1s: " << switch_node1s[zid] << "\n";
                    *selog << match << "measurement load_break_switch node2s: " << switch_node2s[zid] << "\n";
                    *selog << match << "measurement load_break_switch Ypu: " << Ypu[i][j] << "\n";
                    *selog << match << "measurement load_break_switch Yphys: " << Yphys[i][j] << "\n\n";
#endif
                }
            }
#ifdef NET_INJECTION
            else if ( !Zary.mtypes[mmrid].compare("VA") ) {
                if ( !Zary.mcetypes[mmrid].compare("EnergyConsumer") ) {
                    // P and Q injection measurements are composed of physical
                    // measurements of all devices at the node.

                    // TODO: it would be possible to have physical measurements
                    // from a subset of devices at a node so we need to figure
                    // out how to combine the physical and pseudo-measurements
                    // (existing and/or new).
                    // We have one existing Pi and one existing Qi measurement
                    // for every node.  Assuming we have physical measurements
                    // for all existing devices at a node, we could overwrite
                    // the pseudo-measurement with an accumulator over physical
                    // measurements at that node.

                    string meas_node = Zary.mnodes[mmrid];
                    string pinj_zid = meas_node+"_Pinj";
                    string qinj_zid = meas_node+"_Qinj";

                    // assumes pinj and qinj are updated in pairs only
                    // or we would have to check qinj_zid as well
                    if (Zary.ztimes[pinj_zid] != timestamp) {
                        Zary.zvals[pinj_zid] = 0;
                        Zary.zvals[qinj_zid] = 0;
                        Zary.znews[pinj_zid]++;
                        Zary.znews[qinj_zid]++;
                        Zary.ztimes[pinj_zid] = timestamp;
                        Zary.ztimes[qinj_zid] = timestamp;
                    }

                    if (meas_magnitudes.find(mmrid) != meas_magnitudes.end() &&
                        meas_angles.find(mmrid) != meas_angles.end()) {
                        double vmag_phys = meas_magnitudes[mmrid];
                        double vang_phys = meas_angles[mmrid];
                        double vang_rad = vang_phys*M_PI/180.0;

                        // convert from polar to rectangular coordinates
                        Zary.zvals[pinj_zid] -= vmag_phys*cos(vang_rad)/sbase;
                        Zary.zvals[qinj_zid] -= vmag_phys*sin(vang_rad)/sbase;
                    }
                }
                // check other conducting equipment types
                // else if ( !Zary.mcetypes[mmrid].compare("") ) {
                // }
            }
#endif
        }

#ifdef WRITE_FILES
        // write simulation_data files from a running simulation for use
        // with test harness
        // The simulation_data file is used to plot directly against results
        // in SE test harness runs without the platform
        static bool firstTimeFlag = true;
        std::ofstream ofh_data;
        string filename;
        uint ctr = 0;
        uint num_nodes = node_names.size();

        if (firstTimeFlag) {
            //firstTimeFlag = false;
            ofh_data.open("test_files/simulation_data.csv", std::ofstream::out);

            ofh_data << "timestamp,";
            for ( auto& node_name : node_names )
                ofh_data << "vmag_" << node_name << ",varg_" << node_name << ( ++ctr < num_nodes ? "," : "\n" );
            ofh_data.close();

        }
        ofh_data.open("test_files/simulation_data.csv", std::ofstream::app);
        ofh_data << std::setprecision(16);

        ofh_data << timestamp << ",";

        ctr = 0;
        for ( auto& node_name : node_names )
            ofh_data << node_mag[node_name] << "," << node_ang[node_name] << ( ++ctr < num_nodes ? "," : "\n" );

        ofh_data.close();

        // write measurement_data files from a running simulation for use
        // with test harness
        // The measurement_data file is used to generate estimate results
        // in SE test harness runs without the platform
        ctr = 0;

        if (firstTimeFlag) {
            firstTimeFlag = false;
            ofh_data.open("test_files/measurement_data.csv", std::ofstream::out);

            ofh_data << "timestamp,";
            for ( auto& zid : Zary.zids )
                ofh_data << zid << ( ++ctr < zqty ? "," : "\n" );
            ofh_data.close();

        }
        ofh_data.open("test_files/measurement_data.csv", std::ofstream::app);
        ofh_data << std::setprecision(16);

        ofh_data << timestamp << ",";

        ctr = 0;
        for ( auto& zid : Zary.zids )
            ofh_data << Zary.zvals[zid] << ( ++ctr < zqty ? "," : "\n" );

        ofh_data.close();
#endif

#ifdef COMPARE_INJ_MEAS
        *selog << "timestamp";
        for (auto& node : Zary.injnodes )
            *selog << ",pseudo_P_"+node << "," << "pseudo_Q_"+node << "," << node+"_Pinj" << "," << node+"_Qinj";
        *selog << "\n";

        *selog << timestamp;
        for (auto& node : Zary.injnodes )
            *selog << "," << sbase*Zary.zvals["pseudo_P_"+node] << "," << sbase*Zary.zvals["pseudo_Q_"+node] << "," << sbase*Zary.zvals[node+"_Pinj"] << "," << sbase*Zary.zvals[node+"_Qinj"];
        *selog << "\n" << std::flush;
#endif

        return ret;
    }


    private:
    double normalizeAngle(const double& radians) {
        double degrees = 180.0/M_PI * radians;
        // -165 <= degrees <= 195
        while (degrees > 195.0) degrees -= 360.0;
        while (degrees < -165.0) degrees += 360.0;

        return degrees;
    }


    private:
    void publish(const uint& timestamp) {
        // --------------------------------------------------------------------
        // Package and publish the state
        // --------------------------------------------------------------------
        SDMAP est_v, est_angle;
        SDMAP est_vvar, est_anglevar;
        SDMAP est_vmagpu, est_vargpu;

#ifdef DEBUG_PRIMARY
        double measMinMag = DBL_MAX;
        double measMaxMag = DBL_MIN;
        double measSumMag = 0.0;
        double estMinMag = DBL_MAX;
        double estMaxMag = DBL_MIN;
        double estSumMag = 0.0;
        double diffSumMag = 0.0;
        double estMinArg = DBL_MAX;
        double estMaxArg = DBL_MIN;
        double estSumArg = 0.0;
        uint numSum = 0;

        SDMAP meas_vmagpu;
        for ( auto& zid : Zary.zids )
            if ( !Zary.ztypes[zid].compare("vi") )
                meas_vmagpu[Zary.znode1s[zid]] = Zary.zvals[zid];
#endif

        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            complex<double> vnom = node_vnoms[node_name];

            est_v[node_name] = abs( vnom * Vpu[idx] );
            est_angle[node_name] = normalizeAngle(arg( vnom * Vpu[idx] ));

            est_vvar[node_name] = Uvmag[idx];
            est_anglevar[node_name] = normalizeAngle(Uvarg[idx]);

            est_vmagpu[node_name] = abs( Vpu[idx] );
            est_vargpu[node_name] = arg( Vpu[idx] );

#ifdef DEBUG_PRIMARY
            if (meas_vmagpu.find(node_name) != meas_vmagpu.end()) {
                measMinMag = fmin(measMinMag, meas_vmagpu[node_name]);
                measMaxMag = fmax(measMaxMag, meas_vmagpu[node_name]);
                measSumMag += meas_vmagpu[node_name];

                estMinMag = fmin(estMinMag, est_vmagpu[node_name]);
                estMaxMag = fmax(estMaxMag, est_vmagpu[node_name]);
                estSumMag += est_vmagpu[node_name];
                diffSumMag += abs(est_vmagpu[node_name] - meas_vmagpu[node_name]);

                estMinArg = fmin(estMinArg, est_vargpu[node_name]);
                estMaxArg = fmax(estMaxArg, est_vargpu[node_name]);
                estSumArg += est_vargpu[node_name];

                numSum++;
            }
#endif
        }
#ifdef DEBUG_PRIMARY
        double measMeanMag = measSumMag/numSum;
        double estMeanMag = estSumMag/numSum;
        double perErrMag = 100 * diffSumMag/measSumMag;
        double estMeanArg = estSumArg/numSum;

        *selog << "Meas vmag per-unit min: " << measMinMag << ", max: " << measMaxMag << ", mean: " << measMeanMag << "\n" << std::flush;
        *selog << "Est vmag per-unit min: " << estMinMag << ", max: " << estMaxMag << ", mean: " << estMeanMag << ", % err: " << perErrMag << "\n" << std::flush;
        *selog << "Est varg per-unit min: " << estMinArg << ", max: " << estMaxArg << ", mean: " << estMeanArg << "\n" << std::flush;
#endif

        plint->publishEstimate(timestamp, est_v, est_angle,
            est_vvar, est_anglevar, est_vmagpu, est_vargpu);

#ifdef DEBUG_FILES
        string simpath = "output/" + plint->getOutputDir() + "/";
        state_fh.open(simpath+"vmag_pu.csv",std::ofstream::app);
        state_fh << timestamp << ',';
        uint nctr = 0;
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            state_fh << vmag_pu << ( ++nctr < node_qty ? ',' : '\n' );
        }
        state_fh.close();
#endif
#ifdef TEST_SUITE
        string testpath = "output/" + plint->getOutputDir() + "/test_suite/";
        testest_accy_fh.open(testpath+"est_accy.csv",std::ofstream::app);
        testest_accy_fh << ',' << measMinMag << ',' << measMaxMag << ',' << measMeanMag;
        testest_accy_fh << ',' << estMinMag << ',' << estMaxMag << ',' << estMeanMag << ',' << perErrMag << '\n';
        testest_accy_fh.close();

        state_fh.open(testpath+"vmag_pu.csv",std::ofstream::app);
        state_fh << timestamp << ',';
        uint tctr = 0;
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            state_fh << vmag_pu << ( ++tctr < node_qty ? ',' : '\n' );
        }
        state_fh.close();
#endif

#ifdef WRITE_FILES
        string filename = "test_files/results_data.csv";
        results_fh.open(filename,std::ofstream::app);

        results_fh << timestamp << ',';
        results_fh << std::fixed;
        results_fh << std::setprecision(10);
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            results_fh << vmag_pu << ",";
        }
        uint nctr2 = 0;
        for ( auto& node_name : node_names ) {
            double varg_pu = arg( Vpu[ node_idxs[node_name] ] );
            results_fh << varg_pu << ( ++nctr2 < node_qty ? "," : "\n" );
        }
        results_fh.close();
#endif
    }


    private:
    bool estimate(const uint& timestamp, const uint& timestampLastEstimate,
                  const uint& timeZero, const uint& estimatesSinceReset) {
#ifdef DEBUG_PRIMARY
        double estimateStartTime = getWallTime();
#endif

        // TODO: WE NEED TO HANDLE R-MASK IN HERE SOMEWHERE

#ifdef TEST_SUITE
        string testpath = "output/" + plint->getOutputDir() + "/test_suite/";
        testest_accy_fh.open(testpath+"est_accy.csv",std::ofstream::app);
        testest_accy_fh << timestamp;
        testest_accy_fh.close();

        testest_perf_fh.open(testpath+"est_perf.csv",std::ofstream::app);
        testest_perf_fh << timestamp;
        testest_perf_fh.close();
#endif

#ifdef DEBUG_FILES
        // set filename path based on timestamp
        string simpath = "output/" + plint->getOutputDir() + "/";
        std::ostringstream out;
        out << simpath << timestamp << "/";
        string tspath = out.str();

        // create timestamp directory
        mkdir(tspath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

#ifdef DEBUG_FILES
        std::ofstream ofh;
        ofh.open(tspath+"Vpu.csv",std::ofstream::out);
        ofh << std::setprecision(16);
        *selog << "writing " << tspath+"Vpu.csv\n\n" << std::flush;
        *selog << "node_qty is " << node_qty << "\n" << std::flush;

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            *selog << inode << " idx is " << i << "\n" << std::flush;
            try {
                complex<double> tmp = Vpu.at(i);
                double tmpre = tmp.real();
                double tmpim = tmp.imag();
                ofh << tmpre 
                    << ( tmpim >= 0 ? "+" : "-" )
                    << std::abs(tmpim) << "i" 
                    << "\n";
            } catch ( const std::out_of_range& oor ) {
                ofh << "0+0i" << "\n";
            }
        } ofh.close();
#endif

        // --------------------------------------------------------------------
        // Predict Step
        // --------------------------------------------------------------------
        // -- compute x_predict = F*x | F=I (skipping to improve performance)
        // -- compute p_predict = F*P*F' + Q | F=I (can be simplified)

#ifdef DIAGONAL_P
        // prepare P as a diagonal matrix from state uncertainity
        cs *Pmat; this->prep_P(Pmat);
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(Pmat, "P");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Pmat, "P");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pmat,tspath+"P.csv");
#endif

        cs *P1 = cs_transpose(Fmat,1);
        if (!P1) *selog << "\tERROR: null P1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(P1, "P1");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(P1, "P1");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P1,tspath+"P1.csv");
#endif

        cs *P2 = cs_multiply(Pmat,P1); cs_spfree(Pmat); cs_spfree(P1);
        if (!P2) *selog << "\tERROR: null P2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(P2, "P2");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(P2, "P2");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P2,tspath+"P2.csv");
#endif

        cs *P3 = cs_multiply(Fmat,P2); cs_spfree(P2);
        if (!P3) *selog << "\tERROR: null P3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(P3, "P3");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(P3, "P3");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P3,tspath+"P3.csv");
#endif

       // update process covariance matrix
       cs *Qmat; this->prep_Q(Qmat, timestamp, timestampLastEstimate);
#ifdef DEBUG_PRIMARY
        print_cs_summary(Qmat, "Q");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Qmat, "Q");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Qmat,tspath+"Q.csv");
#endif

        cs *Ppre = cs_add(P3,Qmat,1,1); cs_spfree(P3); cs_spfree(Qmat);
        if (!Ppre) *selog << "\tERROR: null Ppre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(Ppre, "Ppre");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Ppre, "Ppre");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Ppre,tspath+"Ppre.csv");
#endif

        cs *xmat; this->prep_x(xmat);
#ifdef DEBUG_PRIMARY
        print_cs_summary(xmat, "x");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(xmat, "x");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xmat,tspath+"x.csv");
#endif

        cs *xpre = cs_multiply(Fmat,xmat); cs_spfree(xmat);
        if (!xpre) *selog << "\tERROR: null xpre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(xpre, "xpre");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(xpre, "xpre");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xpre,tspath+"xpre.csv");
#endif

        // --------------------------------------------------------------------
        // Update Step
        // --------------------------------------------------------------------

#ifdef GADAL_INTERFACE
        for (uint iter_count=0; iter_count < 1; iter_count++) {
#endif
#ifdef DEBUG_PRIMARY
        *selog << "calc_J time -- " << std::flush;
#endif
        cs *Jmat; this->calc_J(Jmat);
#ifdef DEBUG_PRIMARY
        print_cs_summary(Jmat, "J");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Jmat, "J");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Jmat,tspath+"J.csv");
#endif

        // -- compute S = J*P_predict*J' + R

        cs *S1 = cs_transpose(Jmat,1);
        if (!S1) *selog << "\tERROR: null S1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(S1, "S1");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(S1, "S1");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S1,tspath+"S1.csv");
#endif

        cs *S2 = cs_multiply(Ppre,S1);
        if (!S2) *selog << "\tERROR: null S2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(S2, "S2");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(S2, "S2");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S2,tspath+"S2.csv");
#endif

        cs *S3 = cs_multiply(Jmat,S2); cs_spfree(S2);
        if (!S3) *selog << "\tERROR: null S3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(S3, "S3");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(S3, "S3");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S3,tspath+"S3.csv");
#endif

        // update time uncertainty since last measurement values
        this->prep_R(Rmat, timestamp);
#ifdef DEBUG_STATS
        print_cs_stats(Rmat, "Rmat");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Rmat,tspath+"R.csv");
#endif

        cs *Supd = cs_add(Rmat,S3,1,1); cs_spfree(S3);
        if (!Supd) *selog << "\tERROR: null Supd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(Supd, "Supd");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Supd, "Supd");
#endif
#ifdef DEBUG_FILES
        //print_cs_compress(Supd,tspath+"Supd.csv");
        //print_cs_compress_triples(Supd, "Supd_sbase1e6_trip.csv");
        print_cs_compress_triples(Supd,"Supd_trip.csv");
#endif

        // -- compute K = P_predict*J'*S^-1
        // TODO December 21, 2020 CHECK Supd, Kupd between C++ and MATLAB

#ifdef DEBUG_PRIMARY
        double startTime, sec, gb_used;
        string vm_used, res_used;
#endif

#ifdef GS_OPTIMIZE
        cs *K3 = gs_spalloc_fullsquare(zqty);
        double *rhs = K3->x;
#else
        double *rhs = (double *)calloc(zqty*zqty, sizeof(double));
#endif
        // double condnum;

        try {
            // Initialize klusolve variables
            klu_symbolic *klusym;
            klu_numeric *klunum;
            klu_common klucom;
            if (!klu_defaults(&klucom)) throw "klu_defaults failed";

            klusym = klu_analyze(Supd->m,Supd->p,Supd->i,&klucom);
            if (!klusym) throw "klu_analyze failed";

#ifdef DEBUG_PRIMARY
            *selog << "klu_factor time -- " << std::flush;
            startTime = getWallTime();
#endif
            //stdout_cs_compress(Supd);
            klunum = klu_factor(Supd->p,Supd->i,Supd->x,klusym,&klucom);
            if (!klunum) {
#ifdef DEBUG_PRIMARY
                *selog << "Common->status is: " << klucom.status << "\n" << std::flush;
                if ( klucom.status == 1 ) *selog << "\tERROR: KLU_SINGULAR\n" << std::flush;
#endif
                throw "klu_factor failed";
            }

#ifdef DEBUG_PRIMARY
            *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            // KLU condition number estimation
            (void)klu_condest(Supd->p,Supd->x,klusym,klunum,&klucom);
            *selog << "klu_condest Supd condition number: " << klucom.condest << "\n" << std::flush;
            // condnum = klucom.condest;
            // *selog << timestamp << "," << condnum << ",SupdCondNum\n" << std::flush;
#endif
#ifdef TEST_SUITE
            testest_accy_fh.open(testpath+"est_accy.csv",std::ofstream::app);
            testest_accy_fh << "," << klucom.condest;
            testest_accy_fh.close();
#endif

            // initialize an identity right-hand side
#if 000
            for ( uint ii = 0 ; ii < zqty*zqty ; ii++ )
                rhs[ii] = ii/zqty == ii%zqty ? 1 : 0;
#else
            // assumes all non-diagonal entries are 0 through calloc call
            for ( uint ii = 0 ; ii < zqty ; ii++ )
                rhs[ii*zqty + ii] = 1.0;
#endif
            
#ifdef DEBUG_PRIMARY
            *selog << "klu_solve time (bottleneck) -- " << std::flush;
            startTime = getWallTime();
#endif
            klu_solve(klusym,klunum,Supd->m,Supd->n,rhs,&klucom);
            if (klucom.status) {
#ifdef DEBUG_PRIMARY
                *selog << "Common->status is: " << klucom.status << "\n" << std::flush;
#endif
                throw "klu_solve failed";
            }

#ifdef DEBUG_PRIMARY
            sec = getWallTime() - startTime;
            *selog << getMinSec(sec) << "\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            process_mem_usage(vm_used, res_used, gb_used);
            *selog << "klu_solve virtual memory: " << vm_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
//            *selog << "klu_solve resident memory: " << res_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
#endif

#ifdef TEST_SUITE
            testest_perf_fh.open(testpath+"est_perf.csv",std::ofstream::app);
            testest_perf_fh << "," << sec << "," << gb_used;
            testest_perf_fh.close();
#endif

            // free klusym and klunum or memory leak results
            klu_free_symbolic(&klusym, &klucom);
            klu_free_numeric(&klunum, &klucom);
        } catch (const char *msg) {
            *selog << "\tERROR: KLU message: " << msg << "\n" << std::flush;
            throw "klu_error";
        }
#ifndef GADAL_INTERFACE
        cs_spfree(Supd);
#endif

#if 000
        if (estimateExitCount == 40) {
            estimateExitCount = 0;
            throw "klu_error";
        }
#endif

#ifndef GS_OPTIMIZE
        cs *K3raw = cs_spalloc(zqty, zqty, zqty*zqty, 1, 1);

        for (uint j=0; j<zqty; j++)
            for (uint i=0; i<zqty; i++)
                cs_entry_negl(K3raw,i,j,rhs[j*zqty + i]);
                // transposing i and j gives the same result with either
                // of the following calls
                //cs_entry_negl(K3raw,i,j,rhs[i*zqty + j]);
                //cs_entry_negl(K3raw,j,i,rhs[j*zqty + i]);

        cs *K3 = cs_compress(K3raw);
        cs_spfree(K3raw);
        free(rhs);
#endif

//       // determine number of significant digits based on condition number
//       uint digits = 15 - floor(log10(condnum));
//       // set all elements of Supd^-1 to the desired number of significant
//       // figures for MATLAB comparison
//       for (uint i=0; i<K3->nzmax; i++)
//           K3->x[i] = SET_SIGNIFICANT(K3->x[i], digits);

        if (!K3) *selog << "\tERROR: null K3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(K3, "K3");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(K3, "K3");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K3,tspath+"K3.csv");
        //print_cs_full(K3,"benchmarks/K3.csv");
        //print_cs_compress_triples(K3,"benchmarks/K3_trip.csv");
#endif

        // GDB S1 and K1 are both J transpose so use S1 here
        cs *K2 = cs_multiply(Ppre,S1); cs_spfree(S1);
        if (!K2) *selog << "\tERROR: null K2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(K2, "K2");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(K2, "K2");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K2,tspath+"K2.csv");
        //print_cs_full(K2,"benchmarks/K2.csv");
        //print_cs_compress_triples(K2,"benchmarks/K2_trip.csv");
#endif

#ifdef DEBUG_PRIMARY
        *selog << "Kupd time (bottleneck) -- " << std::flush;
        startTime = getWallTime();
#endif
        cs *Kupd = cs_multiply(K2,K3); cs_spfree(K2); cs_spfree(K3);
        if ( !Kupd ) *selog << "\tERROR: Kupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        sec = getWallTime() - startTime;
        *selog << getMinSec(sec) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(Kupd, "Kupd");
#endif
       // Commented out this precision limiting code per Andy request 2/3/21
       //for (uint i=0; i<Kupd->nzmax; i++)
       //    Kupd->x[i] = SET_PRECISION8(Kupd->x[i]);
#ifdef DEBUG_STATS
        print_cs_stats(Kupd, "Kupd");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Kupd,tspath+"Kupd.csv");
        //print_cs_compress_triples(Kupd, "Kupd_sbase1e12_trip.csv", 4);
#endif
#ifdef DEBUG_PRIMARY
        process_mem_usage(vm_used, res_used, gb_used);
        *selog << "Kupd Peak virtual memory: " << vm_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
        //print_cs_full(Kupd,"benchmarks/Kupd.csv");
        //print_cs_compress_triples(Kupd,"benchmarks/Kupd_trip.csv");
#endif

#ifdef TEST_SUITE
        testest_perf_fh.open(testpath+"est_perf.csv",std::ofstream::app);
        testest_perf_fh << "," << sec << "," << gb_used;
        testest_perf_fh.close();
#endif

        // -- compute y = z - h
        cs *zmat; this->sample_z(zmat);
#ifdef DEBUG_PRIMARY
        print_cs_summary(zmat, "z");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(zmat, "z");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(zmat,tspath+"z.csv");
#endif

#ifdef DEBUG_PRIMARY
        *selog << "calc_h time -- " << std::flush;
#endif
        cs *hmat; this->calc_h(hmat);
#ifdef DEBUG_PRIMARY
        print_cs_summary(hmat, "h");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(hmat, "h");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(hmat,tspath+"h.csv");
#endif

        cs *yupd = cs_add(zmat,hmat,1,-1); cs_spfree(zmat); cs_spfree(hmat);
        if (!yupd) *selog << "\tERROR: null yupd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(yupd, "yupd");
#endif
       // Commented out this precision limiting code per Andy request 2/3/21
       //for (uint i=0; i<yupd->nzmax; i++)
       //    yupd->x[i] = SET_PRECISION8(yupd->x[i]);
#ifdef DEBUG_STATS
        print_cs_stats(yupd, "yupd");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(yupd,tspath+"yupd.csv");
        //print_cs_colvec("yupd_sbase1e6_prec8.csv", yupd);
#endif
        //stdout_cs_compress(yupd);

#ifdef GADAL_INTERFACE
        // check for bad measurement data using normalized residuals
        std::vector<double> residuals_calc(Zary.zids.size());
        double resmin = DBL_MAX;
        double resmax = -DBL_MAX;
        int ii=0;
        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            double yval = yupd->x[zidx];
            double rval = Rmat->x[zidx];

            // iterate over Supd compressed-column data in column zidx
            for ( uint p = Supd->p[zidx] ; p < Supd->p[zidx+1] ; p++  ) {
                // get the row index for existing data
                if ( Supd->i[p] == zidx ) {
                    // if this is the diagonal entry, extract it
                    double sval = Supd->x[p];
                    residuals_calc[ii] = abs(yval)/sqrt(rval*abs(sval));
                    ii++;
                    break;
                }
            }
        }
        resmax = *max_element(residuals_calc.begin(),residuals_calc.end());
        resmin = *min_element(residuals_calc.begin(),residuals_calc.end());
        *selog << "Max resi is " << resmax << "\n" <<std::flush;
        *selog << "Min resi is " << resmin << "\n" <<std::flush;
        cs_spfree(Supd);
#endif

        // Residual calculation needs further investigation to be
        // properly formulated before inclusion in released SE code
#if 000
        // calculate residual of estimate with yupd to check if it exceeds
        // a threshold justifying soft reset

        // normalize yupd by dividing each entry by the square root of the
        // corresponding Rmat and Supd product
        double ressum = 0;
        double ysum = 0;
        double rsum = 0;
        double ssum = 0;
        double dsum = 0;
        double resmin = DBL_MAX;
        double resmax = -DBL_MAX;
        double ymin = DBL_MAX;
        double ymax = -DBL_MAX;
        double rmin = DBL_MAX;
        double rmax = -DBL_MAX;
        double smin = DBL_MAX;
        double smax = -DBL_MAX;
        double dmin = DBL_MAX;
        double dmax = -DBL_MAX;

        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            double yval = yupd->x[zidx];
            double rval = Rmat->x[zidx];

            // iterate over Supd compressed-column data in column zidx
            for ( uint p = Supd->p[zidx] ; p < Supd->p[zidx+1] ; p++ ) {
                // get the row index for existing data
                if ( Supd->i[p] == zidx ) {
                    // if this is the diagonal entry, extract it
                    double sval = Supd->x[p];
                    double residual = abs(yval)/sqrt(rval*abs(sval));
                    ressum += residual;
                    resmin = min(resmin, residual);
                    resmax = max(resmax, residual);
                    ysum += abs(yval);
                    ymin = min(ymin, abs(yval));
                    ymax = max(ymax, abs(yval));
                    rsum += rval;
                    rmin = min(rmin, rval);
                    rmax = max(rmax, rval);
                    ssum += abs(sval);
                    smin = min(smin, abs(sval));
                    smax = max(smax, abs(sval));
                    double denom = sqrt(rval*abs(sval));
                    dsum += denom;
                    dmin = min(dmin, denom);
                    dmax = max(dmax, denom);
                    //*selog << "\nresidual intermediate yval: " << yval << ", sval: " << sval << ", rval: " << rval << ", residual: " << residual << "\n" << std::flush;
                    break;
                }
            }
        }

        double resmean = ressum/zqty;
        *selog << "\nRESIDUAL resmean: " << resmean << ", resmin: " << resmin << ", resmax: " << resmax << "\n" << std::flush;
        *selog << "RESIDUAL ymean: " << ysum/zqty << ", ymin: " << ymin << ", ymax: " << ymax << "\n" << std::flush;
        *selog << "RESIDUAL rmean: " << rsum/zqty << ", rmin: " << rmin << ", rmax: " << rmax << "\n" << std::flush;
        *selog << "RESIDUAL smean: " << ssum/zqty << ", smin: " << smin << ", smax: " << smax << "\n" << std::flush;
        *selog << "RESIDUAL denommean: " << dsum/zqty << ", denommin: " << dmin << ", denommax: " << dmax << "\n" << std::flush;

#if 111
        // when residual is added back in, don't free Supd higher up in
        // estimate call
        cs_spfree(Supd);
#endif

        if ( resmean>200.0 && estimatesSinceReset>5)
            // trigger soft reset back in work loop
            return false;
#endif


        // -- compute x_update = x_predict + K * y

#if 000
        // damp gain matrix
        cs *damptemp = gs_singleval_diagonal(1, 0.2);
        cs *ytemp = cs_multiply(yupd,damptemp); cs_spfree(yupd); cs_spfree(damptemp);
        cs *x1 = cs_multiply(Kupd,ytemp); cs_spfree(ytemp);
#else
        cs *x1 = cs_multiply(Kupd,yupd); cs_spfree(yupd);
#endif

        if ( !x1 ) *selog << "\tERROR: x1 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(x1, "x1");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(x1, "x1");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(x1,tspath+"x1.csv");
#endif

        // TODO: potential future spot for more targeted smart/soft reset

        cs *xupd = cs_add(xpre,x1,1,1); cs_spfree(x1); cs_spfree(xpre);
        if ( !xupd ) *selog << "\tERROR: xupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(xupd, "xupd");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(xupd, "xupd");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xupd,tspath+"xupd.csv");
        //print_cs_compress_sparse(xupd,"xupd_sbase1e6_sparse.csv", 8);
#endif

        // -- compute P_update = (I-K_update*J)*P_predict

#ifdef DEBUG_PRIMARY
        *selog << "P4 time -- " << std::flush;
        startTime = getWallTime();
#endif
        cs *P4 = cs_multiply(Kupd,Jmat); cs_spfree(Kupd); cs_spfree(Jmat);
        if ( !P4 ) *selog << "\tERROR: P4 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        print_cs_summary(P4, "P4");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(P4, "P4");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P4,tspath+"P4.csv");
#endif

        cs *P5 = cs_add(eyex,P4,1,-1); cs_spfree(P4);
        if ( !P5 ) *selog << "\tERROR: P5 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(P5, "P5");
#ifdef DEBUG_STATS
        print_cs_stats(P5, "P5");
#endif
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P5,tspath+"P5.csv");
#endif

#ifdef DIAGONAL_P
        cs *Pupd = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !Pupd ) *selog << "\tERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(Pupd, "Pupd");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Pupd, "Pupd");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pupd,tspath+"Pupd.csv");
#endif

#else
        // re-allocate P for the updated state
        Pmat = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !Pmat ) *selog << "\tERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        print_cs_summary(Pmat, "Pupd");
#endif
#ifdef DEBUG_STATS
        print_cs_stats(Pmat, "Pupd");
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pmat,tspath+"Pupd.csv");
#endif
#endif

        // --------------------------------------------------------------------
        // Update persistent state (Vpu and A)
        // --------------------------------------------------------------------
        if (xupd) {
            decompress_state(xupd);
            cs_spfree(xupd);
        }

#ifdef DIAGONAL_P
        // extract the diagonal of P
        if (Pupd) {
            decompress_variance(Pupd);
            cs_spfree(Pupd);
        }
#endif

#ifdef DEBUG_PRIMARY
        sec = getWallTime() - estimateStartTime;
        *selog << "*** Total estimate time: " <<
            getMinSec(sec) << ", timestamp: " <<
            timestamp << ", timestep: " <<
            timestamp-timeZero << "\n" << std::flush;
#ifdef DEBUG_SIZES
#ifndef DIAGONAL_P
        uint Psize = cs_size(Pmat);
        print_sizeof(Psize, "P");
#endif
        uint Fsize = cs_size(Fmat);
        print_sizeof(Fsize, "F");
        uint Rsize = cs_size(Rmat);
        print_sizeof(Rsize, "R");
        uint eyexsize = cs_size(eyex);
        print_sizeof(eyexsize, "eyex");
        uint Vpusize = Vpu.size()*16;
        print_sizeof(Vpusize, "Vpu");
#ifndef DIAGONAL_P
        print_sizeof(Psize+Fsize+Qsize+Rsize+eyexsize+Vpusize, "Total");
#endif
#ifdef DIAGONAL_P
        uint Uvmagsize = Uvmag.size()*8;
        print_sizeof(Uvmagsize, "Uvmag");
        uint Uvargsize = Uvarg.size()*8;
        print_sizeof(Uvargsize, "Uvarg");
        print_sizeof(Fsize+Rsize+eyexsize+Vpusize+Uvmagsize+Uvargsize, "Total");
#endif
#endif
        process_mem_usage(vm_used, res_used, gb_used);
        *selog << "End of estimate virtual memory: " << vm_used << ", timestep: " << timestamp-timeZero << std::flush;
//        *selog << "End of estimate resident memory: " << res_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
        *selog << "\n" << std::flush;

#ifdef TEST_SUITE
        testest_perf_fh.open(testpath+"est_perf.csv",std::ofstream::app);
        testest_perf_fh << "," << sec << "," << gb_used;
        testest_perf_fh.close();
#endif

#ifdef SBASE_TESTING
        // when needed for debugging, exit after first estimate call
        if (++estimateExitCount == 20)
            exit(0);
#endif
#endif

#ifdef TEST_SUITE
        testest_perf_fh.open(testpath+"est_perf.csv",std::ofstream::app);
        testest_perf_fh << "\n";
        testest_perf_fh.close();
#endif

#ifdef GADAL_INTERFACE
        print_est_err();
        }
#endif
        return true;
    }


    private:
    void decompress_state(cs *&xmat) {
        // copy state into vector (states, especially phase, can be 0)
        std::vector<double> xvec(xmat->m,0.0);
        for ( uint idx = 0 ; idx < xmat->nzmax ; idx++ )
            xvec[xmat->i[idx]] = xmat->x[idx];
        // update Vpu
        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            uint vidx = idx-1;
            uint Tidx = node_qty + vidx;
            double vrei = xvec[vidx] * cos(xvec[Tidx]);
            double vimi = xvec[vidx] * sin(xvec[Tidx]);
#ifdef SET_PRECISION12
            vrei = SET_PRECISION12(vrei);
            vimi = SET_PRECISION12(vimi);
#endif
            Vpu[idx] = complex<double>(vrei,vimi);
        }
        // update A
        for ( auto& map_pair : regid_primnode ) {
//            *selog << map_pair.first << std::endl;
            string regid = map_pair.first;
            string primnode = regid_primnode[regid];
            string regnode = regid_regnode[regid];

//            *selog << "primnode: " << primnode << 
//                "\tregnode: " << regnode << std::endl;

            // get i and j
            uint i = node_idxs[primnode];
            uint j = node_idxs[regnode];
            
            // get vi and vj
            double vi = abs(xvec[i-1]);
            double vj = abs(xvec[j-1]);

//            *selog << "vi: " << vi << "\tvj: " << vj << std::endl;

            // assign vj/vi to Amat[j][i]
            Amat[j][i] = vj/vi;

//            *selog << "Amat[j][i]: " << Amat[j][i] << std::endl;

        }
    }

#include <float.h>

#ifdef DIAGONAL_P
    private:
    void decompress_variance(cs *&Pmat) {
        // vector to store the state variance (diagonal of Pmat)
        std::vector<double> uvec(Pmat->n);
        // Pmat is in compressed-column form; iterate over columns
        for ( uint j = 0; j < Pmat->n ; j++ ) {
            // iterate over existing data in column j
            for ( uint p = Pmat->p[j] ; p < Pmat->p[j+1] ; p++ ) {
                // get the row index for existing data
                if ( Pmat->i[p] == j ) {
                    // if this is a diagonal entry, extract it
                    uvec[j] = Pmat->x[p];
                    break;
                }
            }
        }

#ifdef DEBUG_PRIMARY
        double minMag = DBL_MAX;
        double maxMag = DBL_MIN;
        double minArg = DBL_MAX;
        double maxArg = DBL_MIN;
#endif

        // update Umag and Uarg
        for ( auto& node_name: node_names ) {
            uint idx = node_idxs[node_name];
            double uvmag = uvec[idx-1];
            double uvarg = uvec[node_qty + idx-1];
#ifdef SET_PRECISION12
            uvmag = SET_PRECISION12(uvmag);
            uvarg = SET_PRECISION12(uvarg);
#endif
            Uvmag[idx] = uvmag;
            Uvarg[idx] = uvarg;

#ifdef DEBUG_PRIMARY
            minMag = fmin(minMag, fabs(Uvmag[idx]));
            maxMag = fmax(maxMag, fabs(Uvmag[idx]));
            minArg = fmin(minArg, fabs(Uvarg[idx]));
            maxArg = fmax(maxArg, fabs(Uvarg[idx]));
#endif
        }
#ifdef DEBUG_PRIMARY
        *selog << "Uvmag min: " << minMag << ", max: " << maxMag << "\n" << std::flush;
        *selog << "Uvarg min: " << minArg << ", max: " << maxArg << "\n" << std::flush;
#endif
    }
#endif


    private:
    void prep_x(cs *&xmat) {
        // Prepare x
#ifdef GS_OPTIMIZE
        xmat = gs_spalloc_firstcol(xqty);
        if (!xmat) *selog << "\tERROR: null x\n" << std::flush;
#else
        cs* xraw = cs_spalloc(xqty, 1, xqty, 1, 1);
#endif
        for ( auto& node_name : node_names ) {
            // Find the per-unit voltage of active node
            uint idx = node_idxs[node_name];
            complex<double> Vi = Vpu[idx];
#ifdef GS_OPTIMIZE
            // Add the voltage magnitude to x
            gs_entry_firstcol_negl(xmat,idx-1,abs(Vi));
            // Add the voltage angle to x
            gs_entry_firstcol_negl(xmat,node_qty + idx-1,arg(Vi));
#else
            // Add the voltage magnitude to x
            cs_entry_negl(xraw,idx-1,0,abs(Vi));
            // Add the voltage angle to x
            cs_entry_negl(xraw,node_qty + idx-1,0,arg(Vi));
#endif
        }
#ifndef GS_OPTIMIZE
        xmat = cs_compress(xraw);
        cs_spfree(xraw);
#endif
    }


#ifdef DIAGONAL_P
    private:
    void prep_P(cs *&Pmat) {
        // Prepare P as a diagonal matrix from state uncertanty
#ifdef GS_OPTIMIZE
        Pmat = gs_spalloc_diagonal(xqty);
        if (!Pmat) *selog << "\tERROR: null Pmat in prep_P\n" << std::flush;
#else
        cs* Pmatraw = cs_spalloc(xqty, xqty, xqty, 1, 1);
#endif
        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
#ifdef GS_OPTIMIZE
            // insert the voltage magnitude variance
            gs_entry_diagonal_negl(Pmat,idx-1,Uvmag[idx]);
            // insert the voltage phase variance
            gs_entry_diagonal_negl(Pmat,node_qty + idx-1,Uvarg[idx]);
#else
            // insert the voltage magnitude variance
            cs_entry_negl(Pmatraw,idx-1,idx-1,Uvmag[idx]);
            // insert the voltage phase variance
            cs_entry_negl(Pmatraw,node_qty + idx-1,node_qty + idx-1,Uvarg[idx]);
#endif
        }
#ifndef GS_OPTIMIZE
        Pmat = cs_compress(Pmatraw);
        cs_spfree(Pmatraw);
#endif
    }
#endif


    private:
    void prep_Q(cs *&Qmat, const uint& timestamp,
                           const uint& timestampLastEstimate) {
        // update process covariance matrix

        // Set scale factor to determine weighting of previous system state
        // based on elapsed time since previous state estimate,
        // i.e., large models will have more time between estimates
        // due to the size of the matrices involved while smaller models
        // will keep up with real-time GridLAB-D or sensor service output
        // of 3 seconds between measurements

        // Modeling change in system state as a Wiener process; therefore
        // covariance is proportional to the time between estimates.
        // Within 1 standard deviation, change is 0.01 p.u. in a
        // 1 second interval as a high-end estimate for likely voltage change
        // so 0.01^2 is the variance for that voltage change estimate

        double factor;
        // for first estimate calls, either from start or after a restart
        // triggered by an exception, hardwire the uncertainty to be large
        // afterwards, set it to a smaller value based on time between estimates

        // smaller, variable factor value results in slower convergence,
        // heavier damped initial estimates after closing switches
        // i.e., "rounded corners" in estimate plots
        //factor = 0.1;
        if ( timestampLastEstimate == UINT_MAX )
            factor = 0.1;
        else
            factor = 0.0001 * (timestamp - timestampLastEstimate);
#ifdef DEBUG_PRIMARY
        *selog << "Q uncertainty scale factor: " << factor << "\n" << std::flush;
#endif

        // Previously, the scale factor was hardwired for all models
        // factor = 0.03;

#ifdef GS_OPTIMIZE
        Qmat = gs_doubleval_diagonal(node_qty, factor, factor*M_PI);
#else
        cs *Qraw = cs_spalloc (2*node_qty, 2*node_qty, 2*node_qty, 1, 1);
        for (uint i = 0; i < node_qty; i++) {
            cs_entry_negl(Qraw, i, i, factor);
            cs_entry_negl(Qraw, node_qty+i, node_qty+i, factor*M_PI);
        }
        Qmat = cs_compress(Qraw);
        cs_spfree(Qraw);
#endif
    }


    private:
    void prep_R(cs *&Rmat, const uint& timestamp) {
#ifndef GS_OPTIMIZE
        cs* Rraw = cs_spalloc(zqty, zqty, zqty, 1, 1);
#endif

        for ( auto& zid : Zary.zids ) {
            // check whether to apply some form of time-based uncertainty by
            // checking a measurement type related flag
            if (!Zary.zpseudos[zid]) {
                // We are currently using the time since last measurement
                // for a node to determine a time-based uncertainty value.
                // The time since last measurement approach was chosen because
                // measurement uncertainty is expected to be dominated by the
                // most recent existing measurement.

                // This approach does not consider how many measurements did
                // or didn't come through prior to the last measurement since
                // the last estimate call, which could be determined using znew.
                // An approach that would accumulate uncertainty for both
                // missing and existing measurements could be considered.

                // consider no measurements yet for node so uncertainty
                // isn't based on time since last measurement
                if (Zary.ztimes[zid] == UINT_MAX) {
                    double uncertainty;

                    if (Zary.ztypes[zid] == "vi")
                        // standard deviation of expected initial voltage
                        // magnitude
                        uncertainty = 0.2*0.2;
                    else if (Zary.ztypes[zid] == "Ti")
                        // standard deviation of expected initial voltage
                        // angle
                        uncertainty = 0.01*M_PI*M_PI;
                    else if (Zary.ztypes[zid] == "aji")
                        // standard deviation of expected initial tap ratio
                        uncertainty = 0.2*0.2;
                    else if (Zary.ztypes[zid]=="Pi" || Zary.ztypes[zid]=="Qi")
                        // standard deviation of expected initial complex power
                        uncertainty = Zary.znomvals[zid]*Zary.znomvals[zid]*0.25;
                    else {
                        *selog << "\tERROR: prep_R unrecognized measurement type: " <<
                                  Zary.ztypes[zid] << "\n" << std::flush;
                        exit(1);
                    }

                    // variance of R[i,i] is sigma[i]^2 + uncertainty
#ifdef GS_OPTIMIZE
                    gs_entry_diagonal_negl(Rmat,Zary.zidxs[zid],
                                 Zary.zsigs[zid]*Zary.zsigs[zid] + uncertainty);
#else
                    cs_entry_negl(Rraw,Zary.zidxs[zid],Zary.zidxs[zid],
                                 Zary.zsigs[zid]*Zary.zsigs[zid] + uncertainty);
#endif

                // consider when there is an existing measurement so uncertainty
                // is based on time since last measurement
                } else if (timestamp - Zary.ztimes[zid] > 0) {
                    double factor;

                    if (Zary.ztypes[zid] == "vi")
                        // standard deviation of expected change in voltage
                        // magnitude per second
                        factor = 0.0001;
                    else if (Zary.ztypes[zid] == "Ti")
                        // standard deviation of expected change in voltage
                        // angle per second
                        factor = 0.0001*M_PI*M_PI;
                    else if (Zary.ztypes[zid] == "aji")
                        // standard deviation of expected change in tap ratio
                        // per second
                        factor = 0.00625*0.00625;
                    else if (Zary.ztypes[zid]=="Pi" || Zary.ztypes[zid]=="Qi")
                        // standard deviation of expected change in complex
                        // power per second
                        factor = Zary.znomvals[zid]*Zary.znomvals[zid]*0.0001;
                    else {
                        *selog << "\tERROR: prep_R unrecognized measurement type: " <<
                                  Zary.ztypes[zid] << "\n" << std::flush;
                        exit(1);
                    }

                    double timeUncertainty = factor*(timestamp - Zary.ztimes[zid]);
#ifdef DEBUG_PRIMARY
                    if (timeUncertainty > 0.0)
                        *selog << "Rmat non-zero timeUncertainty value, zid: " << zid << ", timestamp: " << timestamp << ", ztimes: " << Zary.ztimes[zid] << ", factor: " << factor << ", timeUncertainty: " << timeUncertainty << "\n" << std::flush;
#endif
                    // variance of R[i,i] is sigma[i]^2 + timeUncertainty
#ifdef GS_OPTIMIZE
                    gs_entry_diagonal_negl(Rmat,Zary.zidxs[zid],
                             Zary.zsigs[zid]*Zary.zsigs[zid] + timeUncertainty);
#else
                    cs_entry_negl(Rraw,Zary.zidxs[zid],Zary.zidxs[zid],
                             Zary.zsigs[zid]*Zary.zsigs[zid] + timeUncertainty);
#endif
                } else {
                // variance of R[i,i] is sigma[i]^2
#ifdef GS_OPTIMIZE
                    gs_entry_diagonal_negl(Rmat,Zary.zidxs[zid],
                                           Zary.zsigs[zid]*Zary.zsigs[zid]);
#else
                    cs_entry_negl(Rraw,Zary.zidxs[zid],Zary.zidxs[zid],
                                  Zary.zsigs[zid]*Zary.zsigs[zid]);
#endif
                }
            }
#ifndef GS_OPTIMIZE
            // for psuedo-measurements when not using the GS optimized
            // C-sparse functions, Rraw is created from scratch to need
            // to re-initialize those values every prep_R call
            else
                cs_entry_negl(Rraw,Zary.zidxs[zid],Zary.zidxs[zid],
                              Zary.zsigs[zid]*Zary.zsigs[zid]);
#endif
        }

#ifndef GS_OPTIMIZE
        Rmat = cs_compress(Rraw);
        cs_spfree(Rraw);
#endif
    }


#ifdef DEBUG_FILES
    private:
    void print_zvals(const string& filename) {
        std::ofstream ofh;
        ofh << std::setprecision(8);
        ofh.open(filename,std::ofstream::out);

        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            double zval = Zary.zvals[zid];
            string ztype = Zary.ztypes[zid];

            // these sbase checks are for when the comparison is to sbase 1e6
            if (sbase > 1.0e+11) {
                if (ztype=="Qi" || ztype=="Pi")
                    zval *= 1.0e+6;
            }
            else if (sbase > 1.0e+9) {
                if (ztype=="Qi" || ztype=="Pi")
                    zval *= 1.0e+4;
            }

            ofh << zidx << ", " << zid << ", " << ztype << ", " << zval << "\n";
        }
        ofh.close();
    }

    private:
    void print_cs_colvec(const string& filename, cs* colvec) {
        std::ofstream ofh;
        ofh << std::setprecision(8);
        ofh.open(filename,std::ofstream::out);

        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            string ztype = Zary.ztypes[zid];
            double val = colvec->x[colvec->i[zidx]];
            if (filename[0]=='k' && (ztype=="Qi" || ztype=="Pi")) {
                //val /= sbase;
            }
            else if (ztype=="Qi" || ztype=="Pi") {
                val *= sbase;
            }
            ofh << zidx << ", " << zid << ", " << ztype << ", " << val << "\n";
        }
        ofh.close();
    }
#endif


    private:
    void sample_z(cs *&zmat) {
        // measurements have been loaded from the sim output message to Zary
#ifdef GS_OPTIMIZE
        zmat = gs_spalloc_firstcol(zqty);
        if (!zmat) *selog << "\tERROR: null z\n" << std::flush;
#else
        cs* zraw = cs_spalloc(zqty, 1, zqty, 1, 1);
#endif
        for ( auto& zid : Zary.zids ) {
            if (std::isnan(Zary.zvals[zid])) {
                *selog << "\tERROR: sample_z zid: " << zid << ", zidx: " << Zary.zidxs[zid] << ", zvals: " << Zary.zvals[zid] << "\n" << std::flush;
            } else {
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(zmat,Zary.zidxs[zid],Zary.zvals[zid]);
                //*selog << "sample_z firstcol matrix entry for zid: " << zid << ", Zary.zidxs: " << Zary.zidxs[zid] << ", zvals value: " << Zary.zvals[zid] << "\n" << std::flush;
#else
                cs_entry_negl(zraw,Zary.zidxs[zid],0,Zary.zvals[zid]);
#endif
            }
        }
#ifndef GS_OPTIMIZE
        zmat = cs_compress(zraw);
        cs_spfree(zraw);
#endif

#ifdef DEBUG_FILES
        //print_zvals("zvals_sbase1e6_prec8.csv");
#endif
    }


    private:
    void set_ni(const uint& i, double& vi, double& g, double& b) {
        // from node i to the reference node
        vi = abs(Vpu[i]);

        complex<double> Yi0 = 0;
        try {
            auto& Yrow = Ypu.at(i);
            for ( auto& yij_pair : Yrow )
                Yi0 += yij_pair.second;
        } catch ( const std::out_of_range& oor ) {}

        g = real(Yi0);
        b = imag(Yi0);
    }


    void set_nij(const uint& i, const uint& j, double& vi, double& vj,
                 double& T, double& ai, double& aj, double& bij, 
                 double& g, double& b) {
        vi = abs(Vpu[i]);
        vj = abs(Vpu[j]);
        T = arg(Vpu[i]) - arg(Vpu[j]);

        // make sure not to reduce sparcity of Y; if Y exists, we can try A
        complex<double> Yij = complex<double>(0,0);
        ai = 1;
        aj = 1;
        bij = 1;
        try {
            auto& Yrow = Ypu.at(i);
            try {
                Yij = Yrow.at(j);

                // We know the nodes are coupled; check for Aij
                // NOTE: A is never iterated over - we don't need at()
                try {
                    auto Arow = Amat.at(i);
                    try {
                        ai = Arow.at(j);
//                        *selog << "|||||||||||||||||| ai assigned to: " << ai << std::endl;
                    } catch ( const std::out_of_range& oor ) {}
                } catch ( const std::out_of_range& oor ) {}

                // We know the nodes are coupled; check for Aji
                // NOTE: A is never iterated over - we don't need at()
                try {
                    auto Arow = Amat.at(j);
                    try {
                        aj = Arow.at(i);
//                        *selog << "|||||||||||||||||| aj assigned to: " << aj << std::endl;
                    } catch ( const std::out_of_range& oor ) {}
                } catch ( const std::out_of_range& oor ) {}

                // We know the nodes are coupled; check for Bij
                // Bij = Bji by definition
                try {
                    auto Brow = Bmat.at(i);
                    try {
                        bij = Brow.at(j);
//                        *selog << "|||||||||||||||||| bij assigned to: " << bij << ", for i: " << i << ", j: " << j << std::endl;
                        // GARY HACK HACK HACK
#if 0
                        if (bij < 1) {
                            *selog << "|||||||||||||||||| bij assigned to: " << bij << ", for i: " << i << ", j: " << j << ", zeroing Yij HACK" << std::endl;
                            Yij = complex<double>(0,0);
                        }
#endif
                    } catch ( const std::out_of_range& oor ) {}
                } catch ( const std::out_of_range& oor ) {}
            } catch ( const std::out_of_range& oor ) {
                *selog << "\tERROR: set_nij catch on Ypu[i].at(j) lookup\n" << std::flush;
                exit(1);
            }
        } catch ( const std::out_of_range& oor ) {
            *selog << "\tERROR: set_nij catch on Ypu.at(i) lookup\n" << std::flush;
            exit(1);
        }

        g = real(-Yij);
        b = imag(-Yij);
    }


    private:
    void calc_h(cs *&h) {
        // each z component has a measurement function component
#ifdef GS_OPTIMIZE
        h = gs_spalloc_firstcol(zqty);
        if (!h) *selog << "\tERROR: null h\n" << std::flush;
#else
        cs* hraw = cs_spalloc(zqty, 1, zqty, 1, 1);
#endif

#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            string ztype = Zary.ztypes[zid];
            // Determine the type of z component
            if ( !ztype.compare("Pi") ) {
                // Real power injection into node i
                uint i = node_idxs[Zary.znode1s[zid]];
                double Pi = 0;
                try {
                    auto& Yrow = Ypu.at(i);
                    for ( auto& rowpair : Yrow ) {
                        uint j = rowpair.first;
                        if (j != i) {
                            double vi, vj, T, ai, aj, bij, g, b;
                            set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                            // Add the real power component flowing from i to j
                            double term = (vi*vi/(ai*ai) * g) -
                                   (vi*vj/(ai*aj) * (g*cos(T) + b*sin(T)));
#ifdef SWITCHES
                            // bij--switch status multiplier between i and j
                            Pi += bij*term;
#else
                            Pi += term;
#endif
                        }
                    }
                    // Add the real power component flowing from i to 0
                    double vi, g, b;
                    set_ni(i, vi, g, b);
                    Pi += vi*vi * g;
                    if (std::isnan(Pi))
                        *selog << "\tERROR: calc_h zid: " << zid << ", zidx: " << zidx << ", ztype: " << ztype << ", g: " << g << ", b: " << b << "\n" << std::flush;
                } catch ( const std::out_of_range& oor ) {}
                if (!std::isnan(Pi)) {
                    // Insert the measurement component
#ifdef GS_OPTIMIZE
                    gs_entry_firstcol_negl(h,zidx,Pi);
#else
                    cs_entry_negl(hraw,zidx,0,Pi);
#endif
                }
            }
            else if ( !ztype.compare("Qi") ) {
                // Reactive power injection into node i
                uint i = node_idxs[Zary.znode1s[zid]];
                double Qi = 0;
                try {
                    auto& Yrow = Ypu.at(i);
                    for ( auto& rowpair : Yrow ) {
                        uint j = rowpair.first;
                        if (j != i) {
                            double vi, vj, T, ai, aj, bij, g, b;
                            set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                            // Add the reactive power component flowing from i to j
                            double term = ((vi*vi/(ai*ai)) * (-b)) -
                                       (vi*vj/(ai*aj) * (g*sin(T) - b*cos(T)));
#ifdef SWITCHES
                            // bij--switch status multiplier between i and j
                            Qi += bij*term;
#else
                            Qi += term;
#endif
                        }
                    }
                    // Add the reactive power component flowing from i to 0
                    double vi, g, b;
                    set_ni(i, vi, g, b);
                    Qi += - vi*vi * b;
                    if (std::isnan(Qi))
                        *selog << "\tERROR: calc_h zid: " << zid << ", zidx: " << zidx << ", ztype: " << ztype << ", g: " << g << ", b: " << b << "\n" << std::flush;
                } catch ( const std::out_of_range& oor ) {}
                if (!std::isnan(Qi)) {
#ifdef GS_OPTIMIZE
                    gs_entry_firstcol_negl(h,zidx,Qi);
#else
                    cs_entry_negl(hraw,zidx,0,Qi);
#endif
                }
            }
            else if ( !ztype.compare("aji" ) ) {
                // aji = vj/vi
                uint i = node_idxs[Zary.znode1s[zid]];
                uint j = node_idxs[Zary.znode2s[zid]];
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                double aji = vj/vi;
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,aji);
#else
                cs_entry_negl(hraw,zidx,0,aji);
#endif
            }
            else if ( !ztype.compare("vi") ) {
                // vi is a direct state measurement
                uint i = node_idxs[Zary.znode1s[zid]];
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,abs(Vpu[i]));
                //*selog << "calc_h vi firstcol matrix entry for zid: " << zid << ", zidx: " << zidx << ", Vpu value: " << abs(Vpu[i]) << "\n" << std::flush;
#else
                cs_entry_negl(hraw,zidx,0,abs(Vpu[i]));
#endif
            }
            else if ( !ztype.compare("Ti") ) {
                // Ti is a direct state measurement
                uint i = node_idxs[Zary.znode1s[zid]];
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,arg(Vpu[i]));
                //*selog << "calc_h Ti firstcol matrix entry for zid: " << zid << ", zidx: " << zidx << ", Vpu value: " << abs(Vpu[i]) << "\n" << std::flush;
#else
                cs_entry_negl(hraw,zidx,0,arg(Vpu[i]));
#endif
            }
            else if ( !ztype.compare("switch_ij") ) {
                // if switch is closed, vi = vj

                // if switch is open, possibility of distributed generation
                // means the relationship between vi and vj is indeterminant
                // this piecewise relationship is not differentiable for calc_J
            }
            else { 
                *selog << "\tERROR: Undefined measurement type " + ztype + "\n" << std::flush;
            }
        }
#ifndef GS_OPTIMIZE
        h = cs_compress(hraw);
        cs_spfree(hraw);
#endif
#ifdef DEBUG_PRIMARY
        *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
    }


    private:
    void calc_J(cs *&J) {
#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
        // each z component has a Jacobian component for each state
#ifdef GS_OPTIMIZE
        J = gs_spalloc_colorder(zqty,xqty,Jshapemap.size());
        if (!J) *selog << "\tERROR: null J\n" << std::flush;
#else
        cs* Jraw = cs_spalloc(zqty, xqty, Jshapemap.size(), 1, 1);
#endif
        // loop over existing Jacobian entries
        for (std::pair<uint, std::array<uint, 5>> Jelem : Jshapemap) {
            // Unpack entry data
            uint zidx = Jelem.second[0];
            uint xidx = Jelem.second[1];
            uint i = Jelem.second[2];
            uint j = Jelem.second[3];
            uint entry_type = Jelem.second[4];

            if ( entry_type == dPi_dvi ) {
                // --- compute dPi/dvi
                double dP = 0;
                // loop over adjacent nodes
                auto& Yrow = Ypu.at(i);
                for ( auto& rowpair : Yrow ) {
                    // intentionally redeclares j
                    uint j = rowpair.first;
                    if (j != i) {
                        double vi, vj, T, ai, aj, bij, g, b;
                        set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                        double term = (2*vi/(ai*ai) * g) -
                                       (vj/(ai*aj) * (g*cos(T) + b*sin(T)));
#ifdef SWITCHES
                        // bij--switch status multiplier between i and j
                        dP += bij*term;
#else
                        dP += term;
#endif
                    }
                }
                // consider the reference node
                double vi, g, b;
                set_ni(i, vi, g, b);
                dP += 2*vi * g;
                if (std::isnan(dP))
                    *selog << "\tERROR: calc_J dPi/dvi zidx: " << zidx << ", xidx: " << xidx << ", g: " << g << ", b: " << b << "\n" << std::flush;
                else {
#ifdef GS_OPTIMIZE
                    gs_entry_colorder_negl(J,zidx,xidx,dP);
#else
                    cs_entry_negl(Jraw,zidx,xidx,dP);
#endif
                }
            }

            else
            if ( entry_type == dPi_dvj ) {
                // --- compute dPi/dvj
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
#ifdef SWITCHES
                // bij--switch status multiplier between i and j
                double dP = bij * (-vi/(ai*aj)) * (g*cos(T) + b*sin(T));
#else
                double dP = (-vi/(ai*aj)) * (g*cos(T) + b*sin(T));
#endif
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dP);
#else
                cs_entry_negl(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dQi_dvi ) {
                // --- compute dQi/dvi
                double dQ = 0;
                // loop over adjacent nodes
                auto& Yrow = Ypu.at(i);
                for ( auto& rowpair : Yrow ) {
                    // intentionally redeclares j
                    uint j = rowpair.first;
                    if (j != i ) {
                        double vi, vj, T, ai, aj, bij, g, b;
                        set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                        double term = (2*vi/(ai*ai) * (-b)) -
                                       (vj/(ai*aj) * (g*sin(T) - b*cos(T)));
#ifdef SWITCHES
                        // bij--switch status multiplier between i and j
                        dQ += bij*term;
#else
                        dQ += term;
#endif
                    }
                }
                // consider the reference node
                double vi, g, b;
                set_ni(i, vi, g, b);
                dQ += -2*vi*b;
                if (std::isnan(dQ))
                    *selog << "\tERROR: calc_J dQi/dvi zidx: " << zidx << ", xidx: " << xidx << ", g: " << g << ", b: " << b << "\n" << std::flush;
                else {
#ifdef GS_OPTIMIZE
                    gs_entry_colorder_negl(J,zidx,xidx,dQ);
#else
                    cs_entry_negl(Jraw,zidx,xidx,dQ);
#endif
                }
            }

            else
            if ( entry_type == dQi_dvj ) {
                // --- compute dQi/dvj
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
#ifdef SWITCHES
                // bij--switch status multiplier between i and j
                double dQ = bij * (-vi/(ai*aj)) * (g*sin(T) - b*cos(T));
#else
                double dQ = (-vi/(ai*aj)) * (g*sin(T) - b*cos(T));
#endif
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dQ);
#else
                cs_entry_negl(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dvi_dvi ) {
                 // --- compute dvi/dvi
#ifdef GS_OPTIMIZE
                 gs_entry_colorder_negl(J,zidx,xidx,1.0);
#else
                 cs_entry_negl(Jraw,zidx,xidx,1.0);
#endif
            }
            
            else
            if ( entry_type == dPi_dTi ) {
                // --- compute dPi/dTi
                double dP = 0;
                // loop over adjacent nodes
                auto &Yrow = Ypu.at(i);
                for ( auto& rowpair : Yrow ) {
                    // intentionally redeclares j
                    uint j = rowpair.first;
                    if (j != i) {
                        double vi, vj, T, ai, aj, bij, g, b;
                        set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                        double term = (vi*vj/(ai*aj)) * (g*sin(T) - b*cos(T));
#ifdef SWITCHES
                        // bij--switch status multiplier between i and j
                        dP += bij*term;
#else
                        dP += term;
#endif
                    }
                }
                // reference node component is 0
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dP);
#else
                cs_entry_negl(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dPi_dTj ) {
                // --- compute dPi/dTj
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
#ifdef SWITCHES
                // bij--switch status multiplier between i and j
                double dP = bij * (-vi*vj/(ai*aj)) * (g*sin(T) - b*cos(T));
#else
                double dP = (-vi*vj/(ai*aj)) * (g*sin(T) - b*cos(T));
#endif
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dP);
#else
                cs_entry_negl(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dQi_dTi ) {
                // compute dQi/dTi
                double dQ = 0;
                // loop over adjacent nodes
                auto& Yrow = Ypu.at(i);
                for ( auto& rowpair : Yrow ) {
                    // intentionally redeclares j
                    uint j = rowpair.first;
                    if (j != i) {
                        double vi, vj, T, ai, aj, bij, g, b;
                        set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                        double term = (-vi*vj/(ai*aj)) * (g*cos(T) + b*sin(T));
#ifdef SWITCHES
                        // bij--switch status multiplier between i and j
                        dQ += bij*term;
#else
                        dQ += term;
#endif
                    }
                }
                // reference component is 0
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dQ);
#else
                cs_entry_negl(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dQi_dTj ) {
                // --- compute dQi/dTj
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
#ifdef SWITCHES
                // bij--switch status multiplier between i and j
                double dQ = bij * (vi*vj/(ai*aj)) * (g*cos(T) + b*sin(T));
#else
                double dQ = (vi*vj/(ai*aj)) * (g*cos(T) + b*sin(T));
#endif
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dQ);
#else
                cs_entry_negl(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dTi_dTi ) {
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,1.0);
#else
                cs_entry_negl(Jraw,zidx,xidx,1.0);
#endif
            }

            else
            if ( entry_type == daji_dvj) {
                // daji/dvj = 1/vi
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                double daji = 1.0/vi;
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J, zidx, xidx, daji);
#else
                cs_entry_negl(Jraw, zidx, xidx, daji);
#endif
            }

            else
            if ( entry_type == daji_dvi) {
                // daji/dvi -vj/vi^2
                double vi, vj, T, ai, aj, bij, g, b;
                set_nij(i, j, vi, vj, T, ai, aj, bij, g, b);
                double daji = -vj/(vi*vi);
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J, zidx, xidx, daji);
#else
                cs_entry_negl(Jraw, zidx, xidx, daji);
#endif
            }

            else {
                *selog << "\tERROR: Undefined jacobian element type type " +
                    entry_type << "\n" << std::flush;
            }

            // loop over regulator tap ratio states
            // --- LATER ---
            // -------------

        }
#ifndef GS_OPTIMIZE
        J = cs_compress(Jraw);
        cs_spfree(Jraw);
#endif
#ifdef DEBUG_PRIMARY
        double endTime = getWallTime();
        *selog << 
            getMinSec(endTime-startTime) << "\n" << std::flush;
#endif
    }
    

#ifdef GS_OPTIMIZE
    private:
    cs *gs_singleval_diagonal(uint mn, double single) {
        cs *A = cs_spalloc (mn, mn, mn, 1, 0);

        for (uint j = 0; j < mn; j++) {
            A->p[j] = A->i[j] = j;
            A->x[j] = single;
        }
        A->p[mn] = mn;

        return (A);
    }

    private:
    cs *gs_doubleval_diagonal(uint halfmn, double upper, double lower) {
        cs *A = cs_spalloc (2*halfmn, 2*halfmn, 2*halfmn, 1, 0);

        uint j;
        for (j = 0; j < halfmn; j++) {
            A->p[j] = A->i[j] = j;
            A->x[j] = upper;
        }
        for (j = halfmn; j < 2*halfmn; j++) {
            A->p[j] = A->i[j] = j;
            A->x[j] = lower;
        }
        A->p[2*halfmn] = 2*halfmn;

        return (A);
    }

    private:
    cs *gs_spalloc_diagonal(uint mn) {
        cs *A = cs_spalloc (mn, mn, mn, 1, 0);
        A->p[mn] = mn;
        return (A);
    }

    private:
    void gs_entry_diagonal(cs *A, uint ij, double value) {
        A->p[ij] = A->i[ij] = ij;
        A->x[ij] = value;
    }

    private:
    cs *gs_spalloc_firstcol(uint m, uint n=1) {
        cs *A = cs_spalloc (m, n, m, 1, 0);
        A->nzmax = 0;
        A->p[0] = 0;
        for (uint jj=1; jj<=n; jj++) A->p[jj] = n;
        return (A);
    }

    private:
    void gs_entry_firstcol(cs *A, uint ii, double value) {
        A->i[A->nzmax] = ii;
        A->x[A->nzmax] = value;
        A->nzmax++;
        A->p[1] = A->nzmax;
    }

    private:
    cs *gs_spalloc_fullsquare(uint mn) {
        cs *A = (cs *)cs_calloc (1, sizeof (cs)) ;
        if (!A) return (NULL) ;
        A->m = A->n = mn ;
        A->nzmax = mn*mn ;
        A->nz = -1 ;
        A->p = (int *)cs_malloc (mn+1, sizeof (int)) ;
        A->i = (int *)cs_malloc (A->nzmax, sizeof (int)) ;
        // make sure A->x starts zero'd out vs. regular cs_spalloc
        A->x = (double*)cs_calloc (A->nzmax, sizeof (double)) ;

        A->p[mn] = A->nzmax;
        for (uint jj=0; jj<mn; jj++) {
            A->p[jj] = jj*mn;
            for (uint ii=0; ii<mn; ii++)
                A->i[jj*mn + ii] = ii;
        }
        return (A);
    }

    private:
    void gs_entry_fullsquare(cs *A, uint ii, uint jj, double value) {
        A->x[jj*A->n + ii] = value;
    }

    private:
    int gsidx;

    private:
    cs *gs_spalloc_colorder(uint m, uint n, uint nzmax) {
        cs *A = (cs*)cs_calloc (1, sizeof (cs)) ;
        if (!A) return (NULL) ;
        A->m = m; A->n = n ;
        A->nzmax = nzmax ;
        A->nz = -1 ;
        // make sure A->p starts zero'd out vs. regular cs_spalloc since
        // zero values are used to determine if A->p has been set
        A->p = (int*)cs_calloc (n+1, sizeof (int)) ;
        A->i = (int*)cs_malloc (nzmax, sizeof (int)) ;
        A->x = (double*)cs_malloc (nzmax, sizeof (double)) ;

        gsidx = 0;
        return (A);
    }

    private:
    void gs_entry_colorder(cs *A, uint ii, uint jj, double value) {
        if (jj>0 && A->p[jj]==0)
            A->p[jj] = gsidx;
        A->i[gsidx] = ii;
        A->x[gsidx++] = value;
        for (uint idx=jj+1; idx <= A->n; idx++)
            A->p[idx] = gsidx;
    }
#endif


#ifdef DEBUG_PRIMARY
    void print_cs_summary(cs *&a, const string &matname, const bool &initFlag=false) {
        *selog << matname << " is " << a->m << " by " << a->n << " with " << a->nzmax << " entries\n" << std::flush;

#ifdef TEST_SUITE
        string testpath = "output/" + plint->getOutputDir() + "/test_suite/";
        if (initFlag) {
          testinit_accy_fh.open(testpath+"init_accy.csv",std::ofstream::app);
          testinit_accy_fh << "," << a->m << "," << a->n << "," << a->nzmax;
          testinit_accy_fh.close();
        } else {
          testest_accy_fh.open(testpath+"est_accy.csv",std::ofstream::app);
          testest_accy_fh << "," << a->m << "," << a->n << "," << a->nzmax;
          testest_accy_fh.close();
        }
#endif
    }
#endif


#ifdef DEBUG_STATS
    void print_cs_stats(cs *&a, const string &matname, const bool &initFlag=false) {
        double minVal = DBL_MAX;
        double maxVal = DBL_MIN;
        double sumVal = 0.0;

        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                minVal = fmin(minVal, a->x[j]);
                maxVal = fmax(maxVal, a->x[j]);
                sumVal += a->x[j];
            }
        }

        double mean = sumVal/a->nzmax;

        *selog << matname << " min: " << minVal << ", max: " << maxVal << ", sum: " << sumVal << ", mean: " << mean << "\n" << std::flush;

#ifdef TEST_SUITE
        string testpath = "output/" + plint->getOutputDir() + "/test_suite/";
        if (initFlag) {
          testinit_accy_fh.open(testpath+"init_accy.csv",std::ofstream::app);
          testinit_accy_fh << "," << minVal << "," << maxVal << "," << mean;
          testinit_accy_fh.close();
        } else {
          testest_accy_fh.open(testpath+"est_accy.csv",std::ofstream::app);
          testest_accy_fh << "," << minVal << "," << maxVal << "," << mean;
          testest_accy_fh.close();
        }
#endif
    }
#endif


#ifdef DEBUG_FILES
    private:
    void print_cs_full(cs *&a, const string &filename="cs.csv",
                           const uint &precision=16) {
        *selog << "starting print_cs_full\n" << std::flush;
        // write to file
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n" << std::flush;
        for ( uint i = 0 ; i < a->m ; i++ ) {
            uint ioff = i*a->n;
            for ( uint j = 0 ; j < a->n-1 ; j++ )
                ofh << a->x[ioff + j] << ",";
            ofh << a->x[ioff + a->n-1] << "\n";
        }
        ofh.close();
        *selog << "done print_cs_full\n" << std::flush;
    }

    private:
    void print_cs_compress_triples(cs *&a, const string &filename="cs.csv",
                                   const uint &precision=16) {
        // First copy into a map
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
#if 000
        std::unordered_map<uint,bool> powerMap;
        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            string ztype = Zary.ztypes[zid];
            powerMap[zidx] = (ztype=="Qi" || ztype=="Pi");
        }
#endif
        // write to file
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        for ( uint j = 0 ; j < a->n ; j++ ) {
            if (j % 1000 == 0)
              *selog << "  writing j = " << j << "\n" << std::flush;
            for ( uint i = 0 ; i < a->m ; i++ ) {
                double val = mat[i][j];
#if 000
                string prefix = "";
                if ( powerMap[j] ) {
                    prefix = " Pi/Qi column ";
                    //val *= sbase; // R, Supd
                    val /= sbase; // Kupd
                }
                if ( powerMap[i] ) {
                    prefix += " Pi/Qi row ";
                    //val *= sbase; // R, Supd, Y, z, h, J
                }
#else
                if (val != 0.0)
                    ofh << i << ", " << j << ", " << val << "\n";
#endif
            }
        }
        ofh.close();
    }

    private:
    void print_cs_compress_sparse(cs *&a, const string &filename="cs.csv",
                                  const uint &precision=16) {
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                ofh << a->i[j] << ", " << i << ", " << a->x[j] << "\n";
            }
        }
        ofh.close();
    }

    private:
    void stdout_cs_compress(cs *&a, const uint &precision=16) {
        // First copy into a map
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
        // write to stdout
        *selog << "printing CS compressed matrix:\n" << std::flush;
        for ( uint i = 0 ; i < a->m ; i++ )
            for ( uint j = 0 ; j < a->n ; j++ )
                *selog << mat[i][j] << ( j == a->n-1 ? "\n" : "," );
        *selog << "done printing CS compressed matrix\n" << std::flush;
    }

    private:
    void print_cs_compress(cs *&a, const string &filename="cs.csv",
                           const uint &precision=16) {
        // First copy into a map
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
        // write to file
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        for ( uint i = 0 ; i < a->m ; i++ )
            for ( uint j = 0 ; j < a->n ; j++ )
                ofh << mat[i][j] << ( j == a->n-1 ? "\n" : "," );
        ofh.close();
    }

#if 000
    private:
    void print_cs_compress_triples(cs *&a, const string &filename="cs.csv",
                                   const uint &precision=16) {
        // First copy into a map
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }

        std::unordered_map<uint,bool> powerMap;
        for ( auto& zid : Zary.zids ) {
            uint zidx = Zary.zidxs[zid];
            string ztype = Zary.ztypes[zid];
            powerMap[zidx] = (ztype=="Qi" || ztype=="Pi");
        }

        // write to file
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        for ( uint j = 0 ; j < a->n ; j++ )
            for ( uint i = 0 ; i < a->m ; i++ ) {
                double val = mat[i][j];
                string prefix = "";
#if 000
                if ( powerMap[j] ) {
                    prefix = " Pi/Qi column ";
                    //val *= sbase; // R, Supd
                    val /= sbase; // Kupd
                }
                if ( powerMap[i] ) {
                    prefix += " Pi/Qi row ";
                    //val *= sbase; // R, Supd, Y, z, h, J
                }
#endif
                if (val != 0.0)
                    ofh << prefix << i << ", " << j << ", " << val << "\n";
            }
        ofh.close();
    }

    private:
    void print_cs_compress_sparse(cs *&a, const string &filename="cs.csv",
                                  const uint &precision=16) {
        std::ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,std::ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        std::unordered_map<uint,std::unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                ofh << a->i[j] << ", " << i << ", " << a->x[j] << "\n";
            }
        }
        ofh.close();
    }
#endif
#endif

#ifdef DEBUG_PRIMARY
    private:
    double getWallTime() {
        struct timeval time;

        if (gettimeofday(&time, NULL))
            return 0;

        return (double)time.tv_sec + (double)time.tv_usec*0.000001;
    }

    private:
    string getMinSec(double seconds) {
        uint useconds = (uint)seconds;
        uint sec = useconds % 60;
        string secstr = (sec<10)? "0"+std::to_string(sec): std::to_string(sec);
        return (std::to_string(useconds/60) + ":" + secstr + std::to_string(seconds - useconds).substr(1));
    }
#endif

#ifdef DEBUG_SIZES
    private:
    uint cs_size(cs *sparse) {
        // this works for compressed matrices.  For uncompressed (raw), use
        //return 16*sparse->nzmax + 28;
        return 12*sparse->nzmax + 4*(sparse->n+1) + 28;
    }

    private:
    void print_sizeof(uint size, string msg) {
        if (size > 1024) {
            double kbsize = size/1024.0;
            if (kbsize > 1024.0) {
                double mbsize = kbsize/1024.0;
                if (mbsize > 1024.0) {
                    double gbsize = mbsize/1024.0;
                    *selog << msg << " size: " << gbsize << " GB\n" << std::flush;
                } else {
                    *selog << msg << " size: " << mbsize << " MB\n" << std::flush;
                }
            } else {
                *selog << msg << " size: " << kbsize << " KB\n" << std::flush;
            }
        } else {
            *selog << msg << " size: " << size << " bytes\n" << std::flush;
        }
    }
#endif

#ifdef DEBUG_PRIMARY
#include <ios>
    private:
    void process_mem_usage(string& vm_used, string& res_used, double& gb_used) {
        double vm_usage     = 0.0;
        double resident_set = 0.0;

        // 'file' stat seems to give the most reliable results
        std::ifstream stat_stream("/proc/self/stat",std::ios_base::in);

        // dummy vars for leading entries in stat that we don't care about
        string pid, comm, state, ppid, pgrp, session, tty_nr;
        string tpgid, flags, minflt, cminflt, majflt, cmajflt;
        string utime, stime, cutime, cstime, priority, nice;
        string O, itrealvalue, starttime;

        // the two fields we want
        unsigned long vsize;
        long rss;

        stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                    >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                    >> utime >> stime >> cutime >> cstime >> priority >> nice
                    >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
        stat_stream.close();

        long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
        vm_usage     = vsize / 1024.0;
        resident_set = rss * page_size_kb;

        gb_used = vm_usage/(1024.0*1024.0);

        string units = " KB";
        if (vm_usage > 1024.0) {
            vm_usage = vm_usage/1024.0;
            units = " MB";
            if (vm_usage > 1024.0) {
                vm_usage = vm_usage/1024.0;
                units = " GB";
            }
        }
        vm_used = std::to_string(vm_usage) + units;

        units = " KB";
        if (resident_set > 1024.0) {
            resident_set = resident_set/1024.0;
            units = " MB";
            if (resident_set > 1024.0) {
                resident_set = resident_set/1024.0;
                units = " GB";
            }
        }
        res_used = std::to_string(resident_set) + units;
    }
#endif

#ifdef GADAL_INTERFACE
    private:
    void print_est_err() {
        SDMAP est_vmagpu;

        double measSumMag = 0.0;
        double diffSumMag = 0.0;
        uint numSum = 0;

        SDMAP meas_vmagpu;
        for ( auto& zid : Zary.zids )
            if ( !Zary.ztypes[zid].compare("vi") )
                meas_vmagpu[Zary.znode1s[zid]] = Zary.zvals[zid];

        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            est_vmagpu[node_name] = abs( Vpu[idx] );

            if (meas_vmagpu.find(node_name) != meas_vmagpu.end()) {
                measSumMag += meas_vmagpu[node_name];
                diffSumMag += abs(est_vmagpu[node_name] - meas_vmagpu[node_name]);
            }
        }

        double perErrMag = 100.0 * diffSumMag/measSumMag;

        *selog << "% err: " << perErrMag << "\n" << std::flush;
    }
#endif
};
#endif
