#ifndef SELOOPWORKER_HPP
#define SELOOPWORKER_HPP

#include "state_estimator_gridappsd.hpp"
using state_estimator_gridappsd::gridappsd_session;

#include "json.hpp"
using json = nlohmann::json;

#include "cs.h"
#include "klu.h"

// standard data types
#include <complex>
#include <list>
#include <unordered_map>
#include <array>

// for mkdir and opendir
#ifdef DEBUG_FILES
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#endif

#ifdef DEBUG_PRIMARY
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#endif

// SLIST holds the lists of node names and regulator names
#ifndef SLIST
#define SLIST std::list<std::string>
#endif

// SIMAP holds the one-indexed positions of nodes
#ifndef SIMAP
#define SIMAP std::unordered_map<std::string,unsigned int>
#endif

// SDMAP holds x, z, and the set of regulator taps
//#ifndef SDMAP
//#define SDMAP std::unordered_map<std::string,double>
//#endif

// SCMAP holds the complex node voltages
#ifndef SCMAP
#define SCMAP std::unordered_map<std::string,std::complex<double>>
#endif

// SSMAP holds the mapping between sensors and nodes
#ifndef SSMAP
#define SSMAP std::unordered_map<std::string,std::string>
#endif

// ICMAP holds the voltage state
#ifndef ICMAP
#define ICMAP std::unordered_map<unsigned int,std::complex<double>>
#endif

// IDMAP holds the state variances
#ifndef IDMAP
#define IDMAP std::unordered_map<unsigned int,double>
#endif

// ISMAP holds the node name lookup table
#ifndef ISMAP
#define ISMAP std::unordered_map<unsigned int,std::string>
#endif

#ifndef IMDMAP
#define IMDMAP std::unordered_map<unsigned int,IDMAP>
#endif

#ifndef A5MAP
#define A5MAP std::multimap<unsigned int,std::array<unsigned int, 5>>
#define A5PAIR std::pair<unsigned int, std::array<unsigned int, 5>>
#endif


// This class listens for system state messages
class SELoopWorker {
    private:

    // passed in from constructor
    SharedQueue<json>* workQueue;
    gridappsd_session* gad;
    SensorArray        zary;
    uint               node_qty;     // number of nodes
    SLIST              node_names;   // node names [list of strings]
    SIMAP              node_idxs;    // node positional indices [node->int]
    SCMAP              node_vnoms;   // complex nominal voltages of nodes
    SSMAP              node_bmrids;  // string mRIDs of the associated buses
    SSMAP              node_phs;     // string phases
    ISMAP              node_name_lookup;
    double             sbase;
    IMMAP              Yphys;        // Ybus [node->[row->col]] [physical units]
    IMDMAP             Amat;         // regulator tap ratios
    SSMAP              regid_primnode_map;
    SSMAP              regid_regnode_map;
    SSMAP              mmrid_pos_type_map; // type of position measurement

    SEProducer *statePublisher = 0;
    json jstate;       // object holding the output message

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

    cs *Fmat;           // process model
    cs *Rmat;           // measurement covariance (diagonal)
    cs *eyex;           // identity matrix of dimension x

    bool firstEstimateFlag = true;
#ifdef SBASE_TESTING
    uint estimateExitCount = 0;
#endif

#ifdef DEBUG_FILES
    ofstream state_fh;  // file to record states
#endif


    public:
    SELoopWorker(SharedQueue<json>* workQueue,
            gridappsd_session* gad,
            const SensorArray& zary,
            const uint& node_qty,
            const SLIST& node_names,
            const SIMAP& node_idxs,
            const SCMAP& node_vnoms,
            const SSMAP& node_bmrids,
            const SSMAP& node_phs,
            const ISMAP& node_name_lookup,
            const double& sbase,
            const IMMAP& Yphys,
            const IMDMAP& Amat,
            const SSMAP& regid_primnode_map,
            const SSMAP& regid_regnode_map,
            const SSMAP& mmrid_pos_type_map) {
        this->workQueue = workQueue;
        this->gad = gad;
        this->zary = zary;
        this->node_qty = node_qty;
        this->node_names = node_names;
        this->node_idxs = node_idxs;
        this->node_vnoms = node_vnoms;
        this->node_bmrids = node_bmrids;
        this->node_phs = node_phs;
        this->node_name_lookup = node_name_lookup;
        this->sbase = sbase;
        this->Yphys = Yphys; 
        this->Amat = Amat;
        this->regid_primnode_map = regid_primnode_map;
        this->regid_regnode_map = regid_regnode_map;
        this->mmrid_pos_type_map = mmrid_pos_type_map;
    }


    public:
    void workLoop() {
        json jmessage;
        uint timestamp, timestampLastEstimate, timeZero;
        bool exitAfterEstimateFlag = false;
        bool doEstimateFlag;

        // do one-time-only processing
        init();

        // initialize what's updated during processing
        // (if things go bad we'll need to reset these)
        initVoltagesAndCovariance();

        for (;;) {
            // ----------------------------------------------------------------
            // Reset the new measurement counter for each node
            // ----------------------------------------------------------------
            doEstimateFlag = false;
            for ( auto& zid : zary.zids ) zary.znew[zid] = 0;

            // drain the queue with quick z-averaging
            do {
                jmessage = workQueue->pop();

                if (jmessage.find("message") != jmessage.end()) {
                    timestamp = jmessage["message"]["timestamp"];
#ifdef DEBUG_PRIMARY
                    if (firstEstimateFlag) {
                        timeZero = timestamp;
                        timestampLastEstimate = timestamp;
                        firstEstimateFlag = false;
                    }
                    *selog << "Draining workQueue size: " << workQueue->size() << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
#endif
                    // do z summation here
                    add_zvals(jmessage);

                    // set flag to indicate a full estimate can be done
                    // if a COMPLETED/CLOSED log message is received
                    doEstimateFlag = true;

                } else if (jmessage.find("processStatus") != jmessage.end()) {
                    // only COMPLETE/CLOSED log messages are put on the queue
                    // so process for that case only
                    if (doEstimateFlag) {
#ifdef DEBUG_PRIMARY
                        *selog << "Got COMPLETE/CLOSED log message on queue, doing full estimate with previous measurement\n" << std::flush;
#endif
                        // set flag to exit after completing full estimate below
                        exitAfterEstimateFlag = true;

                        // we've already done the add_zvals call for the last
                        // measurement so proceed to estimate z-averaging + estimate
                        break;
                    } else {
#ifdef DEBUG_PRIMARY
                        *selog << "Got COMPLETE/CLOSED log message on queue, normal exit because full estimate just done\n" << std::flush;
#endif
                        exit(0);
                    }
                }
            } while (!workQueue->empty()); // uncomment this to drain queue
            //} while (false); // uncomment this to fully process all messages

            // do z averaging here by dividing sum by # of items
// #ifdef DEBUG_PRIMARY
//             *selog << "===========> z-averaging being done after draining queue\n" << std::flush;
// #endif

            for ( auto& zid : zary.zids ) {
                if ( zary.znew[zid] > 1 )
                    zary.zvals[zid] /= zary.znew[zid];
            }

// #ifdef DEBUG_PRIMARY
//             *selog << "zvals before estimate\n" << std::flush;
//             for ( auto& zid : zary.zids ) {
//                 *selog << "measurement of type: " << zary.ztypes[zid] << "\t" << zid << ": " << zary.zvals[zid] << "\t(" << zary.znew[zid] << ")\n" << std::flush;
//             }
// #endif

            // do the core "estimate" processing here since the queue is,
            // for the moment, empty
            // ----------------------------------------------------------------
            // Estimate the state
            // ----------------------------------------------------------------
#ifdef DEBUG_PRIMARY
            *selog << "\nEstimating state for timestep: " << timestamp-timeZero << "\n" << std::flush;
#endif
            try {
                estimate(timestamp, timestampLastEstimate, timeZero);
                timestampLastEstimate = timestamp;
                //sleep(30); // delay to let queue refill for testing
                publish(timestamp);

            } catch(const char* msg) {
                *selog << "\nCaught klu_error exception--resetting state estimation\n" << std::flush;

                // reset Amat to where it started
                // iterate over map of maps setting all entries back to 1
                for (auto& ent1 : Amat)
                    for (auto& ent2 : ent1.second)
                        ent2.second = 1;

                // things went bad so reset what was previously updated
                initVoltagesAndCovariance();

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

#ifdef DEBUG_FILES
        *selog << "writing zary ztypes,znodes1s values\n\n" << std::flush;
        ofstream ofh;
        ofh.open("zary.csv",ofstream::out);
        for ( auto& zid: zary.zids ) {
            ofh << zary.ztypes[zid] << ", " << zary.znode1s[zid] << "\n";
        }
        ofh.close();
#endif

        // set up the output message json object
        jstate["simulation_id"] = gad->simid;

#ifdef DEBUG_FILES
        // create the output directory if needed
        if (!opendir("output")) {
            // if output is already a symbolic link to a shared
            // folder as intended, this won't be needed
            mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }

        // create simulation parent directory
        std::string simpath = "output/sim_" + gad->simid + "/";
        mkdir(simpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        // create init directory
        std::string initpath = simpath + "init/";
        mkdir(initpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        // Construct the producer that will be used to publish the state estimate
        string sePubTopic = "goss.gridappsd.state-estimator.out."+gad->simid;
        statePublisher = new SEProducer(gad->brokerURI,gad->username,gad->password,sePubTopic,"topic");

        // --------------------------------------------------------------------
        // Establish Dimension of State Space and Measurement Space
        // --------------------------------------------------------------------
        xqty = 2*node_qty;
        zqty = zary.zqty;

#ifdef DEBUG_PRIMARY
        *selog << "xqty is " << xqty << "; " << std::flush;
        *selog << "zqty is " << zqty << "\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Determine possible non-zero Jacobian elements
        // --------------------------------------------------------------------
        for ( auto& zid: zary.zids ) {
            uint zidx = zary.zidxs[zid];            // row index of J
            string ztype = zary.ztypes[zid];        // measurement type
            uint i = node_idxs[zary.znode1s[zid]];  // one-indexed

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
            }

            else
            if ( !ztype.compare("Qi") ) {
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
            }

            else
            if ( !ztype.compare("aji") ) {
                // note: the regulation node, j, is assigned to znode2s
                //       the primary node, i, is assigned to znode1s
                uint j = node_idxs[zary.znode2s[zid]];

                // daji/dvj exists: 1/viV
                Jshapemap.insert(A5PAIR(j-1, {zidx, j-1, i, j, daji_dvj}));

                // daja/dvi exists: -vj/vi^2
                Jshapemap.insert(A5PAIR(i-1, {zidx, i-1, i, j, daji_dvi}));

            }

            else
            if
            ( !ztype.compare("vi") ) {
                // dvi/dvi is the only partial that exists
                Jshapemap.insert(A5PAIR(i-1, {zidx,i-1,i,i,dvi_dvi}));
            }

            else
            if ( !ztype.compare("Ti") ) {
                // dTi/dTi is the only partial that exists
                Jshapemap.insert(A5PAIR(node_qty+i-1, {zidx,node_qty+i-1,i,i,dTi_dTi}));
            }

            else { 
                *selog << "ERROR: Unrecognized measurement type " << 
                    ztype << std::flush;
            }
        }
#ifdef DEBUG_PRIMARY
        *selog << "Jshapemap Jacobian elements: " << Jshapemap.size() << "\n" << std::flush;
#endif

        // Cap Yphys with magnitude greater than 1e3 threshold
        double thresh = 1e+3;
#ifdef DEBUG_PRIMARY
        uint ctr = 0;
        *selog << "Yphys scaling started...\n" << std::flush;
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
                *selog << "Yphys scaling down row: " << i << ", col: " << j << "\n" << std::flush;
#endif
                double scaler = thresh / term_mag;
                // update the term
                complex<double> new_term_val = term_val * scaler;
#ifdef DEBUG_PRIMARY
                *selog << "\tYphys scaler: " << scaler << "\n" << std::flush;
                *selog << "\tYphys term_val: " << abs(term_val) << "(" << 180/PI*arg(term_val) << ")\n" << std::flush;
                *selog << "\tYphys new_term_val: " << abs(new_term_val) << "(" << 180/PI*arg(new_term_val) << ")\n" << std::flush;
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
        *selog << "Yphys # of scaled terms: " << ctr << "\n\n" << std::flush;
#endif

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
#ifdef DEBUG_FILES
        // write to file
        //ofstream ofh;
        ofh.open(initpath+"Ypu.csv",ofstream::out);
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
        ofh.open(initpath+"Yphys.csv",ofstream::out);
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
        //ofstream ofh;
        ofh.open(initpath+"vnoms.csv",ofstream::out);
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
        ofh.open(initpath+"nodem.csv",ofstream::out);
        *selog << "writing " << initpath+"nodem.csv\n" << std::flush;
        for ( auto& node_name : node_names )
            ofh << node_name << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor information to file
        ofh.open(initpath+"meas.txt",ofstream::out);
        *selog << "writing output/init/meas.txt\n" << std::flush;
        ofh << "sensor_type\tsensor_name\tnode1\tnode2\tvalue\tsigma\n";
        for ( auto& zid : zary.zids ) {
            ofh << zary.ztypes[zid] << "\t"
                << zid << "\t"
                << zary.znode1s[zid] << "\t"
                << zary.znode2s[zid] << "\t"
                << zary.zvals[zid] << "\t"
                << zary.zsigs[zid] << "\n";
        } ofh.close();

        *selog << "done writing\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        // write sensor types to file
        ofh.open(initpath+"ztypes.csv");
        for ( auto& zid : zary.zids )
            ofh << zary.ztypes[zid] << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor node 1s
        ofh.open(initpath+"znode1s.csv");
        for ( auto& zid : zary.zids )
            ofh << zary.znode1s[zid] << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor node 2s
        ofh.open(initpath+"znode2s.csv");
        for ( auto& zid : zary.zids )
            ofh << zary.znode2s[zid] << "\n";
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
        *selog << "F is " << Fmat->m << " by " << Fmat->n << " with " << Fmat->nzmax << " entries\n" << std::flush;
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
        *selog << "eyex is " << eyex->m << " by " << eyex->n << " with " << eyex->nzmax << " entries\n" << std::flush;
#endif

        // R is the measurement covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        *selog << "Initializing R -- " << std::flush;
#endif
#ifdef GS_OPTIMIZE
        Rmat = gs_spalloc_diagonal(zqty);
        for ( auto& zid : zary.zids )
            // variance of R[i,i] is sigma[i]^2
            // Originally we had this as just sigma[i], which resulted in
            // different sbase producing different estimate results and took
            // a couple weeks to debug working through many matrices to figure
            // out if they were different for different sbase values
            gs_entry_diagonal_negl(Rmat,zary.zidxs[zid],
                                   zary.zsigs[zid]*zary.zsigs[zid]);
#else
        cs* Rraw = cs_spalloc(zqty, zqty, zqty, 1, 1);
        for ( auto& zid : zary.zids )
            // variance of R[i,i] is sigma[i]^2
            // Originally we had this as just sigma[i], which resulted in
            // different sbase producing different estimate results and took
            // a couple weeks to debug working through many matrices to figure
            // out if they were different for different sbase values
            cs_entry_negl(Rraw,zary.zidxs[zid],zary.zidxs[zid],
                          zary.zsigs[zid]*zary.zsigs[zid]);
        Rmat = cs_compress(Rraw);
        cs_spfree(Rraw);
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Rmat,initpath+"R.csv");
        //print_cs_compress_triples(Rmat,"R_sbase1e6_trip.csv", 8);
#endif
#ifdef DEBUG_PRIMARY
        *selog << "R is " << Rmat->m << " by " << Rmat->n << " with " << Rmat->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        // print initial state vector
        ofh.open(initpath+"Vpu.csv",ofstream::out);
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

#ifdef DEBUG_FILES
        // --------------------------------------------------------------------
        // Initialize the state recorder file
        // --------------------------------------------------------------------
        state_fh.open(simpath+"vmag_per-unit.csv",ofstream::out);
        state_fh << "timestamp,";
        uint ctr = 0;
        for ( auto& node_name : node_names )
            state_fh << "\'"+node_name+"\'" << ( ++ctr < node_qty ? "," : "\n" );
        state_fh.close();
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
        *selog << "Voltages Initialized.\n\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Initialize State Covariance Matrix
        // --------------------------------------------------------------------
        double span_vmag = 1.0;
        double span_varg = 1.0/3.0*PI;
        double span_taps = 0.2;

        // TODO: finalize scaling for state uncertainty initialization
        double span_multiplier = 0.02;
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
    void add_zvals(const json& jmessage) {
        // --------------------------------------------------------------------
        // Use the simulation output to update the states
        // --------------------------------------------------------------------
        // This needs to translate simulation output into zary.zvals[zid]
        //  - We need to iterate over the "measurements" in the simoutput
        //  - As in SensorDefConsumer.hpp, measurements can have multiple z's

        // Next, update new measurements
        for ( auto& m : jmessage["message"]["measurements"] ) {
            // link back to information about the measurement using its mRID
            string mmrid = m["measurement_mrid"];
            string m_type = zary.mtypes[mmrid];

            // Check for "PNV" measurement
            if ( !m_type.compare("PNV") ) {

                // update the voltage magnitude (in per-unit)
                string zid = mmrid+"_Vmag";
                double vmag_phys = m["magnitude"];
                // TODO: This uses vnom filled from OpenDSS values, but needs
                // to use GridLAB-D values
                if (zary.znew[zid] == 0)
                    zary.zvals[zid] = 
                        vmag_phys / abs(node_vnoms[zary.znode1s[zid]]);
                else
                    zary.zvals[zid] +=
                        vmag_phys / abs(node_vnoms[zary.znode1s[zid]]);
                zary.znew[zid]++;

                // update the voltage phase
                // --- LATER ---
                // -------------
            }
           
            // Check for "Pos" measurement
            if ( !m_type.compare("Pos") ) {
                if ( !mmrid_pos_type_map[mmrid].compare("regulator_tap") ) {
                    // update the tap ratio
                    string zid = mmrid+"_tap";
                    double tap_position = m["value"];
                    double tap_ratio = 1.0 + 0.1*tap_position/16.0;
//                    *selog << "tap_position: " << tap_position 
//                        << "\ttap_ratio: " << tap_ratio << std::endl;

                    if (zary.znew[zid] == 0)
                        zary.zvals[zid] = tap_ratio;
                    else
                        zary.zvals[zid] += tap_ratio;
                    zary.znew[zid]++;
                    

                    // Update the A matrix with the latest tap ratio measurement
                    // TODO: Consider averaging for queued measturements
                    string primnode = zary.znode1s[zid];
                    uint i = node_idxs[primnode];

                    string regnode = zary.znode2s[zid];
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
                }
            }

            //else if ( !m_type.compare("") ) {
            //}
        }
    }


    private:
    void publish(const uint& timestamp) {
        // --------------------------------------------------------------------
        // Package and publish the state
        // --------------------------------------------------------------------
        
        // Initializae json
        json jstate;
        jstate["simulation_id"] = gad->simid;
        jstate["message"] = json::object();
        jstate["message"]["timestamp"] = timestamp;
        jstate["message"]["Estimate"] = json::object();
        jstate["message"]["Estimate"]["timeStamp"] = timestamp;
        jstate["message"]["Estimate"]["SvEstVoltages"] = json::array();
//      jstate["message"]["measurements"] = json::array();

        for ( auto& node_name : node_names ) {
            // build a json object for each node
            // NOTE: I will not spend too much time on this until we know
            //  exactly what the message should look like
            //  - best guess is node and phase as commented out below
            //  - this would require a new sparql query and data flows
            json state;
            state["ConnectivityNode"] = node_bmrids[node_name];
            state["phase"] = node_phs[node_name];

            // add the state values
            uint idx = node_idxs[node_name];
            complex<double> vnom = node_vnoms[node_name];
            state["v"] = abs ( vnom * Vpu[idx] );

            double degrees = 180.0/PI * arg ( vnom * Vpu[idx] );
            // -165 <= degrees <= 195
            while (degrees > 195.0) degrees -= 360.0;
            while (degrees < -165.0) degrees += 360.0;
            state["angle"] = degrees;

            // TODO: Add v and angle variance values
            state["vVariance"] = 0;         // This comes from P
            state["angleVariance"] = 0;     // This comes from P
            // append this state to the measurement array
            jstate["message"]["Estimate"]["SvEstVoltages"].push_back(state);
        }

//      for ( auto& reg_name : reg_names ) {}

        // Publish the message
        statePublisher->send(jstate.dump());

#ifdef DEBUG_FILES
        // write to file
        std::string simpath = "output/sim_" + gad->simid + "/";
        state_fh.open(simpath+"vmag_per-unit.csv",ofstream::app);
        state_fh << timestamp << ',';
        uint ctr = 0;
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            state_fh << vmag_pu << ( ++ctr < node_qty ? "," : "\n" );
        }
        state_fh.close();
#endif
    }


    private:
    void estimate(const uint& timestamp, const uint& timestampLastEstimate,
                  const uint& timeZero) {
#ifdef DEBUG_PRIMARY
        double estimateStartTime = getWallTime();
#endif

        // TODO: WE NEED TO HANDLE R-MASK IN HERE SOMEWHERE

#ifdef DEBUG_FILES
        // set filename path based on timestamp
        std::string simpath = "output/sim_" + gad->simid + "/ts_";
        std::ostringstream out;
        out << simpath << timestamp << "/";
        std::string tspath = out.str();

        // create timestamp directory
        mkdir(tspath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

#ifdef DEBUG_FILES
        ofstream ofh;
        ofh.open(tspath+"Vpu.csv",ofstream::out);
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
        cs *Pmat; this->prep_P(Pmat);
#endif
#ifdef DEBUG_PRIMARY
        *selog << "P is " << Pmat->m << " by " << Pmat->n <<
            " with " << Pmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pmat,tspath+"P.csv");
#endif

        cs *P1 = cs_transpose(Fmat,1);
        if (!P1) *selog << "ERROR: null P1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P1 is " << P1->m << " by " << P1->n <<
            " with " << P1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P1,tspath+"P1.csv");
#endif

        cs *P2 = cs_multiply(Pmat,P1); cs_spfree(Pmat); cs_spfree(P1);
        if (!P2) *selog << "ERROR: null P2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P2 is " << P2->m << " by " << P2->n <<
            " with " << P2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P2,tspath+"P2.csv");
#endif

        cs *P3 = cs_multiply(Fmat,P2); cs_spfree(P2);
        if (!P3) *selog << "ERROR: null P3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P3 is " << P3->m << " by " << P3->n <<
            " with " << P3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P3,tspath+"P3.csv");
#endif

       cs *Qmat; this->prep_Q(Qmat, timestamp-timestampLastEstimate);
#ifdef DEBUG_PRIMARY
        *selog << "Q is " << Qmat->m << " by " << Qmat->n << " with " << Qmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Qmat,tspath+"Q.csv");
#endif

        cs *Ppre = cs_add(P3,Qmat,1,1); cs_spfree(P3); cs_spfree(Qmat);
        if (!Ppre) *selog << "ERROR: null Ppre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "Ppre is " << Ppre->m << " by " << Ppre->n <<
            " with " << Ppre->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Ppre,tspath+"Ppre.csv");
#endif

        // --------------------------------------------------------------------
        // Update Step
        // --------------------------------------------------------------------

#ifdef DEBUG_PRIMARY
        *selog << "calc_J time -- " << std::flush;
#endif
        cs *Jmat; this->calc_J(Jmat);
#ifdef DEBUG_PRIMARY
        *selog << "J is " << Jmat->m << " by " << Jmat->n <<
            " with " << Jmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Jmat,tspath+"J.csv");
#endif

        // -- compute S = J*P_predict*J' + R

        cs *S1 = cs_transpose(Jmat,1);
        if (!S1) *selog << "ERROR: null S1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "S1 is " << S1->m << " by " << S1->n <<
            " with " << S1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S1,tspath+"S1.csv");
#endif

        cs *S2 = cs_multiply(Ppre,S1);
        if (!S2) *selog << "ERROR: null S2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "S2 is " << S2->m << " by " << S2->n <<
            " with " << S2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S2,tspath+"S2.csv");
#endif

        cs *S3 = cs_multiply(Jmat,S2); cs_spfree(S2);
        if (!S3) *selog << "ERROR: null S3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "S3 is " << S3->m << " by " << S3->n <<
            " with " << S3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S3,tspath+"S3.csv");
#endif

        cs *Supd = cs_add(Rmat,S3,1,1); cs_spfree(S3);
        if (!Supd) *selog << "ERROR: null Supd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "Supd is " << Supd->m << " by " << Supd->n <<
            " with " << Supd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Supd,tspath+"Supd.csv");
        //print_cs_compress_triples(Supd, "Supd_sbase1e6_trip.csv", 8);
#endif

        // -- compute K = P_predict*J'*S^-1

#ifdef DEBUG_PRIMARY
        double startTime;
        string vm_used, res_used;
#endif

#ifdef GS_OPTIMIZE
        cs *K3 = gs_spalloc_fullsquare(zqty);
        double *rhs = K3->x;
#else
#if 000
        double *rhs = (double *)calloc(zqty*zqty, sizeof(double));
#else
        cs *K3 = gs_spalloc_fullsquare(zqty);
        double *rhs = K3->x;
#endif
#endif

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
            klunum = klu_factor(Supd->p,Supd->i,Supd->x,klusym,&klucom);
            if (!klunum) {
#ifdef DEBUG_PRIMARY
                *selog << "Common->status is: " << klucom.status << "\n" << std::flush;
                if ( klucom.status == 1 ) *selog << "\tKLU_SINGULAR\n" << std::flush;
#endif
                throw "klu_factor failed";
            }

#ifdef DEBUG_PRIMARY
            *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            // KLU condition number estimation
            (void)klu_condest(Supd->p,Supd->x,klusym,klunum,&klucom);
            *selog << "klu_condest Supd condition number estimate: " << klucom.condest << "\n" << std::flush;
#endif

            // initialize an identity right-hand side
#if 000
            rhs = new double[zqty*zqty];
            for ( uint ii = 0 ; ii < zqty*zqty ; ii++ )
                rhs[ii] = ii/zqty == ii%zqty ? 1 : 0;
#else
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
            *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            process_mem_usage(vm_used, res_used);
            *selog << "klu_solve virtual memory: " << vm_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
//            *selog << "klu_solve resident memory: " << res_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
#endif

            // free klusym and klunum or memory leak results
            klu_free_symbolic(&klusym, &klucom);
            klu_free_numeric(&klunum, &klucom);
        } catch (const char *msg) {
            *selog << "KLU ERROR: " << msg << "\n" << std::flush;
            throw "klu_error";
        }
        cs_spfree(Supd);

#if 000
        if (estimateExitCount == 20) {
            estimateExitCount = 0;
            throw "klu_error";
        }
#endif

#if 000
#ifndef GS_OPTIMIZE
        cs *K3raw = cs_spalloc(zqty, zqty, zqty*zqty, 1, 1);

        for (uint j=0; j<zqty; j++)
            for (uint i=0; i<zqty; i++)
                cs_entry_negl(K3raw,i,j,rhs[j*zqty + i]);

        cs *K3 = cs_compress(K3raw);
        cs_spfree(K3raw);
        free(rhs);
#endif
#endif

#ifdef DEBUG_PRIMARY
        if ( K3 ) *selog << "K3 is " << K3->m << " by " << K3->n <<
                " with " << K3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K3,tspath+"K3.csv");
#endif

        // GDB S1 and K1 are both J transpose so use S1 here
        cs *K2 = cs_multiply(Ppre,S1); cs_spfree(S1);
        if (!K2) *selog << "ERROR: null K2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "K2 is " << K2->m << " by " <<
             K2->n << " with " << K2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K2,tspath+"K2.csv");
#endif

#ifdef DEBUG_PRIMARY
        *selog << "Kupd time (bottleneck) -- " << std::flush;
        startTime = getWallTime();
#endif
        cs *Kupd = cs_multiply(K2,K3); cs_spfree(K2); cs_spfree(K3);
        if ( !Kupd ) *selog << "ERROR: Kupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        if ( Kupd ) *selog << "Kupd is " << Kupd->m << " by " << Kupd->n <<
                " with " << Kupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Kupd,tspath+"Kupd.csv");
        //print_cs_compress_triples(Kupd, "Kupd_sbase1e12_trip.csv", 4);
#endif

        // -- compute y = z - h
        cs *zmat; this->sample_z(zmat);
#ifdef DEBUG_PRIMARY
        *selog << "z is " << zmat->m << " by " << zmat->n <<
            " with " << zmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(zmat,tspath+"z.csv");
#endif

#ifdef DEBUG_PRIMARY
        *selog << "calc_h time -- " << std::flush;
#endif
        cs *hmat; this->calc_h(hmat);
#ifdef DEBUG_PRIMARY
        *selog << "h is " << hmat->m << " by " << hmat->n <<
            " with " << hmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(hmat,tspath+"h.csv");
#endif

        cs *yupd = cs_add(zmat,hmat,1,-1); cs_spfree(zmat); cs_spfree(hmat);
        if (!yupd) *selog << "ERROR: null yupd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "yupd is " << yupd->m << " by " << yupd->n <<
            " with " << yupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(yupd,tspath+"yupd.csv");
        //print_cs_colvec("yupd_sbase1e6_prec8.csv", yupd);
#endif

        // -- compute x_update = x_predict + K * y

        cs *x1 = cs_multiply(Kupd,yupd); cs_spfree(yupd);
        if ( !x1 ) *selog << "ERROR: x1 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "x1 is " << x1->m << " by " << x1->n <<
                " with " << x1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(x1,tspath+"x1.csv");
#endif

        cs *xmat; this->prep_x(xmat);
#ifdef DEBUG_PRIMARY
        *selog << "x is " << xmat->m << " by " << xmat->n <<
            " with " << xmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xmat,tspath+"x.csv");
#endif

        cs *xpre = cs_multiply(Fmat,xmat); cs_spfree(xmat);
        if (!xpre) *selog << "ERROR: null xpre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "xpre is " << xpre->m << " by " << xpre->n <<
            " with " << xpre->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xpre,tspath+"xpre.csv");
#endif

        cs *xupd = cs_add(xpre,x1,1,1); cs_spfree(x1); cs_spfree(xpre);
        if ( !xupd ) *selog << "ERROR: xupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "xupd is " << xupd->m << " by " << xupd->n <<
                " with " << xupd->nzmax << " entries\n" << std::flush;
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
        if ( !P4 ) *selog << "ERROR: P4 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        *selog << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        if ( P4 ) *selog << "P4 is " << P4->m << " by " << P4->n <<
                " with " << P4->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P4,tspath+"P4.csv");
#endif

        cs *P5 = cs_add(eyex,P4,1,-1); cs_spfree(P4);
        if ( !P5 ) *selog << "ERROR: P5 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P5 is " << P5->m << " by " << P5->n <<
                " with " << P5->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P5,tspath+"P5.csv");
#endif

#ifdef DIAGONAL_P
        cs *Pupd = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !Pupd ) *selog << "ERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P updated is " << Pupd->m << " by " << Pupd->n <<
                " with " << Pupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pupd,tspath+"Pupd.csv");
#endif

#else
        // re-allocate P for the updated state
        Pmat = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !Pmat ) *selog << "ERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else *selog << "P updated is " << Pmat->m << " by " << Pmat->n <<
                " with " << Pmat->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P,tspath+"Pupd.csv");
#endif
#endif

        // --------------------------------------------------------------------
        // Update persistant state (Vpu and A)
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
        *selog << "\n*** Total estimate time: " << 
            getMinSec(getWallTime()-estimateStartTime) << ", timestep: " <<
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
        process_mem_usage(vm_used, res_used);
        *selog << "End of estimate virtual memory: " << vm_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
//        *selog << "End of estimate resident memory: " << res_used << ", timestep: " << timestamp-timeZero << "\n" << std::flush;
        *selog << "\n" << std::flush;
#ifdef SBASE_TESTING
        // when needed for debugging, exit after first estimate call
        if (++estimateExitCount == 20)
            exit(0);
#endif
#endif
    }


    private:
    void decompress_state(cs *&xmat) {
        // copy state into vector (states, especially phase, can be 0)
        vector<double> xvec(xmat->m,0.0);
        for ( uint idx = 0 ; idx < xmat->nzmax ; idx++ )
            xvec[xmat->i[idx]] = xmat->x[idx];
        // update Vpu
        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            uint vidx = idx-1;
            uint Tidx = node_qty + idx-1;
            double vrei = xvec[vidx] * cos(xvec[Tidx]);
            double vimi = xvec[vidx] * sin(xvec[Tidx]);
            Vpu[idx] = complex<double>(vrei,vimi);
        }
        // update A
        for ( auto& map_pair : regid_primnode_map ) {
//            *selog << map_pair.first << std::endl;
            string regid = map_pair.first;
            string primnode = regid_primnode_map[regid];
            string regnode = regid_regnode_map[regid];

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
        vector<double> uvec(Pmat->n);
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
            Uvmag[idx] = uvec[idx-1];
            Uvarg[idx] = uvec[node_qty + idx-1];

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
        if (!xmat) *selog << "ERROR: null x\n" << std::flush;
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
        if (!Pmat) *selog << "ERROR: null Pmat in prep_P\n" << std::flush;
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
    void prep_Q(cs *&Qmat, const uint& timeSinceLastEstimate) {
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
        double factor = 0.0001 * timeSinceLastEstimate;
        //double factor = 0.03;
        // Previously, the scale factor was hardwired to 0.03 for all models

#ifdef GS_OPTIMIZE
        Qmat = gs_doubleval_diagonal(node_qty, factor, factor*PI);
#else
        cs *Qraw = cs_spalloc (2*node_qty, 2*node_qty, 2*node_qty, 1, 1);
        for (uint i = 0; i < node_qty; i++) {
            cs_entry_negl(Qraw, i, i, factor);
            cs_entry_negl(Qraw, node_qty+i, node_qty+i, factor*PI);
        }
        Qmat = cs_compress(Qraw);
        cs_spfree(Qraw);
#endif
    }


#ifdef DEBUG_FILES
    private:
    void print_zvals(const string& filename) {
        ofstream ofh;
        ofh << std::setprecision(8);
        ofh.open(filename,ofstream::out);

        for ( auto& zid : zary.zids ) {
            uint zidx = zary.zidxs[zid];
            double zval = zary.zvals[zid];
            string ztype = zary.ztypes[zid];

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
        ofstream ofh;
        ofh << std::setprecision(8);
        ofh.open(filename,ofstream::out);

        for ( auto& zid : zary.zids ) {
            uint zidx = zary.zidxs[zid];
            string ztype = zary.ztypes[zid];
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
        // measurements have been loaded from the sim output message to zary
#ifdef GS_OPTIMIZE
        zmat = gs_spalloc_firstcol(zqty);
        if (!zmat) *selog << "ERROR: null z\n" << std::flush;
#else
        cs* zraw = cs_spalloc(zqty, 1, zqty, 1, 1);
#endif
        for ( auto& zid : zary.zids ) {
#ifdef GS_OPTIMIZE
            gs_entry_firstcol_negl(zmat,zary.zidxs[zid],zary.zvals[zid]);
#else
            cs_entry_negl(zraw,zary.zidxs[zid],0,zary.zvals[zid]);
#endif
        }
#ifndef GS_OPTIMIZE
        zmat = cs_compress(zraw);
        cs_spfree(zraw);
#endif

#ifdef DEBUG_FILES
        print_zvals("zvals_sbase1e6_prec8.csv");
#endif
    }


    private:
    double vi;
    double vj;
    double T;
    double g;
    double b;
    double ai;
    double aj;


    private:
    void set_n(uint i, uint j) {
        if ( !i ) {
            *selog << "ERROR: Unexpected call to set_n with i=0\n" << std::flush;
            return;
        }
        if ( !j ) {
            // from node i to the reference node
            vi = abs(Vpu[i]);
            vj = 0;
            T = arg(Vpu[i]);
            ai = 1;
            aj = 1;
            complex<double> Yi0;
            try {
                auto& Yrow = Ypu.at(i);
                for ( auto& yij_pair : Yrow )
                    Yi0 += yij_pair.second;
            } catch ( const std::out_of_range& oor ) {}
            g = real(Yi0);
            b = imag(Yi0);
        }
        else {
            vi = abs(Vpu[i]);
            vj = abs(Vpu[j]);
            T = arg(Vpu[i]) - arg(Vpu[j]);
            // make sure not to reduce sparcity of Y; if Y exists, we can try A
            complex<double> Yij;
            ai = 0;
            aj = 0;
            try {
                auto& Yrow = Ypu.at(i);
                try {
                    Yij = Yrow.at(j);
                    // We know the nodes are coupled; check for Aij
                    // NOTE: A is never iterated over - we don't need at()
                    ai = 1;
                    try {
                        auto Arow = Amat.at(i);
                        try {
                            ai = real(Arow.at(j));
//                            *selog << "|||||||||||||||||| ai assigned to: " << ai << std::endl;
                        } catch ( const std::out_of_range& oor ) {}
                    } catch ( const std::out_of_range& oor ) {}
                    // We know the nodes are coupled; check for Aji
                    // NOTE: A is never iterated over - we don't need at()
                    aj = 1;
                    try {
                        auto Arow = Amat.at(j);
                        try {
                            aj = real(Arow.at(i));
//                            *selog << "|||||||||||||||||| aj assigned to: " << aj << std::endl;
                        } catch ( const std::out_of_range& oor ) {}
                    } catch ( const std::out_of_range& oor ) {}
                } catch ( const std::out_of_range& oor ) {
                    *selog << "ERROR: set_n catch on Yrow.at(j) lookup\n" << std::flush;
                    exit(1);
                }
            } catch ( const std::out_of_range& oor ) {
                *selog << "ERROR: set_n catch on Ypu.at(i) lookup\n" << std::flush;
                exit(1);
            }
            g = real(-1.0*Yij);
            b = imag(-1.0*Yij);
        }
    }
    

    private:
    void calc_h(cs *&h) {
        // each z component has a measurement function component
#ifdef GS_OPTIMIZE
        h = gs_spalloc_firstcol(zqty);
        if (!h) *selog << "ERROR: null h\n" << std::flush;
#else
        cs* hraw = cs_spalloc(zqty, 1, zqty, 1, 1);
#endif

#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
        for ( auto& zid : zary.zids ) {
            uint zidx = zary.zidxs[zid];
            string ztype = zary.ztypes[zid];
            // Determine the type of z component
            if ( !ztype.compare("Pi") ) {
                // Real power injection into node i
                uint i = node_idxs[zary.znode1s[zid]];
                double Pi = 0;
                try {
                    auto& Yrow = Ypu.at(i);
                    for ( auto& rowpair : Yrow ) {
                        uint j = rowpair.first;
                        if (j != i) {
                            set_n(i,j);
                            // Add the real power component flowing from i to j
                            Pi = Pi + vi*vi/ai/ai * g - 
                                vi*vj/ai/aj * (g*cos(T) + b*sin(T));
                        }
                    }
                    // Add the real power component flowing from i to 0
                    set_n(i,0);
                    Pi += vi*vi * g; // times cos(T) ????
                } catch ( const std::out_of_range& oor ) {}
                // Insert the measurement component
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,Pi);
#else
                cs_entry_negl(hraw,zidx,0,Pi);
#endif
            }
            else if ( !zary.ztypes[zid].compare("Qi") ) {
                // Reactive power injection into node i
                uint i = node_idxs[zary.znode1s[zid]];
                double Qi = 0;
                try {
                    auto& Yrow = Ypu.at(i);
                    for ( auto& rowpair : Yrow ) {
                        uint j = rowpair.first;
                        if (j != i) {
                            set_n(i,j);
                            // Add the reactive power component flowing from i to j
                            Qi = Qi - vi*vi/ai/ai * b - 
                                vi*vj/ai/aj * (g*sin(T) - b*cos(T));
                        }
                    }
                    // Add the reactive power component flowing from i to 0
                    set_n(i,0);
                    Qi -= vi*vi * b;
                } catch ( const std::out_of_range& oor ) {}
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,Qi);
#else
                cs_entry_negl(hraw,zidx,0,Qi);
#endif
            }
            else if ( !zary.ztypes[zid].compare("aji" ) ) {
                // aji = vj/vi
                uint i = node_idxs[zary.znode1s[zid]];
                uint j = node_idxs[zary.znode2s[zid]];
                set_n(i,j);
                double aji = vj/vi;
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,aji);
#else
                cs_entry_negl(hraw,zidx,0,aji);
#endif
            }
            else if ( !zary.ztypes[zid].compare("vi") ) {
                // vi is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,abs(Vpu[i]));
#else
                cs_entry_negl(hraw,zidx,0,abs(Vpu[i]));
#endif
            }
            else if ( !zary.ztypes[zid].compare("Ti") ) {
                // Ti is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
#ifdef GS_OPTIMIZE
                gs_entry_firstcol_negl(h,zidx,arg(Vpu[i]));
#else
                cs_entry_negl(hraw,zidx,0,arg(Vpu[i]));
#endif
            }
            else { 
                *selog << "WARNING: Undefined measurement type " + ztype + "\n" << std::flush;
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
        if (!J) *selog << "ERROR: null J\n" << std::flush;
#else
        cs* Jraw = cs_spalloc(zqty, xqty, Jshapemap.size(), 1, 1);
#endif
        // loop over existing Jacobian entries
        for (std::pair<unsigned int, std::array<unsigned int, 5>> Jelem : Jshapemap) {
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
                    j = rowpair.first;
                    if (j != i) {
                        set_n(i,j);
                        dP = dP + 2*vi/ai/ai * g - 
                            vj/ai/aj * (g*cos(T) + b*sin(T));
                    }
                }
                // consider the reference node
                set_n(i,0);
                dP = dP + 2*vi * g;
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dP);
#else
                cs_entry_negl(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dPi_dvj ) {
                // --- compute dPi/dvj
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                double dP = -1.0 * vi/ai/aj * (g*cos(T) + b*sin(T));
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
                    uint j = rowpair.first;
                    if (j != i ) {
                        set_n(i,j);
                        dQ = dQ - 2*vi/ai/ai * b - 
                            vj/ai/aj * (g*sin(T) - b*cos(T));
                    }
                }
                // consider the reference node
                set_n(i,0);
                dQ = dQ - 2*vi*b;
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J,zidx,xidx,dQ);
#else
                cs_entry_negl(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dQi_dvj ) {
                // --- compute dQi/dvj
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                double dQ = -1.0 * vi/ai/aj * (g*sin(T) - b*cos(T));
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
                    uint j = rowpair.first;
                    if (j != i) {
                        set_n(i,j);
                        dP = dP + vi*vj/ai/aj * (g*sin(T) - b*cos(T));
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
                // --- compute dP/dTj
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);
 
                set_n(i,j);
                double dP = -1.0 * vi*vj/ai/aj * (g*sin(T) - b*cos(T));
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
                    uint j = rowpair.first;
                    if (j != i) {
                        set_n(i,j);
                        dQ = dQ - vi*vj/ai/aj * (g*cos(T) + b*sin(T));
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
                // --- compute dQ/dTj
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                double dQ = vi*vj/ai/aj * (g*cos(T) + b*sin(T));
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
                set_n(i,j);
                double daji = 1.0/vi;
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J, zidx, xidx, daji);
#else
                cs_entry_negl(Jraw, zidx, xidx, daji);
#endif
            }

            else
            if ( entry_type == daji_dvi) {
                // daja/dvi -vj/vi^2
                set_n(i,j);
                double daji = -vj/(vi*vi);
#ifdef GS_OPTIMIZE
                gs_entry_colorder_negl(J, zidx, xidx, daji);
#else
                cs_entry_negl(Jraw, zidx, xidx, daji);
#endif
            }

            else {
                *selog << "WARNING: Undefined jacobian element type type " + 
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
#endif

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

#ifdef GS_OPTIMIZE
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
        if (jj>0 && A->p[jj]==0) A->p[jj] = gsidx;
        A->i[gsidx] = ii;
        A->x[gsidx++] = value;
        if (jj+1 == A->n) A->p[jj+1] = gsidx;
    }
#endif

#ifdef DEBUG_FILES
    private:
    void print_cs_compress(cs *&a, const string &filename="cs.csv",
                           const uint &precision=16) {
        // First copy into a map
        unordered_map<uint,unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
        // write to file
        ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        for ( uint i = 0 ; i < a->m ; i++ )
            for ( uint j = 0 ; j < a->n ; j++ )
                ofh << mat[i][j] << ( j == a->n-1 ? "\n" : "," );
        ofh.close();
    }

    private:
    void print_cs_compress_triples(cs *&a, const string &filename="cs.csv",
                                   const uint &precision=16) {
        // First copy into a map
        unordered_map<uint,unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }

        unordered_map<uint,bool> powerMap;
        for ( auto& zid : zary.zids ) {
            uint zidx = zary.zidxs[zid];
            string ztype = zary.ztypes[zid];
            powerMap[zidx] = (ztype=="Qi" || ztype=="Pi");
        }

        // write to file
        ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,ofstream::out);
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
        ofstream ofh;
        ofh << std::setprecision(precision);
        ofh.open(filename,ofstream::out);
        *selog << "writing " + filename + "\n\n" << std::flush;
        unordered_map<uint,unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                ofh << a->i[j] << ", " << i << ", " << a->x[j] << "\n";
            }
        }
        ofh.close();
    }
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
        uint sec = (uint)seconds % 60;
        string secstr = (sec<10)? "0"+std::to_string(sec): std::to_string(sec);
        return (std::to_string((uint)seconds/60) + ":" + secstr);
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
#include <iostream>
#include <fstream>
    private:
    void process_mem_usage(string& vm_used, string& res_used) {
        using std::ios_base;
        using std::ifstream;
        using std::string;

        double vm_usage     = 0.0;
        double resident_set = 0.0;

        // 'file' stat seems to give the most reliable results
        ifstream stat_stream("/proc/self/stat",ios_base::in);

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
};
#endif

#endif
