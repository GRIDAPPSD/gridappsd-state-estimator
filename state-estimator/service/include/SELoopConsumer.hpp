#ifndef SELOOPCONSUMER_HPP
#define SELOOPCONSUMER_HPP

#include "json.hpp"
using json = nlohmann::json;

#include "cs.h"
#include "klu.h"

#include "SEConsumer.hpp"

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

#ifndef A5LIST
#define A5LIST std::list<std::array<unsigned int, 5>>
#endif

// negligable
#define NEGL 0.00000001

// This class listens for system state messages
class SELoopConsumer : public SEConsumer {
    protected:
    string simid;

    // system state
    private:
//  cs *x, *P;      // state model
#ifndef DIAGONAL_P
    cs *P;          // x comes from V and A but P is persistent 
#endif
    cs *F, *Q;      // process model
    cs *R;          // measurement covariance (diagonal)
    cs *eyex;       // identity matrix of dimension x
    uint xqty;      // number of states
    uint zqty;      // number of measurements

    
    // establish jacobian entry types
    enum dydx_type : uint {
        dPi_dvi, dPi_dvj, dQi_dvi, dQi_dvj, dvi_dvi,
        dPi_dTi, dPi_dTj, dQi_dTi, dQi_dTj, dTi_dTi,
    };

    A5LIST Jshape;  // list of J entries: {zidx,xidx,i,j,dydx_type}

    private:
    json jtext;     // object holding the input message
    json jstate;    // object holding the output message

    // system topology definition
    public:
    uint node_qty;      // number of nodes
    SLIST node_names;   // node names [list of strings]
    SIMAP node_idxs;    // node positional indices [node->int]
    SCMAP node_vnoms;   // complex nominal voltages of nodes
    SSMAP node_bmrids;  // string mRIDs of the associated buses
    SSMAP node_phs;     // string phases
    IMMAP Yphys;        // Ybus [node->[row->col]] [physical units]
    ISMAP node_name_lookup;

    // system state
    private:
    ICMAP Vpu;          // voltage state in per-unit
    IMMAP A;            // regulator tap ratios <- we need reg information
#ifdef DIAGONAL_P
    IDMAP Uvmag;        // variance of voltage magnitudes (per-unit)
    IDMAP Uvarg;        // variance of voltage angles (per-unit)
#endif

    private:
    double sbase;
    IMMAP Ypu;
    
    private:
    SensorArray zary;

    private:
    ofstream state_fh;  // file to record states

    private:
    SEProducer *statePublisher = 0;

#ifdef DEBUG_PRIMARY
    private:
    bool firstEstimateFlag = true;
    uint timezero;
#endif


    public:
    SELoopConsumer(const string& brokerURI, 
                const string& username,
                const string& password,
                const string& target,
                const string& mode,
                const string& simid,
                const SensorArray& zary,
                const uint& node_qty,
                const SLIST& node_names,
                const SIMAP& node_idxs,
                const SCMAP& node_vnoms,
                const SSMAP& node_bmrids,
                const SSMAP& node_phs,
                const ISMAP& node_name_lookup,
                const double sbase,
                const IMMAP& Yphys,
                const IMMAP& A) {
        this->brokerURI = brokerURI;
        this->username = username;
        this->password = password;
        this->target = target;
        this->mode = mode;
        this->simid = simid;
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
        this->A = A;
    }


    private:
    virtual void init() {
        // set up the output message json object
        jstate["simulation_id"] = simid;

#ifdef DEBUG_FILES
        // create the output directory if needed
        if (!opendir("output")) {
            // if output is already a symbolic link to a shared
            // folder as intended, this won't be needed
            mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }

        // create simulation parent directory
        std::string simpath = "output/sim_" + simid + "/";
        mkdir(simpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        // create init directory
        std::string initpath = simpath + "init/";
        mkdir(initpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        // Construct the producer that will be used to publish the state estimate
        string sePubTopic = "goss.gridappsd.state-estimator.out."+simid;
        statePublisher = new SEProducer(brokerURI,username,password,sePubTopic,"topic");
#ifdef DEBUG_PRIMARY
        cout << "State Estimate Message Producer Constructed.\n\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Establish Dimension of State Space and Measurement Space
        // --------------------------------------------------------------------
        xqty = 2*node_qty;
        zqty = zary.zqty;
#ifdef DEBUG_PRIMARY
        cout << "zary.zqty is " << zary.zqty << "\n\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Initialize Voltages (complex per-unit)
        // --------------------------------------------------------------------
        for ( auto& node_name : node_names ) {
            // Important: V is indexed by node index like A and Y
//          V[node_idxs[node_name]] = node_vnoms[node_name];
            Vpu[node_idxs[node_name]] = 1.0;
        }
#ifdef DEBUG_PRIMARY
        cout << "Voltages Initialized.\n\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Initialize State Covariance Matrix
        // --------------------------------------------------------------------
        double span_vmag = 1.0;
        double span_varg = 1.0/3.0*PI;
        double span_taps = 0.2;

#ifdef DIAGONAL_P
        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            Uvmag[idx] = 0.002*span_vmag;
            Uvarg[idx] = 0.002*span_varg;
        }
#else
        cs *P = gs_doubleval_diagonal(node_qty, 0.002*span_vmag, 0.002*span_varg);
#ifdef DEBUG_FILES
        print_cs_compress(P,initpath+"Pinit.csv");
#endif
#endif

#ifdef DEBUG_PRIMARY
        cout << "State Covariance Matrix Initialized.\n\n" << std::flush;
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
                        Jshape.push_back({zidx,j-1,i,i,dPi_dvi});
                        Jshape.push_back({zidx,node_qty+j-1,i,i,dPi_dTi});
                    } else {
                        // dPi/dvj and dPi/dTj exist for nodes adjacent to i
                        Jshape.push_back({zidx,j-1,i,j,dPi_dvj});
                        Jshape.push_back({zidx,node_qty+j-1,i,j,dPi_dTj});
                    }
                }
            }

            else
            if ( !ztype.compare("Qi") ) {
                for ( auto& row_pair : Yphys[i] ) {
                    uint j = row_pair.first;
                    if ( j == i ) {
                        // dPi/dvi and dPi/dTi exist for node i
                        Jshape.push_back({zidx,j-1,i,i,dQi_dvi});
                        Jshape.push_back({zidx,node_qty+j-1,i,i,dQi_dTi});
                    } else {
                        // dPi/dvj and dPi/dTj exists for nodes adjacent to i
                        Jshape.push_back({zidx,j-1,i,j,dQi_dvj});
                        Jshape.push_back({zidx,node_qty+j-1,i,j,dQi_dTj});
                    }
                }
            }

            else
            if ( !ztype.compare("aji") ) {
                // Later
            }

            else
            if
            ( !ztype.compare("vi") ) {
                // dvi/dvi is the only partial that exists
                Jshape.push_back({zidx,i-1,i,i,dvi_dvi});
            }

            else
            if ( !ztype.compare("Ti") ) {
                // dTi/dTi is the only partial that exists
                Jshape.push_back({zidx,node_qty+i-1,i,i,dTi_dTi});
            }

            else { 
                cout << "ERROR: Unrecognized measurement type " << 
                    ztype << std::flush;
            }
        }
#ifdef DEBUG_PRIMARY
        cout << "Jshape Jacobian elements: " << Jshape.size() << "\n" << std::flush;
#endif


        // --------------------------------------------------------------------
        // Compute Ypu
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        cout << "Computing Ypu time ... " << std::flush;
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
        cout << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        // write to file
        ofstream ofh;
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
                    } catch ( const std::out_of_range& oor) {
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
        cout << "writing " << initpath+"Yphys.csv\n" << std::flush;
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
        ofh.open(initpath+"vnoms.csv",ofstream::out);
        ofh << std::setprecision(16);
        cout << "writing " << initpath+"vnoms.csv\n" << std::flush;
        std::vector<complex<double>> vnoms(node_qty);
        for ( auto& node_name : node_names ) {
            cout << node_name << "\n" << std::flush;
            cout << "\tidx is " << node_idxs[node_name] << "\n" << std::flush;
            cout << "\tvnom is " << node_vnoms[node_name] << "\n" << std::flush;
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
        cout << "writing " << initpath+"nodem.csv\n" << std::flush;
        for ( auto& node_name : node_names )
            ofh << node_name << "\n";
        ofh.close();
#endif

#ifdef DEBUG_FILES
        // write sensor information to file
        ofh.open(initpath+"meas.txt",ofstream::out);
        cout << "writing output/init/meas.txt\n" << std::flush;
        ofh << "sensor_type\tsensor_name\tnode1\tnode2\tvalue\tsigma\n";
        for ( auto& zid : zary.zids ) {
            ofh << zary.ztypes[zid] << "\t"
                << zid << "\t"
                << zary.znode1s[zid] << "\t"
                << zary.znode2s[zid] << "\t"
                << zary.zvals[zid] << "\t"
                << zary.zsigs[zid] << "\n";
        } ofh.close();

        cout << "done writing\n" << std::flush;
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
        cout << "Initializing F ... " << std::flush;
#endif

        F = gs_singleval_diagonal(xqty, 1.0);
        //cout << "F PRINT\n" << std::flush;
        //cs_print(F, 0);
        //cout << "======================================\n" << std::flush;
#ifdef DEBUG_FILES
        print_cs_compress(F,initpath+"F.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "F is " << F->m << " by " << F->n << " with " << F->nzmax << " entries\n" << std::flush;
#endif

        // process covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing Q ... " << std::flush;
#endif
        Q = gs_doubleval_diagonal(node_qty, 0.001, 0.001*PI);
#ifdef DEBUG_FILES
        print_cs_compress(Q,initpath+"Q.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "Q is " << Q->m << " by " << Q->n << " with " << Q->nzmax << " entries\n" << std::flush;
#endif

        // identity matrix of dimension x (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing eyex ... " << std::flush;
#endif
        eyex = gs_singleval_diagonal(xqty, 1.0);
#ifdef DEBUG_FILES
        print_cs_compress(eyex,initpath+"eyex.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "eyex is " << eyex->m << " by " << eyex->n << " with " << eyex->nzmax << " entries\n" << std::flush;
#endif

        // measurement covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing R ... " << std::flush;
#endif
        R = gs_spalloc_diagonal(zqty);
        for ( auto& zid : zary.zids )
            gs_entry_diagonal(R,zary.zidxs[zid],zary.zsigs[zid]/1000000.0);
#ifdef DEBUG_FILES
        print_cs_compress(R,initpath+"R.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "R is " << R->m << " by " << R->n << " with " << R->nzmax << " entries\n" << std::flush;
#endif
        
#ifdef DEBUG_FILES
        // print initial state vector
        ofh.open(initpath+"Vpu.csv",ofstream::out);
        ofh << std::setprecision(16);
        cout << "writing " << initpath+"Vpu.csv\n\n" << std::flush;
        cout << "node_qty is " << node_qty << "\n" << std::flush;

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            cout << inode << " idx is " << i << "\n" << std::flush;
            try {
                complex<double> tmp = Vpu.at(i);
                double tmpre = tmp.real();
                double tmpim = tmp.imag();
                ofh << tmpre 
                    << ( tmpim >= 0 ? "+" : "-" )
                    << std::abs(tmpim) << "i" 
                    << "\n";
            } catch ( const std::out_of_range& oor) {
                ofh << "0+0i" << "\n";
            }
        } ofh.close();
#endif

#ifdef DEBUG_FILES
        // initial measurement vector (these actually don't need to be done here)
        cs* z; this->sample_z(z);
        print_cs_compress(z,initpath+"z.csv"); 
        cs_spfree(z);

        cs* h; this->calc_h(h);
        print_cs_compress(h,initpath+"h.csv"); 
        cs_spfree(h);
        
        cs* J; this->calc_J(J);
        print_cs_compress(J,initpath+"J.csv"); 
        cs_spfree(J);
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


    // ------------------------------------------------------------------------
    // PROCESS ()
    // ------------------------------------------------------------------------
    public:
    virtual void process() {
#ifdef DEBUG_PRIMARY
        cout << "\nProcessing message of " << text.length() 
            << " bytes recieved on measurement topic.\n" << std::flush;
#endif

        jtext = json::parse(text);
        uint timestamp = jtext["message"]["timestamp"];
#ifdef DEBUG_PRIMARY
        cout << "\ttimestamp: " << timestamp << "\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Reset the new measurement indicator
        // --------------------------------------------------------------------
        for ( auto& zid : zary.zids ) zary.znew[zid] = false;

        // --------------------------------------------------------------------
        // Use the simulation output to update the states
        // --------------------------------------------------------------------
        // This needs to translate simulation output into zary.zvals[zid]
        //  - We need to iterate over the "measurements" in the simoutput
        //  - As in SensorDefConsumer.hpp, measurements can have multiple z's

        // Next, update new measurements
        for ( auto& m : jtext["message"]["measurements"] ) {
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
                zary.zvals[mmrid+"_Vmag"] = 
                    vmag_phys / abs(node_vnoms[zary.znode1s[zid]]);
                zary.znew[mmrid+"_Vmag"] = true;

                // update the voltage phase
                // --- LATER ---
                // -------------
            }

            else if ( !m_type.compare("") ) { 
            }
        }
        
        // --------------------------------------------------------------------
        // Estimate the state
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        cout << "\nEstimating state ... \n" << std::flush;
#endif
        this->estimate(timestamp);

// STOP EARLY
//      doneLatch.countDown();
//      return;

        // --------------------------------------------------------------------
        // Package and publish the state
        // --------------------------------------------------------------------
        
        // Initializae json
        json jstate;
        jstate["simulation_id"] = simid;
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
            state["angle"] = 180.0/PI * arg ( vnom * Vpu[idx] );
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
        std::string simpath = "output/sim_" + simid + "/";
        state_fh.open(simpath+"vmag_per-unit.csv",ofstream::app);
        state_fh << timestamp << ',';
        uint ctr = 0;
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            state_fh << vmag_pu << ( ++ctr < node_qty ? "," : "\n" );
        }
        state_fh.close();
#endif

        // --------------------------------------------------------------------
        // Check whether to stop
        // --------------------------------------------------------------------
        if ( text == "stop" ) {
            cout << "TIME TO STOP!\n" << std::flush;
            statePublisher->close();
            doneLatch.countDown();
        }
    }


    private:
    void estimate(const uint& timestamp) {
#ifdef DEBUG_PRIMARY
        double estimateStartTime = getWallTime();
        cout << "xqty is " << xqty << "\n" << std::flush;
        cout << "zqty is " << zqty << "\n" << std::flush;
        cout << "F is " << F->m << " by " << F->n << " with " << F->nzmax << " entries\n" << std::flush;
        cout << "Q is " << Q->m << " by " << Q->n << " with " << Q->nzmax << " entries\n" << std::flush;
#endif

        // TODO: WE NEED TO HANDLE R-MASK IN HERE SOMEWHERE

#ifdef DEBUG_FILES
        // set filename path based on timestamp
        std::string simpath = "output/sim_" + simid + "/ts_";
        std::ostringstream out;
        out << simpath << timestamp << "/";
        std::string tspath = out.str();

        // create timestamp directory
        mkdir(tspath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

        // x, z, h, and J will be maintained here
        
#ifdef DEBUG_FILES
        ofstream ofh;
        ofh.open(tspath+"Vpu.csv",ofstream::out);
        ofh << std::setprecision(16);
        cout << "writing " << tspath+"Vpu.csv\n\n" << std::flush;
        cout << "node_qty is " << node_qty << "\n" << std::flush;

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
            cout << inode << " idx is " << i << "\n" << std::flush;
            try {
                complex<double> tmp = Vpu.at(i);
                double tmpre = tmp.real();
                double tmpim = tmp.imag();
                ofh << tmpre 
                    << ( tmpim >= 0 ? "+" : "-" )
                    << std::abs(tmpim) << "i" 
                    << "\n";
            } catch ( const std::out_of_range& oor) {
                ofh << "0+0i" << "\n";
            }
        } ofh.close();
#endif

        // --------------------------------------------------------------------
        // Predict Step
        // --------------------------------------------------------------------
        // -- compute p_predict = F*P*F' + Q | F=I (can be simplified)
#ifdef DIAGONAL_P
#ifdef DEBUG_PRIMARY
        cout << "prep_P ... " << std::flush;
#endif
        cs *P; this->prep_P(P);
#endif
#ifdef DEBUG_PRIMARY
        cout << "P is " << P->m << " by " << P->n << 
            " with " << P->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P,tspath+"P.csv");
#endif

        cs *P1 = cs_transpose(F,1);
        if (!P1) cout << "ERROR: null P1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P1 is " << P1->m << " by " << P1->n << 
            " with " << P1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P1,tspath+"P1.csv");
#endif

        cs *P2 = cs_multiply(P,P1); cs_spfree(P); cs_spfree(P1);
        if (!P2) cout << "ERROR: null P2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P2 is " << P2->m << " by " << P2->n << 
            " with " << P2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P2,tspath+"P2.csv");
#endif

        cs *P3 = cs_multiply(F,P2); cs_spfree(P2);
        if (!P3) cout << "ERROR: null P3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P3 is " << P3->m << " by " << P3->n << 
            " with " << P3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P3,tspath+"P3.csv");
#endif

        // --------------------------------------------------------------------
        // Update Step
        // --------------------------------------------------------------------
        // -- compute S = J*P_predict*J' + R

        cs *Ppre = cs_add(P3,Q,1,1); cs_spfree(P3);
        if (!Ppre) cout << "ERROR: null Ppre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Ppre is " << Ppre->m << " by " << Ppre->n << 
            " with " << Ppre->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Ppre,tspath+"Ppre.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "calc_J ... " << std::flush;
#endif
        cs *J; this->calc_J(J);
#ifdef DEBUG_PRIMARY
        cout << "J is " << J->m << " by " << J->n << 
            " with " << J->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(J,tspath+"J.csv");
#endif

        cs *S1 = cs_transpose(J,1);
        if (!S1) cout << "ERROR: null S1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "S1 is " << S1->m << " by " << S1->n << 
            " with " << S1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S1,tspath+"S1.csv");
#endif

        cs *S2 = cs_multiply(Ppre,S1);
        if (!S2) cout << "ERROR: null S2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "S2 is " << S2->m << " by " << S2->n << 
            " with " << S2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S2,tspath+"S2.csv");
#endif

        cs *S3 = cs_multiply(J,S2); cs_spfree(S2);
        if (!S3) cout << "ERROR: null S3\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "S3 is " << S3->m << " by " << S3->n << 
            " with " << S3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(S3,tspath+"S3.csv");
#endif

        cs *Supd = cs_add(R,S3,1,1); cs_spfree(S3);
        if (!Supd) cout << "ERROR: null Supd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Supd is " << Supd->m << " by " << Supd->n << 
            " with " << Supd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Supd,tspath+"Supd.csv");
#endif

#ifdef DEBUG_PRIMARY
        double startTime;
        string vm_used, res_used;
#endif
#if 000
        double *rhs;
#else
        cs *K3 = gs_spalloc_fullsquare(zqty);
        double *rhs = K3->x;
        //double *rhs = (double *)calloc(zqty*zqty, sizeof(double));
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
            cout << "klu_factor time ... " << std::flush;
            startTime = getWallTime();
#endif
            klunum = klu_factor(Supd->p,Supd->i,Supd->x,klusym,&klucom);
            if (!klunum) {
#ifdef DEBUG_PRIMARY
                cout << "Common->status is: " << klucom.status << "\n" << std::flush;
                if ( klucom.status == 1 ) cout << "\tKLU_SINGULAR\n" << std::flush;
#endif
                throw "klu_factor failed";
            }

#ifdef DEBUG_PRIMARY
            cout << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
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
            // TODO: BOTTLENECK
            cout << "*** BOTTLENECK: klu_solve completion time ... " << std::flush;
            startTime = getWallTime();
#endif
            klu_solve(klusym,klunum,Supd->m,Supd->n,rhs,&klucom);
            if (klucom.status) {
#ifdef DEBUG_PRIMARY
                cout << "Common->status is: " << klucom.status << "\n" << std::flush;
#endif
                throw "klu_solve failed";
            }

#ifdef DEBUG_PRIMARY
            cout << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            if (firstEstimateFlag) {
                timezero = timestamp;
                firstEstimateFlag = false;
            }
            process_mem_usage(vm_used, res_used);
            //cout << "*** Post-klu_solve virtual memory: " << vm_used << ", timestep: " << timestamp-timezero << "\n" << std::flush;
            cout << "*** Post-klu_solve resident memory: " << res_used << ", timestep: " << timestamp-timezero << "\n" << std::flush;
#endif

            // free klusym and klunum or memory leak results
            klu_free_symbolic(&klusym, &klucom);
            klu_free_numeric(&klunum, &klucom);
        } catch (const char *msg) {
            cout << "KLU ERROR: " << msg << "\n" << std::flush;
            return;
        }
        cs_spfree(Supd);

#ifdef DEBUG_PRIMARY
        if ( K3 ) cout << "K3 is " << K3->m << " by " << K3->n <<
                " with " << K3->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K3,tspath+"K3.csv");
#endif

        // -- compute K = P_predict*J'*S^-1
        // GDB S1 and K1 are both J transpose so use S1 here
        cs *K2 = cs_multiply(Ppre,S1); cs_spfree(S1);
        if (!K2) cout << "ERROR: null K2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "K2 is " << K2->m << " by " 
            << K2->n << " with " << K2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K2,tspath+"K2.csv");
#endif

#ifdef DEBUG_PRIMARY
        // TODO: BOTTLENECK
        cout << "*** BOTTLENECK: K updated time ... " << std::flush;
        startTime = getWallTime();
#endif
        cs *Kupd = cs_multiply(K2,K3); cs_spfree(K2); cs_spfree(K3);
        if ( !Kupd ) cout << "ERROR: Kupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        cout << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        if ( Kupd ) cout << "Kupd is " << Kupd->m << " by " << Kupd->n <<
                " with " << Kupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Kupd,tspath+"Kupd.csv");
#endif

        // -- compute x_update = x_predict + K * y
        // -- compute y = z - h
#ifdef DEBUG_PRIMARY
        cout << "sample_z ... " << std::flush;
#endif
        cs *z; this->sample_z(z);
#ifdef DEBUG_PRIMARY
        cout << "z is " << z->m << " by " << z->n << 
            " with " << z->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(z,tspath+"z.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "calc_h ... " << std::flush;
#endif
        cs *h; this->calc_h(h);
#ifdef DEBUG_PRIMARY
        cout << "h is " << h->m << " by " << h->n << 
            " with " << h->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(h,tspath+"h.csv");
#endif

        cs *yupd = cs_add(z,h,1,-1); cs_spfree(z); cs_spfree(h);
        if (!yupd) cout << "ERROR: null yupd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "yupd is " << yupd->m << " by " << yupd->n << 
            " with " << yupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(yupd,tspath+"yupd.csv");
#endif
        cs *x1 = cs_multiply(Kupd,yupd); cs_spfree(yupd);
        if ( !x1 ) cout << "ERROR: x1 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "x1 is " << x1->m << " by " << x1->n <<
                " with " << x1->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(x1,tspath+"x1.csv");
#endif

        // -- compute x_predict = F*x | F=I (to improve performance, skip this)
#ifdef DEBUG_PRIMARY
        cout << "prep_x ... " << std::flush;
#endif
        cs *x; this->prep_x(x);
#ifdef DEBUG_PRIMARY
        cout << "x is " << x->m << " by " << x->n << 
            " with " << x->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(x,tspath+"x.csv");
#endif

        cs *xpre = cs_multiply(F,x); cs_spfree(x);
        if (!xpre) cout << "ERROR: null xpre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "xpre is " << xpre->m << " by " << xpre->n << 
            " with " << xpre->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xpre,tspath+"xpre.csv");
#endif

        cs *xupd = cs_add(xpre,x1,1,1); cs_spfree(x1); cs_spfree(xpre);
        if ( !xupd ) cout << "ERROR: xupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "xupd is " << xupd->m << " by " << xupd->n <<
                " with " << xupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(xupd,tspath+"xupd.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "P4 time ... " << std::flush;
        startTime = getWallTime();
#endif
        // -- compute P_update = (I-K_update*J)*P_predict
        cs *P4 = cs_multiply(Kupd,J); cs_spfree(Kupd); cs_spfree(J);
        if ( !P4 ) cout << "ERROR: P4 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        cout << getMinSec(getWallTime()-startTime) << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        if ( P4 ) cout << "P4 is " << P4->m << " by " << P4->n <<
                " with " << P4->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P4,tspath+"P4.csv");
#endif

        cs *P5 = cs_add(eyex,P4,1,-1); cs_spfree(P4);
        if ( !P5 ) cout << "ERROR: P5 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P5 is " << P5->m << " by " << P5->n << 
                " with " << P5->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P5,tspath+"P5.csv");
#endif

#ifdef DIAGONAL_P
        cs *Pupd = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !Pupd ) cout << "ERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P updated is " << Pupd->m << " by " << Pupd->n << 
                " with " << Pupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Pupd,tspath+"Pupd.csv");
#endif

#else
        // re-allocate P for the updated state
        P = cs_multiply(P5,Ppre); cs_spfree(P5); cs_spfree(Ppre);
        if ( !P ) cout << "ERROR: P updated null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P updated is " << P->m << " by " << P->n << 
                " with " << P->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(P,tspath+"Pupd.csv");
#endif
#endif


        // --------------------------------------------------------------------
        // Update persistant state (Vpu and A)
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        cout << "calling decompress_state_xupd\n" << std::flush;
#endif
        if (xupd) {
            decompress_state(xupd);
            cs_spfree(xupd);
        }

#ifdef DIAGONAL_P
        // extract the diagonal of P
#ifdef DEBUG_PRIMARY
        cout << "calling decompress_variance_Pupd\n" << std::flush;
#endif
        if (Pupd) {
            decompress_variance(Pupd);
            cs_spfree(Pupd);
        }
#endif

#ifdef DEBUG_PRIMARY
        cout << "\n*** Total estimate time: " << 
            getMinSec(getWallTime()-estimateStartTime) << ", timestep: " <<
            timestamp-timezero << "\n" << std::flush;
#ifdef DEBUG_SIZES
#ifndef DIAGONAL_P
        uint Psize = cs_size(P);
        print_sizeof(Psize, "P");
#endif
        uint Fsize = cs_size(F);
        print_sizeof(Fsize, "F");
        uint Qsize = cs_size(Q);
        print_sizeof(Qsize, "Q");
        uint Rsize = cs_size(R);
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
        print_sizeof(Fsize+Qsize+Rsize+eyexsize+Vpusize+Uvmagsize+Uvargsize, "Total");
#endif
#endif
        process_mem_usage(vm_used, res_used);
        //cout << "End-estimate virtual memory: " << vm_used << ", timestep: " << timestamp-timezero << "\n" << std::flush;
        cout << "End-estimate resident memory: " << res_used << ", timestep: " << timestamp-timezero << "\n\n" << std::flush;
#endif
    }


    private:
    void decompress_state(cs *&x) {
        // copy state into vector (states, especially phase, can be 0)
        vector<double> xvec(x->m,0.0);
        for ( uint idx = 0 ; idx < x->nzmax ; idx++ )
            xvec[x->i[idx]] = x->x[idx];
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
//      for ( auto& reg_name : SLIST reg_names ) {}
    }

#include <float.h>

#ifdef DIAGONAL_P
    private:
    void decompress_variance(cs *&Pmat) {
        // vector to store the state variance (diagonal of Pmat)
        vector<double> uvec(Pmat->n);
        // Pmat is in compressed-column form; iterate over columns
        for ( uint j = 0; j < Pmat->n ; j++ ) {
#if 0000
            // for P, the first entry for each column is always the diagonal
            uint p = Pmat->p[j];
            uvec[j] = Pmat->x[p];
            //for ( uint p = Pmat->p[j] ; p < Pmat->p[j+1] ; p++ )
            //    test whether the first entry is always the diagonal
            //    if (j == Pmat->i[Pmat->p[j]]) cout << "Diagonal First Test: PASS!!!!!\n" << std::flush;
            //    else cout << "Diagonal First Test: fail\n" << std::flush;
#else
            // iterate over existing data in column j
            for ( uint p = Pmat->p[j] ; p < Pmat->p[j+1] ; p++ ) {
                // get the row index for existing data
                if ( Pmat->i[p] == j ) {
                    // if this is a diagonal entry, extract it
                    uvec[j] = Pmat->x[p];
                    break;
                }
            }
#endif
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
        cout << "Uvmag min: " << minMag << ", max: " << maxMag << "\n" << std::flush;
        cout << "Uvarg min: " << minArg << ", max: " << maxArg << "\n" << std::flush;
#endif
    }
#endif


    private:
    void prep_x(cs *&x) {
        // Prepare x
        x = gs_spalloc_firstcol(xqty);
        if (!x) cout << "ERROR: null x\n" << std::flush;

        for ( auto& node_name : node_names ) {
            // Find the per-unit voltage of active node
            uint idx = node_idxs[node_name];
            complex<double> Vi = Vpu[idx];
            // Add the voltage magnitude to x
            if ( abs(Vi) > NEGL ) gs_entry_firstcol(x,idx-1,abs(Vi));
            // Add the voltage angle to x
            if ( arg(Vi) ) gs_entry_firstcol(x,node_qty + idx-1,arg(Vi));
        }
    }


#ifdef DIAGONAL_P
    private:
    void prep_P(cs *&Pmat) {
        // Prepare P as a diagonal matrix from state uncertanty
        Pmat = gs_spalloc_diagonal(xqty);
        if (!Pmat) cout << "ERROR: null Pmat in prep_P\n" << std::flush;

        for ( auto& node_name : node_names ) {
            uint idx = node_idxs[node_name];
            // insert the voltage magnitude variance
            if ( Uvmag[idx] ) gs_entry_diagonal(Pmat,idx-1,Uvmag[idx]);
            // insert the voltage phase variance
            if ( Uvarg[idx] ) gs_entry_diagonal(Pmat,node_qty + idx-1,Uvarg[idx]);
        }
    }
#endif


    private:
    void sample_z(cs *&z) {
        // measurements have been loaded from the sim output message to zary
        z = gs_spalloc_firstcol(zqty);
        if (!z) cout << "ERROR: null z\n" << std::flush;

        for ( auto& zid : zary.zids ) {
            if ( zary.zvals[zid] > NEGL || -zary.zvals[zid] > NEGL )
                gs_entry_firstcol(z,zary.zidxs[zid],zary.zvals[zid]);
        }
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
            cout << "ERROR: Unexpected call to set_n with i=0\n" << std::flush;
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
            } catch(...) {}
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
                        auto Arow = A.at(i);
                        try {
                            ai = real(Arow.at(j));
                        } catch(...) {}
                    } catch(...) {}
                    // We know the nodes are coupled; check for Aji
                    // NOTE: A is never iterated over - we don't need at()
                    aj = 1;
                    try {
                        auto Arow = A.at(j);
                        try {
                            aj = real(Arow.at(i));
                        } catch (...) {}
                    } catch(...) {}
                } catch(...) {
                    cout << "ERROR: set_n catch on Yrow.at(j) lookup\n" << std::flush;
                    exit(1);
                }
            } catch(...) {
                cout << "ERROR: set_n catch on Ypu.at(i) lookup\n" << std::flush;
                exit(1);
            }
            g = real(-1.0*Yij);
            b = imag(-1.0*Yij);
        }
    }
    

    private:
    void calc_h(cs *&h) {
        // each z component has a measurement function component
        h = gs_spalloc_firstcol(zqty);
        if (!h) cout << "ERROR: null h\n" << std::flush;

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
                } catch(...) {}
                // Insert the measurement component
                if ( abs(Pi) > NEGL ) gs_entry_firstcol(h,zidx,Pi);
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
                } catch(...) {}
                if ( abs(Qi) > NEGL ) gs_entry_firstcol(h,zidx,Qi);
            }
            else if ( !zary.ztypes[zid].compare("aji" ) ) {
                // aji is a direct state measurement
            }
            else if ( !zary.ztypes[zid].compare("vi") ) {
                // vi is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
                if ( abs(Vpu[i]) > NEGL ) gs_entry_firstcol(h,zidx,abs(Vpu[i]));
            }
            else if ( !zary.ztypes[zid].compare("Ti") ) {
                // Ti is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
                if ( arg(Vpu[i]) > NEGL ) gs_entry_firstcol(h,zidx,arg(Vpu[i]));
            }
            else { 
                cout << "WARNING: Undefined measurement type " + ztype + "\n" << std::flush;
            }
        }
#ifdef DEBUG_PRIMARY
        cout << "calc_h time: " << getMinSec(getWallTime()-startTime)
            << "\n" << std::flush;
#endif
    }


    private:
    void calc_J(cs *&J) {
#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
        // each z component has a Jacobian component for each state
#ifdef XNORAW
        J = gs_spalloc_generic(zqty,xqty,Jshape.size());
        if (!J) cout << "ERROR: null J\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "J is " << J->m << " by " 
            << J->n << " with " << J->nzmax << " entries\n" << std::flush;
#endif
#else
        cs *Jraw = cs_spalloc(zqty,xqty,Jshape.size(),1,1);
        if (!Jraw) cout << "ERROR: null Jraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Jraw is " << Jraw->m << " by " 
            << Jraw->n << " with " << Jraw->nzmax << " entries\n" << std::flush;
#endif
#endif
#ifdef DEBUG_HEARTBEAT
        uint beat_ctr = 0;
        uint total_ctr = Jshape.size();
#endif
        // loop over existing Jacobian entries
        for ( std::array<unsigned int, 5>& Jpartial : Jshape ) {
#ifdef DEBUG_HEARTBEAT
            if ( ++beat_ctr % 100 == 0 ) 
                cout << "--- calc_J heartbeat - " << beat_ctr << ", " <<
                    getPerComp(beat_ctr, total_ctr) << ", " <<
                    getMinSec(getWallTime()-startTime) << " ---\n" << std::flush;
#endif
            
            // Unpack entry data
            uint zidx = Jpartial[0];
            uint xidx = Jpartial[1];
            uint i = Jpartial[2];
            uint j = Jpartial[3];
            uint entry_type = Jpartial[4];

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
#ifdef XNORAW
                if ( abs(dP > NEGL ) ) gs_entry_generic(J,zidx,xidx,dP);
#else
                if ( abs(dP > NEGL ) ) cs_entry(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dPi_dvj ) {
                // --- compute dPi/dvj
                double dP = 0;
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                dP = -1.0 * vi/ai/aj * (g*cos(T) + b*sin(T));
#ifdef XNORAW
                if ( abs(dP) > NEGL ) gs_entry_generic(J,zidx,xidx,dP);
#else
                if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
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
#ifdef XNORAW
                if ( abs(dQ) > NEGL ) gs_entry_generic(J,zidx,xidx,dQ);
#else
                if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dQi_dvj ) {
                // --- compute dQi/dvj
                double dQ = 0;
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                dQ = -1.0 * vi/ai/aj * (g*sin(T) - b*cos(T));
#ifdef XNORAW
                if ( abs(dQ) > NEGL ) gs_entry_generic(J,zidx,xidx,dQ);
#else
                if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dvi_dvi ) {
                 // --- compute dvi/dvi
#ifdef XNORAW
                 gs_entry_generic(J,zidx,xidx,1.0);
#else
                 cs_entry(Jraw,zidx,xidx,1.0);
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
#ifdef XNORAW
                if ( abs(dP) > NEGL ) gs_entry_generic(J,zidx,xidx,dP);
#else
                if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
#endif
            }

            else
            if ( entry_type == dPi_dTj ) {
                 // --- compute dP/dTj
                 double dP = 0;
                 auto& Yrow = Ypu.at(i);
                 complex<double> Yij = Yrow.at(j);
 
                 set_n(i,j);
                 dP = -1.0 * vi*vj/ai/aj * (g*sin(T) - b*cos(T));
#ifdef XNORAW
                 if ( abs(dP) > NEGL ) gs_entry_generic(J,zidx,xidx,dP);
#else
                 if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
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
#ifdef XNORAW
                if (abs(dQ) > NEGL ) gs_entry_generic(J,zidx,xidx,dQ);
#else
                if (abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dQi_dTj ) {
                // --- compute dQ/dTj
                double dQ = 0;
                auto& Yrow = Ypu.at(i);
                complex<double> Yij = Yrow.at(j);

                set_n(i,j);
                dQ = vi*vj/ai/aj * (g*cos(T) + b*sin(T));
#ifdef XNORAW
                if ( abs(dQ) > NEGL ) gs_entry_generic(J,zidx,xidx,dQ);
#else
                if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
#endif
            }

            else
            if ( entry_type == dTi_dTi ) {
#ifdef XNORAW
                gs_entry_generic(J,zidx,xidx,1.0);
#else
                cs_entry(Jraw,zidx,xidx,1.0);
#endif
            }

            else {
                cout << "WARNING: Undefined jacobian element type type " + 
                    entry_type << "\n" << std::flush;
            }

            // loop over regulator tap ratio states
            // --- LATER ---
            // -------------

        }
#ifndef XNORAW
        J = cs_compress(Jraw);
        //cout << "Jraw PRINT\n" << std::flush;
        //cs_print(Jraw, 0);
        //cout << "======================================\n" << std::flush;
        //cout << "J PRINT\n" << std::flush;
        //cs_print(J, 0);
        //cout << "======================================\n" << std::flush;
        cs_spfree(Jraw);
#else
        //cout << "J PRINT\n" << std::flush;
        //cs_print(J, 0);
        //cout << "======================================\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
        double endTime = getWallTime();
        cout << "calc_J time: " <<
            getMinSec(endTime-startTime) << "\n" << std::flush;
#endif
    }
    

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
        //cs *A = cs_spalloc (mn, mn, mn*mn, 1, 0);
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

#if 000
    // this isn't correctly implemented so compile it out for now

    private:
    cs *gs_spalloc_generic(uint m, uint n, uint nzmax) {
        //cs *A = cs_spalloc (mn, mn, mn*mn, 1, 0);
        cs *A = (cs*)cs_calloc (1, sizeof (cs)) ;
        if (!A) return (NULL) ;
        A->m = m; A->n = n ;
        A->nzmax = nzmax ;
        A->nz = -1 ;
        A->p = (int*)cs_malloc (n+1, sizeof (int)) ;
        A->i = (int*)cs_malloc (nzmax, sizeof (int)) ;
        // make sure A->x starts zero'd out vs. regular cs_spalloc
        A->x = (double*)cs_calloc (A->nzmax, sizeof (double)) ;

        // TODO: this code assumes all entries and ordered, which is not
        // the case for a generic mxn sparse matrix
        A->p[n] = A->nzmax;
        for (uint jj=0; jj<n; jj++) {
            A->p[jj] = jj*n;
            for (uint ii=0; ii<m; ii++)
                A->i[jj*n + ii] = ii;
        }
        return (A);
    }

    private:
    void gs_entry_generic(cs *A, uint ii, uint jj, double value) {
#if 1000000
        cout << "$$$ gs_entry_generic row: " << ii << ", col: " << jj << ", val: " << value << "\n" << std::flush;
#else
        // TODO: this needs to be coded if actually used
        // the "square" method implementation
        A->x[jj*A->n + ii] = value;

        // stolen from the "firstcol" method as a template
        A->i[A->nzmax] = ii;
        A->x[A->nzmax] = value;
        A->nzmax++;
        A->p[1] = A->nzmax;
#endif
    }
#endif


#ifdef DEBUG_FILES
    private:
    void print_cs_compress(cs *&a, const string &filename="cs.csv") {
        // First copy into a map
        unordered_map<uint,unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
        // write to file
        ofstream ofh;
        ofh << std::setprecision(16);
        ofh.open(filename,ofstream::out);
        cout << "writing " + filename + "\n\n" << std::flush;
        for ( uint i = 0 ; i < a->m ; i++ )
            for ( uint j = 0 ; j < a->n ; j++ )
                ofh << mat[i][j] << ( j == a->n-1 ? "\n" : "," );
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

#ifdef DEBUG_HEARTBEAT
    private:
    string getPerComp(uint current, uint total) {
        return std::to_string((uint)(100.0 * current/total)) + "%";
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
                    cout << msg << " size: " << gbsize << " GB\n" << std::flush;
                } else {
                    cout << msg << " size: " << mbsize << " MB\n" << std::flush;
                }
            } else {
                cout << msg << " size: " << kbsize << " KB\n" << std::flush;
            }
        } else {
            cout << msg << " size: " << size << " bytes\n" << std::flush;
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

#endif
