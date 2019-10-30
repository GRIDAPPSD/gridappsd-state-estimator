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

// for mkdir and opendir
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#ifdef DEBUG_PRIMARY
#include <sys/time.h>
#include <time.h>
#endif

// SLIST holds the lists of node names and regulator names
#ifndef SLIST
#define SLIST std::list<std::string>
#endif

// SIMAP holds the one-indexed positions of nodes
#ifndef SIMAP
#define SIMAP std::unordered_map<std::string,uint>
#endif

// SDMAP holds x, z, and the set of regulator taps
#ifndef SDMAP
#define SDMAP std::unordered_map<std::string,double>
#endif

// SCMAP holds the complex node voltages
#ifndef SCMAP
#define SCMAP std::unordered_map<std::string,std::complex<double>>
#endif

// SSMAP holds the mapping between sensors and nodes
#ifndef SSMAP
#define SSMAP std::unordered_map<std::string,std::string>
#endif

// this holds the voltage state
#ifndef ICMAP
#define ICMAP std::unordered_map<uint,std::complex<double>>
#endif

#ifndef ISMAP
#define ISMAP std::unordered_map<uint,std::string>
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
    cs *P;          // x comes from V and A but P is persistent 
    cs *F, *Q;      // process model
    cs *R;
//  cs *z, *R;      // measurement model
//  cs *h, *J;
    cs *eyex;       // identity matrix of dimension x
    uint xqty;       // number of states
    uint zqty;       // number of measurements

//  cs *x, *xpre, *x1, *xupd;                       // state vector
//  cs *P, *Ppre, *P1, *P2, *P3, *P4, *P, *Pupd;    // state covariance
//  cs *y1, *yupd;                                  // residual vector
//  cs *S1, *S2, *S3, *Supd;                        // residual covariance
//  cs *K1, *K2, *K3, *Kupd;                        // gain matrix

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
//  cs *State_Cov;      // state covariance matrix;

    private:
    double sbase;
    IMMAP Ypu;
    
    private:
    SensorArray zary;

    private:
    ofstream state_fh;  // file to record states

    private:
    SEProducer *statePublisher = 0;

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
#ifdef DEBUG_SECONDARY
        for ( auto& node : node_names ) cout << node+"\n" << std::flush;
#endif
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
#ifdef DEBUG_SECONDARY
        cout << "------\n" << std::flush;
        for ( auto& node : this->node_names ) cout << node+"\n" << std::flush;
        cout << "------\n" << std::flush;
        for ( auto& zid : this->zary.zids ) {
            cout << "\t" << zid << "\t" << this->zary.zvals[zid] << "\n" << std::flush;
        }
        cout << "------\n" << std::flush;
#endif
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
        cs *Praw = cs_spalloc(0,0,xqty,1,1);
        if (!Praw) cout << "ERROR: null Praw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Praw is " << Praw->m << " by " 
            << Praw->n << " with " << Praw->nzmax << " entries\n" << std::flush;
#endif
        for ( uint idx = 0 ; idx < node_qty ; idx++ ) {
            // Add variance for the voltage magnitude state
            cs_entry(Praw,idx,idx,0.002*span_vmag);
            // Add variance for the voltage phase state
            cs_entry(Praw,node_qty+idx,node_qty+idx,0.002*span_varg);
        }
        P = cs_compress(Praw); cs_spfree(Praw);

#ifdef DEBUG_FILES
        print_cs_compress(P,initpath+"Pinit.csv");
#endif


#ifdef DEBUG_PRIMARY
        cout << "State Covariance Matrix Initialized.\n\n" << std::flush;
#endif


        // --------------------------------------------------------------------
        // Compute Ypu
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        cout << "Computing Ypu ...\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
#ifdef DEBUG_SECONDARY
        uint beat_ctr = 0;
        uint total_ctr = node_names.size();
#endif
        for ( auto& inode : node_names ) {
#ifdef DEBUG_SECONDARY
            if ( ++beat_ctr % 100 == 0 ) 
                cout << "--- Ypu heartbeat - " << beat_ctr << ", " <<
                    getPerComp(beat_ctr, total_ctr) << ", " << 
                    getMinSec(getWallTime()-startTime) << " ---\n" << std::flush;
#endif
#ifdef DEBUG_SECONDARY
            cout << inode << "\n" << std::flush;
#endif
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
        cout << "Ypu completion time: " <<
            getMinSec(getWallTime()-startTime) << "\n\n" << std::flush;
#endif

//      // print
//      for ( auto& inode : node_names ) {
//          uint i = node_idxs[inode];
//          try {
//              auto& row = Ypu.at(i);
//              for ( auto& jnode: node_names ) {
//                  uint j = node_idxs[jnode];
//                  try {
//                      complex<double> yij = row.at(j);
//                      complex<double> vnomi = node_vnoms[inode];
//                      complex<double> vnomj = node_vnoms[jnode];
//                      cout << "Y(" << i << "," << j << ") -> " << yij << "\n" << std::flush;
//                  } catch(...) {}
//              }
//          } catch(...) {}
//      }

#ifdef DEBUG_FILES
        // write to file
        ofstream ofh;
        ofh.open(initpath+"Ypu.csv",ofstream::out);
        ofh << std::setprecision(16);
#ifdef DEBUG_SECONDARY
        cout << "writing " << initpath+"Ypu.csv\n\n" << std::flush;
        cout << "node_qty is " << node_qty << "\n" << std::flush;
#endif

        for ( auto& inode : node_names ) {
            uint i = node_idxs[inode];
#ifdef DEBUG_SECONDARY
            cout << inode << " idx is " << i << "\n" << std::flush;
#endif
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
                            << ( ++jctr < node_qty ? ',' : "\n" );
                    } catch ( const std::out_of_range& oor) {
                        ofh << "0+0i" << ( ++jctr < node_qty ? ',' : "\n" );
                    }
                }
            } catch ( const std::out_of_range& oor ) {
                uint jctr = 0;
                for ( auto& jnode : node_names )
                    ofh << "0+0i" << ( ++jctr < node_qty ? ',' : "\n" );
            }
        } ofh.close();
#endif

#ifdef DEBUG_FILES
        // write Y to file
        ofh.open(initpath+"Yphys.csv",ofstream::out);
        ofh << std::setprecision(16);
        cout << "writing " << initpath+"Yphys.csv\n" << std::flush;
//      for ( auto& inode : node_names ) {
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
                            << ( j < node_qty ? ',' : "\n" );
                    } catch ( const std::out_of_range& oor ) {
                        ofh << "0+0i" << ( j < node_qty ? ',' : "\n" );
                    }
                }
            } catch ( const std::out_of_range& oor ) {
                for ( uint j = 0 ; j < node_qty ; j++ )
                    ofh << "0+0i" << ( j < node_qty ? ',' : "\n" );
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
//          vnoms[1] = 13;
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
        cs *Fraw = cs_spalloc(0,0,xqty,1,1);
        if (!Fraw) cout << "ERROR: null Fraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Fraw is " << Fraw->m << " by " 
            << Fraw->n << " with " << Fraw->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        cout << "Initializing F\n" << std::flush;
#endif
        for ( uint ii = 0 ; ii < xqty ; ii++ )
            cs_entry(Fraw,ii,ii,1);
        F = cs_compress(Fraw); cs_spfree(Fraw);
#ifdef DEBUG_FILES
        print_cs_compress(F,initpath+"F.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "F is " << F->m << " by " << F->n << " with " << F->nzmax << " entries\n\n" << std::flush;
#endif

        // process covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing Q\n" << std::flush;
#endif
        cs *Qraw = cs_spalloc(0,0,xqty,1,1);
        if (!Qraw) cout << "ERROR: null Qraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Qraw is " << Qraw->m << " by " 
            << Qraw->n << " with " << Qraw->nzmax << " entries\n" << std::flush;
#endif
        //for ( uint ii = 0 ; ii < xqty ; ii++ )
        //    cs_entry(Qraw,ii,ii,0.04*sqrt(1.0/4));      // TUNABLE
        for ( uint idx = 0 ; idx < node_qty ; idx++ ) {
            cs_entry(Qraw,idx,idx,0.001);
            cs_entry(Qraw,node_qty+idx,node_qty+idx,0.001*PI);
        }
        Q = cs_compress(Qraw); cs_spfree(Qraw);
#ifdef DEBUG_FILES
        print_cs_compress(Q,initpath+"Q.csv");
#endif
#ifdef DEBUG_PRIMARY
        cout << "Q is " << Q->m << " by " << Q->n << " with " << Q->nzmax << " entries\n\n" << std::flush;
#endif

        // identity matrix of dimension x (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing eye\n\n" << std::flush;
#endif
        cs *eyexraw = cs_spalloc(0,0,xqty,1,1);
        if (!eyexraw) cout << "ERROR: null eyexraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "eyexraw is " << eyexraw->m << " by " 
            << eyexraw->n << " with " << eyexraw->nzmax << " entries\n" << std::flush;
#endif
        for ( uint ii = 0 ; ii < xqty ; ii++ )
            cs_entry(eyexraw,ii,ii,1.0);
        eyex = cs_compress(eyexraw); cs_spfree(eyexraw);
#ifdef DEBUG_FILES
        print_cs_compress(eyex,initpath+"eyex.csv");
#endif

        // measurement covariance matrix (constant)
#ifdef DEBUG_PRIMARY
        cout << "Initializing R\n\n" << std::flush;
#endif
        cs *Rraw = cs_spalloc(0,0,zqty,1,1);
        if (!Rraw) cout << "ERROR: null Rraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Rraw is " << Rraw->m << " by " 
            << Rraw->n << " with " << Rraw->nzmax << " entries\n" << std::flush;
#endif
        for ( auto& zid : zary.zids )
            cs_entry(Rraw,zary.zidxs[zid],zary.zidxs[zid],zary.zsigs[zid]/1000000.0);
        R = cs_compress(Rraw); cs_spfree(Rraw);
#ifdef DEBUG_FILES
        print_cs_compress(R,initpath+"R.csv");
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
        // initial measurement vector [these actually don't need to be done here]
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
            state_fh << "\'"+node_name+"\'" << ( ++ctr < node_qty ? ',' : "\n" );
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

#ifdef DEBUG_SECONDARY
        cout << text << "\n\n" << std::flush;
#endif

#ifdef DEBUG_SECONDARY
        for ( auto& mobj : zary.mmrids ) {
            cout << mobj << " -> " << zary.mtypes[mobj] << "\n" << std::flush;
        }
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
#ifdef DEBUG_SECONDARY
//          cout << m.dump(2)+"\n" << std::flush;
//          cout << mmrid+"\n" << std::flush;
//          cout << "\t" + m_type + "\n" << std::flush;
#endif

            // Check for "PNV" measurement
            if ( !m_type.compare("PNV") ) {

                // update the voltage magnitude (in per-unit)
                string zid = mmrid+"_Vmag";
                double vmag_phys = m["magnitude"];
                // TODO: This uses vnom filled from OpenDSS values, but needs to use
                // GridLAB-D values
                zary.zvals[mmrid+"_Vmag"] = 
                    vmag_phys / abs(node_vnoms[zary.znode1s[zid]]);
                zary.znew[mmrid+"_Vmag"] = true;

#ifdef DEBUG_SECONDARY
                cout << mmrid+"_Vmag" << " -> " << zary.zvals[mmrid+"_Vmag"] << "\n" << std::flush;
#endif

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
        cout << "Estimating state ... \n" << std::flush;
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
#ifdef DEBUG_SECONDARY
        cout << jstate.dump(4).substr(200)+"\n" << std::flush;
#endif
        statePublisher->send(jstate.dump());

#ifdef DEBUG_FILES
        // write to file
        std::string simpath = "output/sim_" + simid + "/";
        state_fh.open(simpath+"vmag_per-unit.csv",ofstream::app);
        state_fh << timestamp << ',';
        uint ctr = 0;
        for ( auto& node_name : node_names ) {
            double vmag_pu = abs( Vpu[ node_idxs[node_name] ] );
            state_fh << vmag_pu << ( ++ctr < node_qty ? ',' : "\n" );
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
        cout << "xqty is " << xqty << "\n" << std::flush;
        cout << "zqty is " << zqty << "\n" << std::flush;
        cout << "F is " << F->m << " by " << F->n << " with " << F->nzmax << " entries\n" << std::flush;
        cout << "Q is " << Q->m << " by " << Q->n << " with " << Q->nzmax << " entries\n" << std::flush;
#endif

        // TODO: WE NEED TO HANDLE R-MASK IN HERE SOMEWHERE; ZARY NEEDS TO BE PERSISTENT

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
        
#ifdef DEBUG_PRIMARY
        cout << "prepx ... " << std::flush;
#endif
        cs *x; this->prep_x(x);

#ifdef DEBUG_PRIMARY
        cout << "x is " << x->m << " by " << x->n << 
            " with " << x->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(x,tspath+"x.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "P is " << P->m << " by " << P->n << 
            " with " << P->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(P,tspath+"P.csv");
#endif

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

#ifdef DEBUG_PRIMARY
        cout << "calc_J ... \n" << std::flush;
#endif
        cs *J; this->calc_J(J);
#ifdef DEBUG_PRIMARY
        cout << "J is " << J->m << " by " << J->n << 
            " with " << J->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(J,tspath+"J.csv");
#endif

        // --------------------------------------------------------------------
        // Predict Step
        // --------------------------------------------------------------------
        // -- compute x_predict = F*x | F=I (to improve performance, skip this)
        cs *xpre = cs_multiply(F,x); cs_spfree(x);
        if (!xpre) cout << "ERROR: null xpre\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "xpre is " << xpre->m << " by " << xpre->n << 
            " with " << xpre->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(xpre,tspath+"xpre.csv");
#endif

        // -- compute p_predict = F*P*F' + Q | F=I (can be simplified)

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
        cout << "Predict step complete.\n" << std::flush;
#endif
        
        // --------------------------------------------------------------------
        // Update Step
        // --------------------------------------------------------------------
        // -- compute y = z - h
        cs *yupd = cs_add(z,h,1,-1); cs_spfree(z); cs_spfree(h);
        if (!yupd) cout << "ERROR: null yupd\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "yupd is " << yupd->m << " by " << yupd->n << 
            " with " << yupd->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(yupd,tspath+"yupd.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "y updated\n" << std::flush;
#endif

        // -- compute S = J*P_predict*J' + R

        cs *S1 = cs_transpose(J,1);
        if (!S1) cout << "ERROR: null S1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "S1 is " << S1->m << " by " << S1->n << 
            " with " << S1->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(S1,tspath+"S1.csv");
#endif

        cs *S2 = cs_multiply(Ppre,S1); cs_spfree(S1);
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
//      cout << "Supd->nzmax is: " << Supd->nzmax << "\n";
//      cout << "Supd->p is: " << Supd->p << "\n";
//      for ( uint ii = 0 ; ii < Supd->m + 1 ; ii++ )
//          cout << "\t" << Supd->p[ii] << "\n";
//      cout << "Supd->i is: " << Supd->i << "\n";
//      for ( uint ii = 0 ; ii < Supd->nzmax ; ii++ )
//          cout << "\t" << Supd->i[ii] << ", ";
//      cout << "\n";
//      for ( uint ii = 0 ; ii < Supd->nzmax ; ii++ )
//          cout << "\t" << Supd->x[ii] << ", ";
//      cout << "\n";
#endif
#ifdef DEBUG_FILES
        print_cs_compress(Supd,tspath+"Supd.csv");
#endif

#ifdef DEBUG_PRIMARY
            cout << "in KLU block\n" << std::flush;
#endif

        double *rhs;

        try {
            // Initialize klusolve variables
            klu_symbolic *klusym;
            klu_numeric *klunum;
            klu_common klucom;
            if (!klu_defaults(&klucom)) throw "klu_defaults failed";

#ifdef DEBUG_PRIMARY
            cout << "klucom initialized.\n" << std::flush;
#endif
            klusym = klu_analyze(Supd->m,Supd->p,Supd->i,&klucom);
            if (!klusym) throw "klu_analyze failed";

#ifdef DEBUG_PRIMARY
            cout << "klusym initialized.\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            double startTime = getWallTime();
#endif
            klunum = klu_factor(Supd->p,Supd->i,Supd->x,klusym,&klucom);
#ifdef DEBUG_PRIMARY
            cout << "klu_factor time: " << getMinSec(getWallTime()-startTime)
                << "\n" << std::flush;
#endif
            if (!klunum) {
#ifdef DEBUG_PRIMARY
                cout << "Common->status is: " << klucom.status << "\n" << std::flush;
                if ( klucom.status == 1 ) cout << "\tKLU_SINGULAR\n" << std::flush;
#endif
                throw "klu_factor failed";
            }

#ifdef DEBUG_PRIMARY
            cout << "klunum initialized.\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            startTime = getWallTime();
#endif
            // initialize an identiy right-hand side
            rhs = new double[zqty*zqty];
            for ( uint ii = 0 ; ii < zqty*zqty ; ii++ )
                rhs[ii] = ii/zqty == ii%zqty ? 1 : 0;
            
#ifdef DEBUG_PRIMARY
            cout << "identity rhs time: " << getMinSec(getWallTime()-startTime)
                << "\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
            cout << "identity rhs created\n" << std::flush;
#endif

#ifdef DEBUG_PRIMARY
            startTime = getWallTime();
#endif
            klu_solve(klusym,klunum,Supd->m,Supd->n,rhs,&klucom);
#ifdef DEBUG_PRIMARY
            cout << "klu_solve time: " << getMinSec(getWallTime()-startTime)
                << "\n" << std::flush;
#endif
            if (klucom.status) {
#ifdef DEBUG_PRIMARY
                cout << "Common->status is: " << klucom.status << "\n" << std::flush;
#endif
                throw "klu_solve failed";
            }

#ifdef DEBUG_PRIMARY
            cout << "klu_solve complete\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
            cout << "klusym size to free: " << sizeof(klusym) << "\n" << std::flush;
            cout << "klunum size to free: " << sizeof(klunum) << "\n" << std::flush;
#endif
            // free klusym and klunum or major memory leak results
            klu_free_symbolic(&klusym, &klucom);
            klu_free_numeric(&klunum, &klucom);

        } catch (const char *msg) {
            cout << "KLU ERROR: " << msg << "\n" << std::flush;
            return;
        }
#ifdef DEBUG_PRIMARY
        cout << "left KLU block\n" << std::flush;
#endif
        cs_spfree(Supd);

        cs *K3raw = cs_spalloc(0,0,zqty*zqty,1,1);
        if (!K3raw) cout << "ERROR: null K3raw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "K3raw is " << K3raw->m << " by " 
            << K3raw->n << " with " << K3raw->nzmax << " entries\n" << std::flush;
#endif

        // convert the result to cs*
        for ( uint ii = 0 ; ii < zqty ; ii++ )
            for ( uint jj = 0 ; jj < zqty ; jj++ )
                if (rhs[ii+zqty*jj])
                    cs_entry(K3raw,ii,jj,rhs[ii+zqty*jj]);

#ifdef DEBUG_PRIMARY
        cout << "rhs copied to K3raw\n" << std::flush;
#endif
        delete rhs;

        // -- compute K = P_predict*J'*S^-1
        cs *K1 = cs_transpose(J,1);
        if (!K1) cout << "ERROR: null K1\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "K1 is " << K1->m << " by " << K1->n << 
            " with " << K1->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(K1,tspath+"K1.csv");
#endif

        cs *K2 = cs_multiply(Ppre,K1); cs_spfree(K1);
        if (!K2) cout << "ERROR: null K2\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "K2 is " << K2->m << " by " 
            << K2->n << " with " << K2->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_FILES
        print_cs_compress(K2,tspath+"K2.csv");
#endif

#ifdef DEBUG_FILES
//      print_cs_compress(K3raw,tspath+"K3raw.csv");
#endif

        cs *K3 = cs_compress(K3raw); cs_spfree(K3raw);
        if ( !K3 ) cout << "ERROR: K3 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "K3 is " << K3->m << " by " << K3->n << 
                " with " << K3->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(K3,tspath+"K3.csv");
#endif

        cs *Kupd = cs_multiply(K2,K3); cs_spfree(K2); cs_spfree(K3);
        if ( !Kupd ) cout << "ERROR: Kupd null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Kupd is " << Kupd->m << " by " << Kupd->n << 
                " with " << Kupd->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(Kupd,tspath+"Kupd.csv");
#endif

#ifdef DEBUG_PRIMARY
        cout << "K updated\n" << std::flush;
#endif

        // -- compute x_update = x_predict + K * y

        cs *x1 = cs_multiply(Kupd,yupd); cs_spfree(yupd);
        if ( !x1 ) cout << "ERROR: x1 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "x1 is " << x1->m << " by " << x1->n << 
                " with " << x1->nzmax << " entries\n" << std::flush;
#endif

#ifdef DEBUG_FILES
        print_cs_compress(x1,tspath+"x1.csv");
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
        cout << "x updated\n" << std::flush;
#endif

        // -- compute P_update = (I-K_update*J)*P_predict
        cs *P4 = cs_multiply(Kupd,J); cs_spfree(Kupd); cs_spfree(J);
        if ( !P4 ) cout << "ERROR: P4 null\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "P4 is " << P4->m << " by " << P4->n << 
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

#ifdef DEBUG_PRIMARY
        cout << "P updated\n" << std::flush;

        cout << "Update step complete.\n" << std::flush;
#endif

        // --------------------------------------------------------------------
        // Update persistant state (Vpu and A)
        // --------------------------------------------------------------------
#ifdef DEBUG_PRIMARY
        cout << "calling decompress_state_xupd\n" << std::flush;
#endif
        if (xupd) {
            decompress_state(xupd);
            // GDB 10/29/19 free xupd after last use
            cs_spfree(xupd);
        }
    }

    private:
    void decompress_state(cs *&x) {
        // copy state into vector (states, especially phase, can be 0)
        vector<double> xvec(x->m,0.0);
        for ( uint idx = 0 ; idx < x->nzmax ; idx++ )
            xvec[x->i[idx]] = x->x[idx];
        // update Vpu
        //  - NOTE: THIS WILL CHANGE IF SOURCE BUS STATES ARE NOT IN X
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

    private:
    void prep_x(cs *&x) {
        // Prepare x
        cs *xraw = cs_spalloc(xqty,1,xqty,1,1);
        if (!xraw) cout << "ERROR: null xraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "xraw is " << xraw->m << " by " 
            << xraw->n << " with " << xraw->nzmax << " entries\n" << std::flush;
#endif
        for ( auto& node_name : node_names ) {
            // Find the per-unit voltage of active node
            uint idx = node_idxs[node_name];
            complex<double> Vi = Vpu[idx];
            // Add the voltage magnitude to x
            uint vidx = idx - 1;
            if ( abs(Vi) > NEGL ) cs_entry(xraw,vidx,0,abs(Vi));
            // Add the voltage angle to x
            uint Tidx = node_qty + idx - 1;
            if ( arg(Vi) ) cs_entry(xraw,Tidx,0,arg(Vi));
        }
        x = cs_compress(xraw);
        cs_spfree(xraw);
    }

    private:
    void sample_z(cs *&z) {
        // measurements have been loaded from the sim output message to zary
        cs *zraw = cs_spalloc(zqty,1,zqty,1,1);
        if (!zraw) cout << "ERROR: null zraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "zraw is " << zraw->m << " by " 
            << zraw->n << " with " << zraw->nzmax << " entries\n" << std::flush;
#endif
        for ( auto& zid : zary.zids ) {
#ifdef DEBUG_SECONDARY
            cout << "\n" << zid << '[' << zary.zidxs[zid] << 
                    ']' << "\t" << zary.zvals[zid] << std::flush;
#endif
            if ( zary.zvals[zid] > NEGL || -zary.zvals[zid] > NEGL )
                cs_entry(zraw,zary.zidxs[zid],0,zary.zvals[zid]);
        }
#ifdef DEBUG_SECONDARY
        cout << "\n" << std::flush;
#endif
        z = cs_compress(zraw); 
        cs_spfree(zraw);
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
#ifdef DEBUG_DETAILS
            //cerr << "\t\t***SEDBG:set_n j==0 sets ai and aj to 0\n" << std::flush;
            // for per-unit Ybus, shunt admittance is the sum of row admittances
#endif
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
#ifdef DEBUG_DETAILS
            //cerr << "\t\t***SEDBG:set_n i: " << i << ", j: " << j << ", ai and aj initialized to 0\n" << std::flush;
#endif
            try {
                auto& Yrow = Ypu.at(i);
#ifdef DEBUG_DETAILS
                //for ( auto& rowpair : Yrow ) {
                //    cerr << "***SEDBG:set_n check Yrow: " << rowpair.first << ", " << rowpair.second << "\n" << std::flush;
                //}
#endif
                try {
#ifdef DEBUG_DETAILS
                    //cerr << "\t\t***SEDBG:set_n Yrow: " << Yrow << "\n" << std::flush;
#endif
                    Yij = Yrow.at(j);
                    // We know the nodes are coupled; check for Aij
                    // NOTE: A is never iterated over - we don't need at()
                    ai = 1;
#ifdef DEBUG_DETAILS
                    //cerr << "\t\t***SEDBG:set_n ai set to 1\n" << std::flush;
#endif
                    try {
                        auto Arow = A.at(i);
                        try {
                            ai = real(Arow.at(j));
#ifdef DEBUG_DETAILS
                            //cerr << "\t\t***SEDBG:set_n ai finalized to: " << ai << "\n" << std::flush;
#endif
                        } catch(...) {}
                    } catch(...) {}
                    // We know the nodes are coupled; check for Aji
                    // NOTE: A is never iterated over - we don't need at()
                    aj = 1;
#ifdef DEBUG_DETAILS
                    //cerr << "\t\t***SEDBG:set_n aj set to 1\n" << std::flush;
#endif
                    try {
                        auto Arow = A.at(j);
                        try {
                            aj = real(Arow.at(i));
#ifdef DEBUG_DETAILS
                            //cerr << "\t\t***SEDBG:set_n aj finalized to: " << aj << "\n" << std::flush;
#endif
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
        cs *hraw = cs_spalloc(zqty,1,zqty,1,1);
        if (!hraw) cout << "ERROR: null hraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "hraw is " << hraw->m << " by " 
            << hraw->n << " with " << hraw->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        double startTime = getWallTime();
#endif
#ifdef DEBUG_SECONDARY
        uint beat_ctr = 0, total_ctr = zary.zids.size();
#endif
        for ( auto& zid : zary.zids ) {
#ifdef DEBUG_DETAILS
            cerr << "***SEDBG:calc_h zid: " << zid << "\n" << std::flush;
#endif
#ifdef DEBUG_SECONDARY
            if ( ++beat_ctr % 100 == 0 )
                cout << "--- calc_h heartbeat - " << beat_ctr << ", " <<
                    getPerComp(beat_ctr, total_ctr) << ", " <<
                    getMinSec(getWallTime()-startTime) << " ---\n" << std::flush;
#endif

            uint zidx = zary.zidxs[zid];
            string ztype = zary.ztypes[zid];
            // Determine the type of z component
            if ( !ztype.compare("Pi") ) {
                // Real power injection into node i
                uint i = node_idxs[zary.znode1s[zid]];
#ifdef DEBUG_DETAILS
                cerr << "***SEDBG:calc_h Pi i: " << i << "\n" << std::flush;
#endif
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
#ifdef DEBUG_DETAILS
                            cerr << "\t***SEDBG:calc_h Pi j: " << j << ", vi: " << vi << ", vj: " << vj << ", ai: " << ai << ", aj: " << aj << ", g: " << g << ", b: " << b << ", T: " << T << ", Pi: " << Pi << "\n" << std::flush;
#endif
                        }
                    }
                    // Add the real power component flowing from i to 0
                    set_n(i,0);
                    Pi += vi*vi * g; // times cos(T) ????
#ifdef DEBUG_DETAILS
                    cerr << "***SEDBG:calc_h Pi post summation vi: " << vi << ", g: " << g << ", Pi: " << Pi << "\n" << std::flush;
#endif
                } catch(...) {}
                // Insert the measurement component
                if ( abs(Pi) > NEGL ) cs_entry(hraw,zidx,0,Pi);
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
                if ( abs(Qi) > NEGL ) cs_entry(hraw,zidx,0,Qi);
            }
            else if ( !zary.ztypes[zid].compare("aji" ) ) {
                // aji is a direct state measurement
            }
            else if ( !zary.ztypes[zid].compare("vi") ) {
                // vi is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
                if ( abs(Vpu[i]) > NEGL ) cs_entry(hraw,zidx,0,abs(Vpu[i]));
#ifdef DEBUG_DETAILS
                cerr << "***SEDBG:calc_h vi i: " << i << ", abs(Vpu[i]): " << abs(Vpu[i]) << "\n" << std::flush;
#endif
            }
            else if ( !zary.ztypes[zid].compare("Ti") ) {
                // Ti is a direct state measurement
                uint i = node_idxs[zary.znode1s[zid]];
                if ( arg(Vpu[i]) > NEGL ) cs_entry(hraw,zidx,0,arg(Vpu[i]));
            }
            else { 
                cout << "WARNING: Undefined measurement type " + ztype + "\n" << std::flush;
            }
        }
        h = cs_compress(hraw); cs_spfree(hraw);
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
        cs *Jraw = cs_spalloc(zqty,xqty,zqty*xqty,1,1);
        if (!Jraw) cout << "ERROR: null Jraw\n" << std::flush;
#ifdef DEBUG_PRIMARY
        else cout << "Jraw is " << Jraw->m << " by " 
            << Jraw->n << " with " << Jraw->nzmax << " entries\n" << std::flush;
#endif
#ifdef DEBUG_PRIMARY
        uint beat_ctr = 0;
        uint total_ctr = zary.zids.size();
#endif
        // loop over z
        for ( auto& zid : zary.zids ) {
#ifdef DEBUG_PRIMARY
            if ( ++beat_ctr % 100 == 0 ) 
                cout << "--- calc_J heartbeat - " << beat_ctr << ", " <<
                    getPerComp(beat_ctr, total_ctr) << ", " <<
                    getMinSec(getWallTime()-startTime) << " ---\n" << std::flush;
#endif
#ifdef DEBUG_DETAILS
            cerr << "***SEDBG:calc_J zid: " << zid << "\n" << std::flush;
#endif
            uint zidx = zary.zidxs[zid];
            string ztype = zary.ztypes[zid];
            uint i = node_idxs[zary.znode1s[zid]];

            // loop over voltage magnitude states
            for ( auto& node_name : node_names ) {
                uint vidx = node_idxs[node_name];
                uint xidx = vidx-1;
                // Computation of d/dv depends on the measurement type
                if ( !ztype.compare("Pi" ) ) {
                    if ( vidx == i ) {
#ifdef DEBUG_DETAILS
                        cerr << "***SEDBG:calc_J Pi magnitude vidx==i, i: " << i << "\n" << std::flush;
#endif
                        // --- compute dPi/dvi
                        double dP = 0;
                        // loop over adjacent nodes
                        try {
                            auto& Yrow = Ypu.at(i);
                            for ( auto& rowpair : Yrow ) {
                                uint j = rowpair.first;
                                if (j != vidx) {
                                    set_n(i,j);
                                    dP = dP + 2*vi/ai/ai * g - 
                                        vj/ai/aj * (g*cos(T) + b*sin(T));
#ifdef DEBUG_DETAILS
                                    cerr << "\t***SEDBG:calc_J Pi magnitude j: " << j << ", vi: " << vi << ", vj: " << vj << ", ai: " << ai << ", aj: " << aj << ", g: " << g << ", b: " << b << ", T: " << T << ", dP: " << dP << "\n" << std::flush;
#endif
                                }
                            }
                            // consider the reference node
                            set_n(i,0);
                            dP = dP + 2*vi * g;
#ifdef DEBUG_DETAILS
                            cerr << "***SEDBG:calc_J Pi magnitude post summation vi: " << vi << ", g: " << g << ", dP: " << dP << "\n" << std::flush;
#endif
                        } catch(...) {}
                        if ( abs(dP > NEGL ) ) cs_entry(Jraw,zidx,xidx,dP);
                    } else {
#ifdef DEBUG_DETAILS
                        cerr << "***SEDBG:calc_J Pi magnitude vidx!=i, i: " << i << ", vidx: " << vidx << "\n" << std::flush;
#endif
                        // --- compute dPi/dvj
                        double dP = 0;
                        uint j = vidx;
                        try {
                            auto& Yrow = Ypu.at(i);
                            complex<double> Yij = Yrow.at(j);

                            set_n(i,j);
                            dP = -1.0 * vi/ai/aj * (g*cos(T) + b*sin(T));
                        } catch(...) {}
#ifdef DEBUG_DETAILS
                        cerr << "\t***SEDBG:calc_J Pi magnitude j: " << j << ", vi: " << vi << ", ai: " << ai << ", aj: " << aj << ", g: " << g << ", b: " << b << ", T: " << T << ", dP: " << dP << "\n" << std::flush;
#endif
                        if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
                    }
                }
                else if ( !ztype.compare("Qi") ) {
                    if ( vidx == i ) {
                        // --- compute dQ/dvi
                        double dQ = 0;
                        // loop over adjacent nodes
                        try {
                            auto& Yrow = Ypu.at(i);
                            for ( auto& rowpair : Yrow ) {
                                uint j = rowpair.first;
                                if (j != vidx) {
                                    set_n(i,j);
                                    dQ = dQ - 2*vi/ai/ai * b - 
                                        vj/ai/aj * (g*sin(T) - b*cos(T));
                                }
                            }
                            // consider the reference node
                            set_n(i,0);
                            dQ = dQ - 2*vi*b;
                        } catch (...) {}
                        if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
                    } else {
                        // --- compute dQ/dvj
                        double dQ = 0;
                        uint j = vidx;
                        try {
                            auto& Yrow = Ypu.at(i);
                            complex<double> Yij = Yrow.at(j);

                            set_n(i,j);
                            dQ = -1.0 * vi/ai/aj * (g*sin(T) - b*cos(T));
                        } catch(...) {}
                        if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
                    }
                }
                else if ( !ztype.compare("aji") ) {
                    // daji/dv = 0
                }
                else if ( !ztype.compare("vi") ) {
                    if ( vidx == i ) {
                        // --- compute dvi/dvi
                        cs_entry(Jraw,zidx,xidx,1.0);
#ifdef DEBUG_DETAILS
                        cerr << "***SEDBG:calc_J vi i: " << i << ", J entry set to 1.0\n" << std::flush;
#endif
                    }
                }
                else if ( !ztype.compare("Ti") ) {
                    // dT/dv = 0
                }

                else {
                    cout << "WARNING: Undefined measurement type " + ztype + "\n" << std::flush;
                }
            }

            // loop over voltage phase states
            for ( auto& node_name : node_names ) {
                // IF ( NOT SOURCE NODE ) ???
                uint vidx = node_idxs[node_name];
                uint xidx = vidx-1 + node_qty; // THIS WOULD CHANGE IF SOURCES KNOWN
                if ( !ztype.compare("Pi") ) {
                    if ( vidx == i ) {
#ifdef DEBUG_DETAILS
                        cerr << "***SEDBG:calc_J Pi phase vidx==i, i: " << i << "\n" << std::flush;
#endif
                        // --- compute dPi/dTi
                        double dP = 0;
                        // loop over adjacent nodes
                        try {
                            auto &Yrow = Ypu.at(i);
                            for ( auto& rowpair : Yrow ) {
                                uint j = rowpair.first;
                                if (j != vidx) {
                                    set_n(i,j);
                                    dP = dP + vi*vj/ai/aj * (g*sin(T) - b*cos(T));
#ifdef DEBUG_DETAILS
                                    cerr << "\t***SEDBG:calc_J Pi phase j: " << j << ", vi: " << vi << ", vj: " << vj << ", ai: " << ai << ", aj: " << aj << ", g: " << g << ", b: " << b << ", T: " << T << ", dP: " << dP << "\n" << std::flush;
#endif
                                }
                            }
                            // reference node component is 0
                        } catch(...) {}
                        if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
                    } else {
#ifdef DEBUG_DETAILS
                        cerr << "***SEDBG:calc_J Pi phase vidx!=i, i: " << i << "\n" << std::flush;
#endif
                        // --- compute dP/dTj
                        double dP = 0;
                        uint j = vidx;
                        try {
                            auto& Yrow = Ypu.at(i);
                            complex<double> Yij = Yrow.at(j);

                            set_n(i,j);
                            dP = -1.0 * vi*vj/ai/aj * (g*sin(T) - b*cos(T));
                        } catch(...) {}
#ifdef DEBUG_DETAILS
                        cerr << "\t***SEDBG:calc_J Pi phase j: " << j << ", vi: " << vi << ", vj: " << vj << ", ai: " << ai << ", aj: " << aj << ", g: " << g << ", b: " << b << ", T: " << T << ", dP: " << dP << "\n" << std::flush;
#endif
                        if ( abs(dP) > NEGL ) cs_entry(Jraw,zidx,xidx,dP);
                    }
                }
                else if ( !ztype.compare("Qi") ) {
                    if ( vidx == i ) {
                        // compute dQi/dTi
                        double dQ = 0;
                        // loop over adjacent nodes
                        try {
                            auto& Yrow = Ypu.at(i);
                            for ( auto& rowpair : Yrow ) {
                                uint j = rowpair.first;
                                if (j != vidx) {
                                    set_n(i,j);
                                    dQ = dQ - vi*vj/ai/aj * (g*cos(T) + b*sin(T));
                                }
                            }
                            // reference component is 0
                        } catch(...) {}
                        if (abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
                    } else {
                        // --- compute dQ/dTj
                        double dQ = 0;
                        uint j = vidx;
                        try {
                            auto& Yrow = Ypu.at(i);
                            complex<double> Yij = Yrow.at(j);

                            set_n(i,j);
                            dQ = vi*vj/ai/aj * (g*cos(T) + b*sin(T));
                        } catch(...) {}
                        if ( abs(dQ) > NEGL ) cs_entry(Jraw,zidx,xidx,dQ);
                    }
                }
                else if ( !ztype.compare("ajj") ) {
                    // dajj/dT = 0
                }
                else if ( !ztype.compare("vi") ) {
                    // dvi/dT = 0
                }
                else if ( !ztype.compare("Ti") ) {
                    if ( vidx == i ) {
                        // --- compute dTi/dTi
                        cs_entry(Jraw,zidx,xidx,1.0);
                    }
                }
                else {
                    cout << "WARNING: Undefined measurement type " + ztype + "\n" << std::flush;
                }
            }

            // loop over regulator tap ratio states
            // --- LATER ---
            // -------------

        }
        J = cs_compress(Jraw); cs_spfree(Jraw);

#ifdef DEBUG_PRIMARY
        double endTime = getWallTime();
        cout << "calc_J wall clock execution time: " << getMinSec(endTime-startTime) << "\n\n" << std::flush;
#endif
    }
    
#ifdef DEBUG_FILES
    private:
    void print_cs_compress(cs *&a, const string &filename="cs.csv") {
        // First copy into a map
        unordered_map<uint,unordered_map<uint,double>> mat;
        for ( uint i = 0 ; i < a->n ; i++ ) {
//          cout << "in column " << i << " idx from " << a->p[i] << 
//                  " to " << a->p[i+1] << "\n" << std::flush;
            for ( uint j = a->p[i] ; j < a->p[i+1] ; j++ ) {
                mat[a->i[j]][i] = a->x[j];
            }
        }
        // write to file
        ofstream ofh;
        ofh << std::setprecision(16);
        ofh.open(filename,ofstream::out);
        cout << "writing " + filename + "\n\n"; << std::flush 
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

    private:
    string getPerComp(uint current, uint total) {
        return std::to_string((uint)(100.0 * current/total)) + "%";
    }

};
#endif

#endif
