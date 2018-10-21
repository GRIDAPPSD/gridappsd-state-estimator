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

// SLIST holds the lists of node names and regulator names
#ifndef SLIST
#define SLIST std::list<std::string>
#endif

// SIMAP holds the one-indexed positions of nodes
#ifndef SIMAP
#define SIMAP std::unordered_map<std::string,unsigned int>
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




// This class listens for system state messages
class SELoopConsumer : public SEConsumer {
	protected:
	string simid;

	// system state
	private:
	cs *x, *P;		// state model
	cs *F, *Q;		// process model
	cs *z, *R;		// measurement model
	cs *h, *J;
	cs *eyex;		// identity matrix of dimension x
	int xqty;	// number of states and measurements

//	cs *x, *xpre, *x1, *xupd;						// state vector
//	cs *P, *Ppre, *P1, *P2, *P3, *P4, *P, *Pupd;	// state covariance
//	cs *y1, *yupd;									// residual vector
//	cs *S1, *S2, *S3, *Supd;						// residual covariance
//	cs *K1, *K2, *K3, *Kupd;						// gain matrix

	private:
	json jtext;		// object holding the input message
	json jstate;	// object holding the output message

	// system topology definition
	public:
	uint numns;		// number of nodes
	SLIST nnames;	// node names [list of strings]
	SIMAP nodemap;	// node positional indices [node->int]
	IMMAP Y;		// Ybus [node->[row->col]]

	public:
	SensorArray zary;
	uint zqty;		// number of measurements 

	private:
	SEProducer *statePublisher = NULL;

	public:
	SELoopConsumer(const string& brokerURI, 
				const string& username,
				const string& password,
				const string& target,
				const string& mode,
				const string& simid) {
		this->brokerURI = brokerURI;
		this->username = username;
		this->password = password;
		this->target = target;
		this->mode = mode;
		this->simid = simid;
	}

	private:
	virtual void init() {
		// set up the output message json object
		jstate["simulation_id"] = simid;

		// Construct the producer that will be used to publish the state estimate
		string sePubTopic = "goss.gridappsd.state-estimator.out."+simid;
		statePublisher = new SEProducer(brokerURI,username,password,sePubTopic,"topic");
		cout << "State Estimate Message Producer Constructed.\n";
		
		// --------------------------------------------------------------------
		// Initialize cs variables for state estimation
		// --------------------------------------------------------------------
		xqty = 2*numns;
		zqty = zary.zqty;
		// state transition matrix (constant)
		cs *Fraw = cs_spalloc(0,0,xqty,1,1);
		for ( int ii = 0 ; ii < xqty ; ii++ )
			cs_entry(Fraw,ii,ii,1);
		F = cs_compress(Fraw); cs_spfree(Fraw);
		// process covariance matrix (constant)
		cs *Qraw = cs_spalloc(0,0,xqty,1,1);
		for ( int ii = 0 ; ii < xqty ; ii++ )
			cs_entry(Qraw,ii,ii,0.04*sqrt(1.0/4));		// THIS MAY NEED TO CHANGE
		Q = cs_compress(Qraw); cs_spfree(Qraw);
		// identity matrix of dimension x (constant)
		cs *eyexraw = cs_spalloc(0,0,xqty,1,1);
		for ( int ii = 0 ; ii < xqty ; ii++ )
			cs_entry(eyexraw,ii,ii,1.0);
		eyex = cs_compress(eyexraw); cs_spfree(eyexraw);
		// measurement covariance matrix (constant)
		cs *Rraw = cs_spalloc(0,0,zqty,1,1);
		int idx = 0;
		for ( auto& measurement : zary.zids ) {
			cs_entry(Rraw,idx,idx,zary.zsigs[measurement]);
			idx++;
		} R = cs_compress(Rraw); cs_spfree(Rraw);
		// initial state vector
		
		// initial measurement vector [these actually don't need to be done here]
//		this->sample_z();
//		this->calc_h();
//		this->calc_J();
	}

	public:
	virtual void process(const string& text) {
		cout << "\nMessage of " << text.length() 
			<< " bytes recieved on measurement topic.\n";

		jtext = json::parse(text);
		int timestamp = jtext["message"]["timestamp"];
		cout << "\ttimestamp: " << timestamp << "\n";

		// load the measurements into mvals
		// WE ALSO NEED TO TRACK WHICH MEASUREMENTS ARE NEW
		for ( auto &measurement : jtext["message"]["measurements"] ) {
			string measurement_mrid = measurement["measurement_mrid"];
			// check the sensor type
			if ( !zary.mtypes[measurement_mrid].compare("PNV") ) {
				// update the voltage magnitude measurement
				double vmag = measurement["magnitude"];
				zary.zvals[measurement_mrid+"_Vmag"] = vmag;
				// update the voltage angle measurement
				double varg = measurement["angle"];
				// zary.zvals[meas_name+"_Varg"] = varg;
			} else
			if ( strcmp) {
			}
		}
		
		// prepare the output message object
		jstate["message"]["measurements"] = json::array();
		jstate["message"]["timestamp"] = jtext["message"]["timestamp"];


//		// iterate over measurements
//		int measctr = 0;
//		for ( auto itr = jtext["message"]["measurements"].begin() ;
//				itr != jtext["message"]["measurements"].end() ;
//				itr++ ) {
////			cout << '\t' << (*itr)["measurement_mrid"] << '\n';a
//			measctr++;
//			try {
//				(*itr).at("magnitude");
//				(*itr).at("angle");
//				jstate["message"]["measurements"].push_back(*itr);
//			} catch (json::out_of_range) {}
//		}

		
//		int statectr = 0;
//		for ( auto itr = jstate["message"]["measurements"].begin() ;
//				itr != jstate["message"]["measurements"].end() ;
//				itr++ ) {
//			statectr++;
//		}	

//		cout << jstate["message"]["measurements"].dump() << "\n";
//
//		cout << "\t" << measctr << " measurements and " 
//			<< statectr << " publishable states...\n";
//
//		cout << "\ttimestamp: " << timestamp << "\n";


		// Publish the message
		statePublisher->send(jstate.dump());



		// If the SE falls behind, messages are skipped...



		// When we recieve a message to terminate:
		if ( text == "stop" ) {
			cout << "TIME TO STOP!\n";
			statePublisher->close();
			doneLatch.countDown();
		}
	}

	private:
	void estimate(void) {
		// z, h, and J will be maintained external to this
		this->sample_z();
		this->calc_h();
		this->calc_J();

		// --------------------------------------------------------------------
		// Predict Step
		// --------------------------------------------------------------------
		// -- compute x_predict = F*x | F=I (to improve performance, skip this)
		cs *xpre = cs_multiply(F,x);
		// -- compute p_predict = F*P*F' + Q | F=I (can be simplified)
		cs *P1 = cs_transpose(F,1);
		cs *P2 = cs_multiply(P,P1); cs_spfree(P1);
		cs *P3 = cs_multiply(F,P2); cs_spfree(P2);
		cs *Ppre = cs_add(P3,Q,1,1); cs_spfree(P2);
		
		// --------------------------------------------------------------------
		// Update Step
		// --------------------------------------------------------------------
		// -- compute y = J*x_predict + z
		cs *y1 = cs_multiply(J,xpre);
		cs *yupd = cs_add(z,y1,1,-1); cs_spfree(y1);
		// -- compute S = J*P_predict*J' + R
		cs *S1 = cs_transpose(J,1);
		cs *S2 = cs_multiply(Ppre,S1); cs_spfree(S1);
		cs *S3 = cs_multiply(J,S2); cs_spfree(S2);
		cs *Supd = cs_add(R,S3,1,1); cs_spfree(S3);
		// -- compute K = P_predict*J'*S^-1
		cs *K1 = cs_transpose(J,1);
		cs *K2 = cs_multiply(Ppre,K1); cs_spfree(K1);
			// Initialize klusolve variables
			klu_symbolic *klusym;
			klu_numeric *klunum;
			klu_common klucom;
			if (!klu_defaults(&klucom)) throw "klu_defaults failed";
			klusym = klu_analyze(Supd->m,Supd->p,Supd->i,&klucom);
			if (!klusym) throw "klu_analyze failed";
			klunum = klu_factor(Supd->p,Supd->i,Supd->x,klusym,&klucom);
			if (!klunum) throw "klu_factor failed";
			// initialize an identiy right-hand side
			double *rhs = new double[zqty*zqty];
			for ( int ii = 0 ; ii < zqty*zqty ; ii++ )
				rhs[ii] = ii/zqty == ii*zqty ? 1 : 0;
			klu_solve(klusym,klunum,Supd->m,Supd->n,rhs,&klucom);
			// convert the result to cs*
			cs *K3raw = cs_spalloc(0,0,zqty*zqty,1,1);
			for ( int ii = 0 ; ii < zqty ; ii++ )
				for ( int jj = 0 ; jj < zqty ; jj++ )
					if (rhs[ii+zqty*jj])
						cs_entry(K3raw,ii,jj,rhs[ii+zqty*jj]);
			delete rhs;
		cs *K3 = cs_compress(K3raw); cs_spfree(K3raw);
		cs *Kupd = cs_multiply(K2,K3); cs_spfree(K2); cs_spfree(K3);
		// -- compute x_update = x_predict + K * y
		cs *x1 = cs_multiply(Kupd,yupd);
		cs *xupd = cs_add(xpre,x1,1,1); cs_spfree(x1);
		// -- compute P_update = (KYH+I)*P_predict
		cs *P4 = cs_multiply(Kupd,J);
		cs *P5 = cs_add(eyex,P4,1,-1); cs_spfree(P4);
		cs *Pupd = cs_multiply(P5,Ppre); cs_spfree(P5);

		// --------------------------------------------------------------------
		// Clean up
		// --------------------------------------------------------------------
		// update system state
		cs_spfree(xpre); cs_spfree(Ppre);
		cs_spfree(x); x = xupd; xupd = NULL;
		cs_spfree(P); P = Pupd; Pupd = NULL;
		// free residual
		cs_spfree(yupd); cs_spfree(Supd);
		// free gain matrix
		cs_spfree(Kupd);

	}

	private:
	void sample_z(void) {
		// measurements have been loaded from the sim output message to zary
		cs *zraw = cs_spalloc(0,0,zqty,1,1);
		int ctr = 0;
		for ( auto& zentry : zary.zids )
			cs_entry(zraw,ctr++,0,zary.zvals[zentry]);
		z = cs_compress(zraw); cs_spfree(zraw);
	}

	private:
	void calc_h(void) {
		// we are directly modifying a cs* as a cs_compressed cs*
		if ( z->nz > 0 ) throw "h must be maintained as a cs_compressed cs*";

	}

	private:
	void calc_J(void) {
		// measurements ahve been loaded from the sim output message to zary
	}
};

#endif
