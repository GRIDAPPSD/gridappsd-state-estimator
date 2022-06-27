#ifndef PLATFORMINTERFACEBASE_HPP
#define PLATFORMINTERFACEBASE_HPP

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

#include "SensorArray.hpp"

// Methods are documented after they are declared. For map data structures that
// must be populated in PlatformInterface implementations, the data type for
// map values is given in parentheses after the description. The definitions
// of the different data structures is given above.

class PlatformInterfaceBase {
public:
    PlatformInterfaceBase(int argc, char** argv, const double& sbase) {
        sbase_ref = &sbase;
    }
    // Performs any overall platform-specific initialization needed. E.g.,
    // initializing and connecting to the messaging system.
    //     double sbase: system base power used to provide numerical stability
    //                   in matrix computations. Value set in State Estimator
    //                   main function and is simply passed to the constructor.


    virtual void setupMeasurements()=0;
    // Performs any setup/initialization needed so that each call to
    // fillMeasurement() will provide the measurement data for a timestamp.
    // E.g., subscribe to messaging system for simulation output.


    virtual void fillTopo()=0;
    // Fills topology related data structures. You must populate these:
    //     SLIST node_names: list of node names (string)
    //     IMMAP Yphys: physical units admittance sparse matrix stored by
    //                  row and column indices. Symmetric matrix where
    //                  Yphys[i][j] = Yphys[j][i] (complex double)


    void fillTopology() {
        fillTopo();

        node_qty = 0;
        for ( auto& node_name : node_names ) {
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }
    }
    // Does not need to be implemented in platform-specific PlatformInterface
    // definitions. State Estimator calls this which first calls the
    // PlatformInterface::fillTopo method and then populates these for you:
    //     uint node_qty: number of nodes
    //     SIMAP node_idxs: index number for each node (uint)
    //     ISMAP node_name_lookup: node name for each index (string)


    virtual void fillVnoms()=0;
    // Fills nominal voltage data structure. You must populate this:
    //     SCMAP node_vnoms: nominal voltage for each node (complex double)


    virtual void fillSensors()=0;
    // Fills sensor-related data structures. The SensorArray structure is a
    // composite of other data structures that contains the information to
    // translate between nodes and measurments.  In addition to SensorArray,
    // a few other maps must be populated and a few are optional depending on
    // whether the platform identifies distinct types of measurements such as
    // regulator tap position and switch state changes. You must populate these:
    //     SensorArray Zary: composite of multiple maps (see documentation
    //                       below and SensorArray.hpp)
    //     IMDMAP Amat: regulator tap ratio matrix referenced by the primary
    //                  and regulator node indices (double)
    //     SSMAP regid_primnode: primary node name for each regulator id
    //                           (string)
    //     SSMAP regid_regnode: regulation/secondary node name for each
    //                          regulator id (string)
    //
    // You should populate these if distinct measurement types are identified
    // by the platform that include regulator tap position and switch state
    // changes:
    //     SSMAP mmrid_pos_type: type of equipment with position measurement
    //                           for each measurement id (mrid) in order to
    //                           distinguish the type of change. Recognized
    //                           values are "regulator_tap" and
    //                           "load_break_switch" (string)
    //     SSMAP switch_node1s: first node name paired with each switch (string)
    //     SSMAP switch_node2s: second node name paired with each switch
    //                          (string)
    //
    // For populating Zary there are also required and optional data structures
    // that must be populated. The required data structures are:
    //     SLIST Zary.zids: list of measurement component IDs, mrid+"_"+ztype
    //                      (string)
    //     SIMAP Zary.zidxs: measurement component index for each measurement
    //                       component ID (string)
    //     SSMAP Zary.ztypes: measurement component type for each measurement
    //                        component ID. Recognized values are "vi", "Ti",
    //                        "aji", "Pi", "Qi" (string)
    //     SSMAP Zary.znode1s: point node or from node name for flow
    //                         measurements for each measurement component
    //                         ID (string)
    //     SSMAP Zary.znode2s: point node or to node name for flow measurements
    //                         for each measurement component ID (string)
    //     SDMAP Zary.zvals: value of the latest measurement for each
    //                       measurement component ID (double)
    //     SDMAP Zary.zsigs: standard deviations of measurement value for each
    //                       measurement component ID (double)
    //     SBMAP Zary.zpseudos: flag indicating a pseudo-measurement for each
    //                          measurement component ID, true==pseudo (bool)
    //     SDMAP Zary.znomvals: nominal value of measurement for each
    //                          measurement component ID (double)
    // Like the previously mentioned optional data structures, Zary also has
    // data structures that only need to be populated if the platform
    // identifies distince measurement types.  These are:
    //     SLIST Zary.mmrids: list of measurement identifiers or mrids (string)
    //     SSMAP Zary.mtypes: measurement type for each mrid. Recognized values
    //                        are "PNV", "Pos", "VA" (string)
    //     SSMAP Zary.mnodes: node connected to each measurement mrid (string)
    //     SSMAP Zary.mcetypes: conducting equipment type for each mrid. Used
    //                          for VA measurements so only those of type
    //                          "EnergyConsumer" are processed (string)


    virtual bool fillMeasurement()=0;
    // Fills the measurement data structures from the data provided by the
    // platform.  E.g, from a message sent by the platform messaging system.
    // The following data structures must be populated:
    //     uint meas_timestamp: timestamp for measurement
    //     SLIST meas_mrids: unique measurement identifiers (mrids) for
    //                       timestamp (string)
    //     SDMAP meas_magnitudes: measurement magnitude values for given mrids
    //                            (double)
    //     SDMAP meas_angles: if available, measurement angle values for given
    //                        mrids (double)
    //     SDMAP meas_values: if available, regulator tap position or switch
    //                        state values for given mrids (double)


    virtual bool nextMeasurementWaiting()=0;
    // Returns true/false indicating whether there are additional measurements
    // that can be processed immediately. If true, State Estimator will
    // average voltage magnitudes for all waiting measurements before producing
    // a new state estimate, in effect catching up to current measurement if
    // producing the last state estimate resulted in falling behind. By
    // returning false a state estimate will be produced for every timestamp
    // even if there are waiting measurements. For reading measurement data
    // from a file, false should always be returned to avoid producing a single
    // estimate over all measurements. For processing simulator or field data,
    // hardwiring a false return value will result in estimates falling behind
    // the measurement data timestamp if the time to compute a state estimate
    // is more than the time between measurements.


    virtual void setupPublishing()=0;
    // Performs any setup/initialization needed so that publishEstimate()
    // is able to provide (publish as a message, write to file) state estimate
    // results for a timestamp. E.g., create messaging system topic needed
    // for publishing. If needed to publish estimates, you should populate:
    //     SSMAP node_bmrids: bus identifier for each node (string)
    //     SSMAP node_phs: phase (A/B/C/N/s1/s2) for each node (string)


    virtual void publishEstimate(const uint& timestamp,
        SDMAP& est_v, SDMAP& est_angle, SDMAP& est_vvar, SDMAP& est_anglevar,
        SDMAP& est_vmagpu, SDMAP& est_vargpu)=0;
    // Sends a full state estimate for a given timestamp to the disposition
    // established setupPublishing(). The platform interface implementation
    // for this method can choose to use any/all of the provided data
    // structures passed as arguments in order to output the estimate.
    // These data structures are populated by State Estimator prior to calling
    // publishEstimate() for each timestamp. The data available are:
    //     uint timestamp: timestamp associated with estimate
    //     SDMAP est_v: estimated magnitude in physical units for each node
    //                  (double)
    //     SDMAP est_angle: estimated angle in degrees for each node (double)
    //     SDMAP est_vvar: estimated variance for magnitude for each node
    //                     (double)
    //     SDMAP est_anglevar: estimated variance for angle for each node
    //                         (double)
    //     SDMAP est_vmagpu: estimated per-unit voltage magnitude for each node
    //                       (double)
    //     SDMAP est_vargpu: estimated per-unit voltage phase angle per node
    //                       (double)


    virtual string getOutputDir()=0;
    // Returns the directory name for diagnostic output files, typically
    // a unique name per invocation to avoid overwriting files.  E.g.,
    // simulation id.


    // Accessors used to retrieve platform interface data by State Estimator
    // code. Not needed in PlatformInterface code as the data can be referenced
    // directly with class variables.
    uint getsbase() {
        return *sbase_ref;
    }

    uint getnode_qty() {
        return node_qty;
    }

    SLIST getnode_names() {
        return node_names;
    }

    SIMAP getnode_idxs() {
        return node_idxs;
    }

    ISMAP getnode_name_lookup() {
        return node_name_lookup;
    }

    IMMAP getYphys() {
        return Yphys;
    }

    SCMAP getVnoms() {
        return node_vnoms;
    }

    SensorArray getZary() {
        return Zary;
    }

    IMDMAP getAmat() {
        return Amat;
    }

    SSMAP getregid_primnode() {
        return regid_primnode;
    }

    SSMAP getregid_regnode() {
        return regid_regnode;
    }

    SSMAP getmmrid_pos_type() {
        return mmrid_pos_type;
    }

    SSMAP getswitch_node1s() {
        return switch_node1s;
    }

    SSMAP getswitch_node2s() {
        return switch_node2s;
    }

    uint getmeas_timestamp() {
        return meas_timestamp;
    }

    SLIST getmeas_mrids() {
        return meas_mrids;
    }

    SDMAP getmeas_magnitudes() {
        return meas_magnitudes;
    }

    SDMAP getmeas_angles() {
        return meas_angles;
    }

    SDMAP getmeas_values() {
        return meas_values;
    }

    SSMAP getnode_bmrids() {
        return node_bmrids;
    }

    SSMAP getnode_phs() {
        return node_phs;
    }

protected:
    // system base power passed to constructor:
    const double* sbase_ref;

    // You must populate these in PlatformInterface::fillTopo:
    SLIST node_names;
    IMMAP Yphys;

    // These are populated for you in PlatformInterfaceBase::fillTopology:
    uint node_qty;
    SIMAP node_idxs;
    ISMAP node_name_lookup;

    // You must populate this in PlatformInterface::fillVnoms:
    SCMAP node_vnoms;

    // You must populate these in PlatformInterface::fillSensors:
    SensorArray Zary;
    IMDMAP Amat;
    SSMAP regid_primnode;
    SSMAP regid_regnode;
    // You may populate these in PlatformInterface::fillSensors if the
    // interface processes switch state and regulator tap position changes:
    SSMAP mmrid_pos_type;
    SSMAP switch_node1s;
    SSMAP switch_node2s;

    // These are populated for you to use in PlatformInterface:fillMeasurement:
    uint meas_timestamp;
    SLIST meas_mrids;
    SDMAP meas_magnitudes;
    SDMAP meas_angles;
    SDMAP meas_values;

    // You may populate these in PlatformInterface::setupPublishing depending
    // upon whether they are needed for publishing estimates
    SSMAP node_bmrids;
    SSMAP node_phs;
};

#endif
