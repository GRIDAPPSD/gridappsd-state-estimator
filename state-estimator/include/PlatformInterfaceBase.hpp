#ifndef PLATFORMINTERFACEBASE_HPP
#define PLATFORMINTERFACEBASE_HPP

SSMAP ssmap_empty;

class PlatformInterfaceBase {
public:
    PlatformInterfaceBase(int, char**, const double& sbase) {
        sbase_ref = &sbase;
    }

    virtual void setupMeasurements()=0;

    virtual void fillTopo()=0;

    void fillTopology() {
        fillTopo();

        node_qty = 0;
        for ( auto& node_name : node_names ) {
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }
    }

    virtual void fillVnoms()=0;

    virtual void fillSensors()=0;

    uint getsbase() {
        return *sbase_ref;
    }

    SharedQueue<json>* getWorkQueue() {
        return &workQueue;
    }

    IMMAP getYphys() {
        return Yphys;
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

    SSMAP getnode_bmrids() {
        return node_bmrids;
    }

    SSMAP getnode_phs() {
        return node_phs;
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

protected:
    const double* sbase_ref;

    // PlatformInterface::setupMeasurements manages this:
    SharedQueue<json> workQueue;

    // PlatformInterface::fillTopology must populate these:
    IMMAP Yphys;
    SLIST node_names;
    // If PlatformInterface publishes results, PlatformInterface::fillTopology
    // must populate these:
    SSMAP node_bmrids;
    SSMAP node_phs;
    // PlatformInterfaceBase::fillTopology populates these:
    uint node_qty;
    SIMAP node_idxs;
    ISMAP node_name_lookup;

    // PlatformInterface::fillVnoms must populate this:
    SCMAP node_vnoms;

    // PlatformInterface::fillSensors must populate these:
    SensorArray Zary;
    IMDMAP Amat;
    SSMAP regid_primnode;
    SSMAP regid_regnode;
    // If PlatformInterface processes switch state and regulator tap position
    // changes, PlatformInterface::fillSensors must populate these:
    SSMAP mmrid_pos_type;
    SSMAP switch_node1s;
    SSMAP switch_node2s;
};

#endif
