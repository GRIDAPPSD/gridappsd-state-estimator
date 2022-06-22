#ifndef PLATFORMINTERFACEBASE_HPP
#define PLATFORMINTERFACEBASE_HPP

SSMAP ssmap_empty;

class PlatformInterfaceBase {
public:
    PlatformInterfaceBase(int, char**, const double& sbase) {
        sbase_ref = &sbase;
    }

    virtual void fillTopologyMinimal(IMMAP& Yphys, SLIST& node_names,
        SSMAP& node_bmrids=ssmap_empty, SSMAP& node_phs=ssmap_empty)=0;

    void fillTopology(IMMAP& Yphys, uint& node_qty, SLIST& node_names,
        SIMAP& node_idxs, ISMAP& node_name_lookup,
        SSMAP& node_bmrids=ssmap_empty, SSMAP& node_phs=ssmap_empty) {

        fillTopologyMinimal(Yphys, node_names, node_bmrids, node_phs);

        node_qty = 0;
        for ( auto& node_name : node_names ) {
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }

        Yphys_ref = &Yphys;
        node_qty_ref = &node_qty;
        node_names_ref = &node_names;
        node_idxs_ref = &node_idxs;
        node_name_lookup_ref = &node_name_lookup;
        node_bmrids_ref = &node_bmrids;
        node_phs_ref = &node_phs;
    }

    void fillVnom(SCMAP& node_vnoms) {
        node_vnoms_ref = &node_vnoms;
    }

    void fillSensors(SensorArray& zary,
        IMDMAP& Amat, SSMAP& regid_primnode, SSMAP& regid_regnode,
        SSMAP& mmrid_pos_type=ssmap_empty,
        SSMAP& switch_node1s=ssmap_empty, SSMAP& switch_node2s=ssmap_empty) {
        zary_ref = &zary;
        Amat_ref = &Amat;
        regid_primnode_ref = &regid_primnode;
        regid_regnode_ref = &regid_regnode;
        mmrid_pos_type_ref = &mmrid_pos_type;
        switch_node1s_ref = &switch_node1s;
        switch_node2s_ref = &switch_node2s;
    }

protected:
    const double* sbase_ref;
    IMMAP* Yphys_ref;
    const uint* node_qty_ref;
    SLIST* node_names_ref;
    SIMAP* node_idxs_ref;
    ISMAP* node_name_lookup_ref;
    SSMAP* node_bmrids_ref;
    SSMAP* node_phs_ref;
    SCMAP* node_vnoms_ref;
    SensorArray* zary_ref;
    IMDMAP* Amat_ref;
    SSMAP* regid_primnode_ref;
    SSMAP* regid_regnode_ref;
    SSMAP* mmrid_pos_type_ref;
    SSMAP* switch_node1s_ref;
    SSMAP* switch_node2s_ref;
};

#endif
