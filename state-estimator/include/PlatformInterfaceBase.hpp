#ifndef PLATFORMINTERFACEBASE_HPP
#define PLATFORMINTERFACEBASE_HPP

SSMAP ssmap_empty;

class PlatformInterfaceBase {
public:
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

        node_names_ref = &node_names;
        node_idxs_ref = &node_idxs;
    }

    virtual void fillVnom(SCMAP& node_vnoms)=0;

    virtual void fillMeasurements(SensorArray& zary,
        IMDMAP& Amat, SSMAP& regid_primnode_map, SSMAP& regid_regnode_map,
        SSMAP& mmrid_pos_type=ssmap_empty,
        SSMAP& switch_node1s=ssmap_empty, SSMAP& switch_node2s=ssmap_empty)=0;

protected:
    SLIST* node_names_ref;
    SIMAP* node_idxs_ref;
};

#endif
