#ifndef PLATFORMINTERFACECOMMON_HPP
#define PLATFORMINTERFACECOMMON_HPP

SSMAP ssmap_empty;

class PlatformInterfaceCommon {
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
    }

    virtual void fillVnom(const SLIST& node_names, SCMAP& node_vnoms)=0;

private:
};

#endif
