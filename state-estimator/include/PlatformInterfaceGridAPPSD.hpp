#ifndef PLATFORMINTERFACEGRIDAPPSD_HPP
#define PLATFORMINTERFACEGRIDAPPSD_HPP

//class PlatformInterface : public PlatformInterfaceCommon {
class PlatformInterface {
public:
    void fillTopology(IMMAP& Yphys, uint& node_qty, SLIST& node_names,
        SIMAP& node_idxs, ISMAP& node_name_lookup) {

    }


    void fillVnom(SCMAP& node_vnoms) {

    }


    void fillMeasurements(SensorArray& zary, IMDMAP& Amat,
        SSMAP& regid_primnode_map, SSMAP& regid_regnode_map) {

    }

private:
};

#endif
