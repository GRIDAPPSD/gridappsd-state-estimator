#ifndef PLATFORMINTERFACEGRIDAPPSD_HPP
#define PLATFORMINTERFACEGRIDAPPSD_HPP

//class PlatformInterface : public PlatformInterfaceCommon {
class PlatformInterface {
public:
    void fillTopologyMinimal(IMMAP& Yphys, SLIST& node_names) {

    }


    void fillVnom(const SLIST& node_names, SCMAP& node_vnoms) {

    }


    void fillMeasurements(SIMAP& node_idxs, SensorArray& zary,
        IMDMAP& Amat, SSMAP& regid_primnode_map, SSMAP& regid_regnode_map) {

    }

private:
};

#endif
