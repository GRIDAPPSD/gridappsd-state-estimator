#ifndef SENSORARRAY_HPP
#define SENSORARRAY_HPP

// store object names and hash names to sensor-related information

class SensorArray {
public:
    SLIST mmrids;   // list of measurement mrids
    SSMAP mtypes;   // measurement types (e.g. "PNV")
    SSMAP mnodes;   // mmrid to measurement node map
    SSMAP mcetypes; // mmrid to conducting equipment type map

    SLIST zids;     // list of measurement component IDs mmrid+"_"+ztype
    SIMAP zidxs;    // indexes for vectors like z and h
    SSMAP ztypes;   // measurement component types (e.g. "Vmag")
    SDMAP zsigs;    // standard deviations of measurements
    SSMAP znode1s;  // point node or from node for flow measurements
    SSMAP znode2s;  // point node or to node for flow measurements
    SDMAP zvals;    // value of the latest measurement
    SIMAP znews;    // counter for new measurements
    SIMAP ztimes;   // timestamp of last measurement
    SBMAP zpseudos; // flag indicating a pseudo-measrement
    SDMAP znomvals; // nominal value of measurement
    uint  zqty = 0; // number of measurement components

#ifdef COMPARE_INJ_MEAS
    SLIST injnodes;
#endif
};

#endif
