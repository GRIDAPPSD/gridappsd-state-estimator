#ifndef PLATFORMINTERFACEFILE_HPP
#define PLATFORMINTERFACEFILE_HPP

#include <iomanip> // std::setprecision

class PlatformInterface : public PlatformInterfaceBase {
public:
    PlatformInterface(int argc, char** argv, const double& sbase) : PlatformInterfaceBase(argc, argv, sbase) {
    }


    void setupMeasurements() {
        string filename = FILE_INTERFACE_READ;
        filename += "/measurement_data.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading measurements from test harness file: " << filename << "\n\n" << std::flush;
#endif
        meas_fh.open(filename);

        // header line is an ordered list of zids
        // parse measurement file header to get the zids into an STL vector
        string meas_line;
        getline(meas_fh, meas_line);
        string cell;
        std::stringstream headerStream(meas_line);
        getline(headerStream, cell, ','); // throwaway timestamp header token
        while ( getline(headerStream, cell, ',') )
            meas_zids.push_back(cell);
    }


    void fillTopo() {
        string filename = FILE_INTERFACE_READ;
        filename += "/ysparse.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading ybus from test harness file: " << filename <<
            "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: ysparse file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line);  // throwaway header line
        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell;
            getline(lineStream, cell, ','); int i = std::stoi(cell);
            getline(lineStream, cell, ','); int j = std::stoi(cell);
            getline(lineStream, cell, ','); double G = std::stod(cell);
            getline(lineStream, cell, ','); double B = std::stod(cell);

            Yphys[i][j] = complex<double>(G,B);
            if ( i != j ) Yphys[j][i] = complex<double>(G,B);
        }
        ifs.close();

        filename = FILE_INTERFACE_READ;
        filename += "/nodelist.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading nodelist from test harness file: " << filename <<
            "\n\n" << std::flush;
#endif
        ifs.open(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: nodelist file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        while ( getline(ifs, line) ) {
            // Extract the node name
            string node_name = regex_replace(line,regex("\""),"");
            // Store the node information
            node_names.push_back(node_name);
        }
        ifs.close();
    }


    void fillVnoms() {
#ifdef FILE_INTERFACE_VNOM
        string filename = FILE_INTERFACE_READ;
        filename += "/vnom.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading vnom from test harness file: " << filename <<
            "\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: vnom file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line);  // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string node, cell;
            getline(lineStream, node, ',');
            getline(lineStream, cell, ','); double mag = std::stod(cell);
            getline(lineStream, cell, ','); double arg = std::stod(cell);
            double vre = mag * cos( arg * PI/180.0 );
            double vim = mag * sin( arg * PI/180.0 );
            complex<double> vnom = complex<double>(vre,vim);
            node_vnoms[node] = vnom;
        }
        ifs.close();
#else
        for ( auto& node_name : *node_names_ref )
            node_vnoms[node_name] = 1;
#endif
    }


    void fillSensors() {
        string filename = FILE_INTERFACE_READ;
        filename += "/regid.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading regulator mappings from test harness file: " <<
            filename << "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: regid file not found: " << filename <<
                "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line); // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string regid, primnode, regnode;
            getline(lineStream, regid, ',');
            getline(lineStream, primnode, ',');
            getline(lineStream, regnode, ',');

            regid_primnode[regid] = primnode;
            regid_regnode[regid] = regnode;

            uint primidx = node_idxs[primnode];
            uint regidx = node_idxs[regnode];
            // initialize the A matrix
            Amat[primidx][regidx] = 1; // this will change
            Amat[regidx][primidx] = 1; // this stays unity and may not be needed
        }
        ifs.close();

        // For the file interface, file is read for all measurements so no need
        // to do anything for pseudo-measurements as with SensorDefConsumer
        filename = FILE_INTERFACE_READ;
        filename += "/measurements.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading sensor measurements from test harness file: " <<
            filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: measurements file not found: " <<
                filename << "\n\n" << std::flush;
            exit(0);
        }

        getline(ifs, line); // throwaway header line

        while ( getline(ifs, line) ) {
            std::stringstream lineStream(line);
            string cell, zid;
            getline(lineStream, cell, ',');
            getline(lineStream, zid, ','); Zary.zids.push_back(zid);
            Zary.zidxs[zid] = Zary.zqty++;
            Zary.ztypes[zid] = cell;
            getline(lineStream, cell, ','); Zary.znode1s[zid] = cell;
            getline(lineStream, cell, ','); Zary.znode2s[zid] = cell;
            getline(lineStream, cell, ','); Zary.zvals[zid] = std::stod(cell);
            getline(lineStream, cell, ','); Zary.zsigs[zid] = std::stod(cell);
            getline(lineStream, cell, ','); Zary.zpseudos[zid] = cell=="1";
            getline(lineStream, cell, ','); Zary.znomvals[zid] =std::stod(cell);
        }
        ifs.close();
    }


    bool fillMeasurement() {
        bool ret = true;

        meas_timestamp = 0;
        meas_mrids.clear();
        meas_magnitudes.clear();

        string meas_line;
        if ( getline(meas_fh, meas_line) ) {
            std::stringstream lineStream(meas_line);
            string cell, zid;
            getline(lineStream, cell, ',');
            double doubletime = stod(cell);
            meas_timestamp = (uint)doubletime;

            uint idx = 0;
            while ( getline(lineStream, cell, ',') ) {
                zid = meas_zids[idx++];
                meas_mrids.push_back(zid);
                meas_magnitudes[zid] = stod(cell);
            }
        } else {
#ifdef DEBUG_PRIMARY
            *selog << "Reached end of measurement_data.csv file, normal exit\n" << std::flush;
#endif
            ret = false;
        }

        return ret;
    }


    bool nextMeasurementWaiting() {
        // always returning false tells the SE work loop to complete an
        // estimate for every measurement
        return false;
    }


    void setupPublishing() {
        string filename = FILE_INTERFACE_READ;
        filename += "/results_data.csv";
        est_fh.open(filename, std::ofstream::trunc);
        est_fh << "timestamp,";

        for ( auto& node_name : node_names )
            est_fh << "vmag_"+node_name+",";
        uint nctr = 0;
        for ( auto& node_name : node_names )
            est_fh << "varg_"+node_name << ( ++nctr < node_qty ? "," : "\n" );

        est_fh.close();
    }


    void publishEstimate(const uint& timestamp,
        SDMAP&, SDMAP&, SDMAP&, SDMAP&, SDMAP& est_vmagpu, SDMAP& est_vargpu) {

        string filename = FILE_INTERFACE_READ;
        filename += "/results_data.csv";
        est_fh.open(filename, std::ofstream::app);

        est_fh << timestamp << ',';
        est_fh << std::fixed;
        est_fh << std::setprecision(10);

        for ( auto& node_name : node_names )
            est_fh << est_vmagpu[node_name] << ",";
        uint nctr = 0;
        for ( auto& node_name : node_names )
            est_fh << est_vargpu[node_name] << ( ++nctr < node_qty ? "," : "\n" );

        est_fh.close();
    }


    std::vector<string> getZids() {
        return meas_zids;
    }

private:
    std::ifstream meas_fh;
    std::ofstream est_fh;
    std::vector<string> meas_zids;
};

#endif
