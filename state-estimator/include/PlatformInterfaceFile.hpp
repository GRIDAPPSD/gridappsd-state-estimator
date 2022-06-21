#ifndef PLATFORMINTERFACEFILE_HPP
#define PLATFORMINTERFACEFILE_HPP

//class PlatformInterface : public PlatformInterfaceCommon {
class PlatformInterface {
public:
    void fillTopology(IMMAP& Yphys, uint& node_qty, SLIST& node_names,
        SIMAP& node_idxs, ISMAP& node_name_lookup) {

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

        node_qty = 0;
        for ( auto& node_name : node_names ) {
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }

        node_names_ref = &node_names;
        node_idxs_ref = &node_idxs;
    }


    void fillVnom(SCMAP& node_vnoms) {

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


    void fillMeasurements(SensorArray& zary, IMDMAP& Amat,
        SSMAP& regid_primnode_map, SSMAP& regid_regnode_map) {

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

            regid_primnode_map[regid] = primnode;
            regid_regnode_map[regid] = regnode;

            uint primidx = (*node_idxs_ref)[primnode];
            uint regidx = (*node_idxs_ref)[regnode];
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
            getline(lineStream, zid, ','); zary.zids.push_back(zid);
            zary.zidxs[zid] = zary.zqty++;
            zary.ztypes[zid] = cell;
            getline(lineStream, cell, ','); zary.znode1s[zid] = cell;
            getline(lineStream, cell, ','); zary.znode2s[zid] = cell;
            getline(lineStream, cell, ','); zary.zvals[zid] = std::stod(cell);
            getline(lineStream, cell, ','); zary.zsigs[zid] = std::stod(cell);
            getline(lineStream, cell, ','); zary.zpseudos[zid] = cell=="1";
            getline(lineStream, cell, ','); zary.znomvals[zid] =std::stod(cell);
        }
        ifs.close();
    }

private:
  SLIST* node_names_ref;
  SIMAP* node_idxs_ref;

};

#endif
