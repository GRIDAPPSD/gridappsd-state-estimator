#ifndef PLATFORMINTERFACEFILE_HPP
#define PLATFORMINTERFACEFILE_HPP

class PlatformInterface : public PlatformInterfaceCommon {
public:
    void fillTopologyMinimal(IMMAP& Yphys, SLIST& node_names) {

        string filename = FILE_INTERFACE_READ;
        filename += "/ysparse.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading ybus from test harness file: " << filename << "\n\n" << std::flush;
#endif
        std::ifstream ifs(filename);
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
        *selog << "Reading nodelist from test harness file: " << filename << "\n\n" << std::flush;
#endif
        ifs.open(filename);

        while ( getline(ifs, line) ) {
            // Extract the node name
            string node_name = regex_replace(line,regex("\""),"");
            // Store the node information
            node_names.push_back(node_name);
        }
        ifs.close();
    }


    void fillVnom(const SLIST& node_names, SCMAP& node_vnoms) {

#ifdef FILE_INTERFACE_VNOM
        string filename = FILE_INTERFACE_READ;
        filename += "/vnom.csv";
#ifdef DEBUG_PRIMARY
        *selog << "Reading vnom from test harness file: " << filename << "\n" << std::flush;
#endif
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            *selog << "\n*** ERROR: vnom file not found: " << filename << "\n\n" << std::flush;
            exit(0);
        }

        string line;
        getline(ifs, line);  // throwaway header line
        while (getline(ifs, line)) {
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
        for ( auto& node_name : node_names )
            node_vnoms[node_name] = 1;
#endif
    }

private:
};

#endif
