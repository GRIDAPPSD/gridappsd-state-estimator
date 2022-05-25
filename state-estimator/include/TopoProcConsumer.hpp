#ifndef TOPOPROCCONSUMER_HPP
#define TOPOPROCCONSUMER_HPP
#ifdef GRIDAPPSD_INTERFACE

// Store node names in a linked list and hash node name to their position
// Iterate over the linked list to access all nodes or states
// Note that positions are one-indexed
// Store sensor names in a linked list and hash node names to various params

using namespace std;

// This class listens for the Y-bus message and constructs the topology
class TopoProcConsumer : public SEConsumer {
    
    // Need to figure out how to get structures out of here:
    // Pointers or set reference
    private:
    SLIST node_names;
    SIMAP node_idxs;
    ISMAP node_name_lookup;
    uint node_qty = 0;
    // to add a node:
    //    -- node_names.push_back(noden);
    //    -- node_idxs[noden] = ++node_qty;
    
    private:
    IMMAP Y;
    //    -- two-dimensional sparse matrix
    // To add an element:
    //    -- Y[i][j] = std::complex<double>(G,B);
//    // G, B, g, and b are derived from Y:
//    //    -- Gij = std::real(Y[i][j]);
//    //    -- Bij = std::imag(Y[i][j]);
//    //    -- gij = std::real(-1.0*Y[i][j]);
//    //    -- bij = std::imag(-1.0*Y[i][j]);
    
    public:
    TopoProcConsumer(const string& brokerURI, 
                const string& username,
                const string& password,
                const string& target,
                const string& mode) {
        this->brokerURI = brokerURI;
        this->username = username;
        this->password = password;
        this->target = target;
        this->mode = mode;
    }

    public:
    void fillTopo(uint& node_qty, SLIST& node_names, SIMAP& node_idxs,
            ISMAP& node_name_lookup, IMMAP& Y) {
        node_qty = this->node_qty;
        node_names = this->node_names;
        node_idxs = this->node_idxs;
        node_name_lookup = this->node_name_lookup;
        Y = this->Y;
    }
    
    public:
    virtual void process() {
        
        // --------------------------------------------------------------------
        // PARSE THE MESSAGE AND PROCESS THE TOPOLOGY
        // --------------------------------------------------------------------
        string line;
        bool firstline = true;

#ifdef DEBUG_PRIMARY
        *selog << "Received ybus message of " << text.length() << " bytes\n\n" << std::flush;
#endif

#ifdef FILE_INTERFACE_WRITE
        std::ofstream ofs("test_files/ysparse.csv", ofstream::out);
#endif

        json jtext = json::parse(text);

#ifdef DEBUG_PRIMARY
        *selog << "Parsing ybus -- " << std::flush;
#endif
        // This is actually a list of lines from ysparse
        json jlines_ysparse = jtext["data"]["yParse"];
        for ( auto& jline : jlines_ysparse ) {
            line = jline;
#ifdef FILE_INTERFACE_WRITE
            ofs << line << "\n";
#endif
            if (firstline) firstline = false;
            else {
                // split the line {Row,Col,G,B}
                std::stringstream lineStream(line);
                string cell;
                getline(lineStream, cell, ','); int i = stoi(cell);
                getline(lineStream, cell, ','); int j = stoi(cell);
                getline(lineStream, cell, ','); double G = stod(cell);
                getline(lineStream, cell, ','); double B = stod(cell);

                Y[i][j] = complex<double>(G,B);
                if ( i != j ) Y[j][i] = complex<double>(G,B);
            }
        }
#ifdef DEBUG_PRIMARY
        *selog << "complete.\n\n" << std::flush;
        *selog << "Parsing nodelist -- " << std::flush;
#endif
#ifdef FILE_INTERFACE_WRITE
        ofs.close();
        ofs.open("test_files/nodelist.csv", ofstream::out);
#endif

        json jlines_nodelist = jtext["data"]["nodeList"];
        for ( auto& jline : jlines_nodelist ) {
            line = jline;
            // Extract the node name
            string node_name = line.substr(1, line.size()-2);
#ifdef FILE_INTERFACE_WRITE
            ofs << node_name << "\n";
#endif
            // Store the node information
            node_names.push_back(node_name);
            node_idxs[node_name] = ++node_qty;
            node_name_lookup[node_qty] = node_name;
        }
#ifdef DEBUG_PRIMARY
        *selog << "complete.\n\n" << std::flush;
#endif
#ifdef FILE_INTERFACE_WRITE
        ofs.close();
#endif
//        // print
//        for ( auto& inode : node_names ) {
//            auto i = node_idxs[inode];
//            try {
//                auto row = Y.at(i);
//                for ( auto& jnode: node_names ) {
//                    auto j = node_idxs[jnode];
//                    try {
//                        complex<double> ycomp = row.at(j);
//                        *selog << "Y(" << i << "," << j << ") -> " << ycomp << "\n" << std::flush;
//                    } catch( const std::out_of_range& oor ) {}
//                }
//            } catch( const std::out_of_range& oor ) {}
//        }

        // --------------------------------------------------------------------
        // TOPOLOGY PROCESSING COMPLETE
        // --------------------------------------------------------------------
        // release latch
        doneLatch.countDown();
    }
};

#endif
#endif
