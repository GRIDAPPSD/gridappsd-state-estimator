
SSPATH=../SuiteSparse

JSONINCPATH=../json/single_include/nlohmann
SSINCPATH=$(SSPATH)/include
AMQINCPATH=/usr/local/include/activemq-cpp-3.9.5
# APR is part of the Linux distro by default, hopefully here
APRINCPATH=/usr/include/apr-1.0
INCLUDES=-I$(JSONINCPATH) -I$(SSINCPATH) -I$(AMQINCPATH) -I$(APRINCPATH) -I./include

# SparseSuite libraries
SSCONFLIBPATH=$(SSPATH)/SuiteSparse_config
CSLIBPATH=$(SSPATH)/CXSparse/Lib
AMDLIBPATH=$(SSPATH)/AMD/Lib
BTFLIBPATH=$(SSPATH)/BTF/Lib
COLAMDLIBPATH=$(SSPATH)/COLAMD/Lib
KLULIBPATH=$(SSPATH)/KLU/Lib
# use the first definition below for static libs, the second for shared libs
SSLIBPATHS=-L$(SSCONFLIBPATH) -L$(CSLIBPATH) -L$(AMDLIBPATH) -L$(BTFLIBPATH) -L$(COLAMDLIBPATH) -L$(KLULIBPATH)
#SSLIBPATHS=-L$(SSPATH)/lib
SSLIBS=-lklu -lamd -lbtf -lcolamd -lcxsparse -lsuitesparseconfig

CXXFLAGS=-Ofast

bin/state-estimator: obj/state-estimator.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? $(SSLIBPATHS) $(SSLIBS) -lactivemq-cpp -lstdc++ -lssl -o $@

obj/state-estimator.o: src/state-estimator.cpp $(wildcard include/*.hpp)
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -std=c++11 -c $(INCLUDES) $< -o $@

clean:
	rm -f obj/*
	rm -f bin/*
