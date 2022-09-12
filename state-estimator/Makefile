
SSPATH=../SuiteSparse
SSINCPATH=$(SSPATH)/include
JSONINCPATH=../json/single_include/nlohmann

FILE_INCLUDES=-I$(SSINCPATH) -I./include

AMQINCPATH1=/gridappsd/include/activemq-cpp-3.9.4
AMQINCPATH2=/usr/local/include/activemq-cpp-3.9.4
AMQINCPATH3=/usr/local/include/activemq-cpp-3.9.5
# APR is part of the Linux distro by default, hopefully here
APRINCPATH=/usr/include/apr-1.0

GRIDAPPSD_INCLUDES=$(FILE_INCLUDES) -I$(JSONINCPATH) -I$(AMQINCPATH1) -I$(AMQINCPATH2) -I$(AMQINCPATH3) -I$(APRINCPATH)

HELICS=$(dir $(shell which helics_broker))..
GADAL_INCLUDES=$(FILE_INCLUDES) -I$(JSONINCPATH) -I$(HELICS)/include -std=c++17

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

FILE_LIBS=$(SSLIBPATHS) $(SSLIBS) -lstdc++

AMQLIBDIR1=/gridappsd/lib
AMQLIBDIR2=/usr/local/lib
AMQLIBPATH=-L$(AMQLIBDIR1) -L$(AMQLIBDIR2)

GRIDAPPSD_LIBS=$(FILE_LIBS) $(AMQLIBPATH) -lactivemq-cpp -lssl -Wl,-rpath=$(AMQLIBDIR1) -Wl,-rpath=$(AMQLIBDIR2)

GADAL_LIBS=$(FILE_LIBS) -L$(HELICS)/lib -lhelicscpp -lhelics

# Changed from -Ofast to -O2 on 7/30/20 because under an Ubuntu 20 VM with
# g++ version 9.3.0 -Ofast was giving bad numerical results in terms of
# intermittent very high Supd condition numbers and incorrect state estimates
# Then on 9/11/20, when adding the Rmat code for uncertainty based on time
# since last measurement, it happened again where it would give KLU errors
# right away every few times SE was started.  Just some very innocuous code
# adding zlast to zary and not even using it caused the issue.  So now -O
# seems to be the one that produces reliable results.  Benchmarking on the 9500
# node model shows no discernable loss in performance fortunately.
CXXFLAGS=-O
#CXXFLAGS=-O2
#CXXFLAGS=-Ofast

all: bin/state-estimator bin/state-estimator-file bin/state-estimator-gadal

bin/state-estimator: obj/state-estimator.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? $(GRIDAPPSD_LIBS) -o $@

obj/state-estimator.o: src/state-estimator.cpp $(wildcard include/*.hpp)
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -std=c++11 -DGRIDAPPSD_INTERFACE -c $(GRIDAPPSD_INCLUDES) $< -o $@

file bin/state-estimator-file: obj/state-estimator-file.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? $(FILE_LIBS) -o bin/state-estimator-file

obj/state-estimator-file.o: src/state-estimator.cpp $(wildcard include/*.hpp)
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -std=c++11 -DFILE_INTERFACE -c $(FILE_INCLUDES) $< -o $@

gadal bin/state-estimator-gadal: obj/state-estimator-gadal.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? $(GADAL_LIBS) -o bin/state-estimator-gadal

obj/state-estimator-gadal.o: src/state-estimator.cpp $(wildcard include/*.hpp)
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -std=c++11 -DGADAL_INTERFACE -c $(GADAL_INCLUDES) $< -o $@

clean:
	rm -f obj/*
	rm -f bin/*
