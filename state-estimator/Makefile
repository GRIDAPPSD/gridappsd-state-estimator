
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

SGDIDAL_DEFINES_NOT_NEEDED=-DASIO_STANDALONE -DBOOST_DATE_TIME_NO_LIB -DCLI11_HAS_FILESYSTEM=0 -DHELICS_CXX_STATIC_DEFINE -DHELICS_ENABLE_WEBSERVER=1 -DHELICS_STATIC_CORE_LIBRARY -DTOML11_DISABLE_STD_FILESYSTEM -DUSE_BOOST_SPIRIT=1 -DUSE_STD_OPTIONAL=1 -DUSE_STD_STRING_VIEW=1
SGIDAL_DEFINES=

#HELICS=/home/d39842/helics/HELICS
#SGIDAL_INCLUDES_NOT_NEEDED=-I$(HELICS)/build/include -I$(HELICS)/ThirdParty/containers -I$(HELICS)/ThirdParty/concurrency -I$(HELICS)/ThirdParty/utilities -I$(HELICS)/build/src/helics -I$(HELICS)/ThirdParty/toml -I$(HELICS)/ThirdParty/units -I$(HELICS)/build/ThirdParty/units -I$(HELICS)/ThirdParty/json_cpp/src/lib_json/../../include -I$(HELICS)/build/ThirdParty/json_cpp/include/json -I$(HELICS)/ThirdParty/networking -I$(HELICS)/ThirdParty/networking/ThirdParty/NetIF -isystem $(HELICS)/ThirdParty -isystem $(HELICS)/ThirdParty/asio/asio/include -DNDEBUG -fPIE -pthread -pedantic -Wstrict-aliasing=1 -Wunreachable-code -Woverloaded-virtual -Wundef -Wcast-align -Wlogical-op -Wduplicated-cond -Wnull-dereference -Wimplicit-fallthrough=2 -Wno-psabi -Wno-deprecated-declarations -Wclass-memaccess -Wall -Wextra -Wshadow
#SGIDAL_INCLUDES=$(FILE_INCLUDES) -I$(HELICS)/build/helics_generated_includes -I$(HELICS)/src -I$(HELICS)/ThirdParty/utilities/gmlc/utilities -isystem $(HELICS)/ThirdParty -std=c++17
HELICS_INSTALL=/home/d39842/helics/install
SGIDAL_INCLUDES=$(FILE_INCLUDES) -I$(HELICS_INSTALL)/include -std=c++17

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

HELICS=/home/d39842/helics/HELICS
#SGIDAL_LIBS=$(FILE_LIBS) -DNDEBUG -Wl,-rpath,/usr/lib/x86_64-linux-gnu $(HELICS)/build/lib/libhelics_apps.a $(HELICS)/build/lib/libhelics_application_api.a $(HELICS)/build/lib/libhelics_network.a $(HELICS)/build/lib/libhelics_core.a $(HELICS)/build/lib/libhelics_common.a -lrt $(HELICS)/build/lib/libgmlc_utilities.a $(HELICS)/build/lib/libunits.a $(HELICS)/build/lib/libjsoncpp.a $(HELICS)/build/lib/libspdlog.a $(HELICS)/build/lib/libfmt.a /usr/lib/x86_64-linux-gnu/libzmq.so $(HELICS)/build/lib/libgmlc_networking.a -pthread
SGIDAL_LIBS=$(FILE_LIBS) $(HELICS)/build/lib/libhelics_apps.a $(HELICS)/build/lib/libhelics_application_api.a $(HELICS)/build/lib/libhelics_network.a $(HELICS)/build/lib/libhelics_core.a $(HELICS)/build/lib/libhelics_common.a -lrt $(HELICS)/build/lib/libgmlc_utilities.a $(HELICS)/build/lib/libunits.a $(HELICS)/build/lib/libjsoncpp.a $(HELICS)/build/lib/libspdlog.a $(HELICS)/build/lib/libfmt.a /usr/lib/x86_64-linux-gnu/libzmq.so $(HELICS)/build/lib/libgmlc_networking.a -pthread

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

all: bin/state-estimator bin/state-estimator-file bin/state-estimator-sgidal

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

sgidal bin/state-estimator-sgidal: obj/state-estimator-sgidal.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? $(SGIDAL_LIBS) -o bin/state-estimator-sgidal

obj/state-estimator-sgidal.o: src/state-estimator.cpp $(wildcard include/*.hpp)
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -std=c++11 -DSGIDAL_INTERFACE -c $(SGIDAL_DEFINES) $(SGIDAL_INCLUDES) $< -o $@

clean:
	rm -f obj/*
	rm -f bin/*
