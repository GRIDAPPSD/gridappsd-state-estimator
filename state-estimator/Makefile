
SSPATH=../SuiteSparse

JSONINCPATH=../json/single_include/nlohmann
SSINCPATH=$(SSPATH)/include
AMQINCPATH=/usr/local/include/activemq-cpp-3.9.5
# APR is part of the Linux distro by default, hopefully here
APRINCPATH=/usr/include/apr-1.0
INCLUDES=-I$(JSONINCPATH) -I$(SSINCPATH) -I$(AMQINCPATH) -I$(APRINCPATH) -I./include

# Assumes static library linking or otherwise shared libs are all
# in $(SSPATH)/lib
SSCONFLIBPATH=$(SSPATH)/SuiteSparse_config
CSLIBPATH=$(SSPATH)/CXSparse/Lib
AMDLIBPATH=$(SSPATH)/AMD/Lib
BTFLIBPATH=$(SSPATH)/BTF/Lib
COLAMDLIBPATH=$(SSPATH)/COLAMD/Lib
KLULIBPATH=$(SSPATH)/KLU/Lib

CXXFLAGS=-Ofast

bin/state-estimator: obj/state-estimator.o
	mkdir -p bin
	g++ $(CXXFLAGS) $? -L$(SSCONFLIBPATH) -L$(CSLIBPATH) -L$(AMDLIBPATH) -L$(BTFLIBPATH) -L$(COLAMDLIBPATH) -L$(KLULIBPATH) -lklu -lamd -lbtf -lcolamd -lcxsparse -lsuitesparseconfig -lactivemq-cpp -lssl -o $@

obj/state-estimator.o: src/state-estimator.cpp include/GenericConsumer.hpp include/gridappsd_requests.hpp include/SEConsumer.hpp include/SELoopConsumer.hpp include/SensorArray.hpp include/SensorDefConsumer.hpp include/SEProducer.hpp include/sparql_queries_CIM100.hpp include/sparql_queries.hpp include/state_estimator_gridappsd.hpp include/state_estimator_util.hpp include/State.hpp include/TopoProcConsumer.hpp include/VnomConsumer.hpp
	mkdir -p obj
	g++ $(CXXFLAGS) -Wno-deprecated-declarations -c $(INCLUDES) $< -o $@

clean:
	rm -f obj/*
	rm -f bin/*
