FROM ubuntu:20.04

# ----------------------------------------------------
# INSTALL HELICS
# ----------------------------------------------------

RUN apt-get update \
    && export DEBIAN_FRONTEND="noninteractive" \
    && export TZ="America/Pacific" \
    && apt install -y \
       libboost-dev \
       libzmq5-dev \
       git \
       cmake-curses-gui \
       build-essential

RUN mkdir /build \
    && cd /build \
    && git clone https://github.com/GMLC-TDC/HELICS \
    && cd HELICS \
    && mkdir build \
    && cd build \
    && cmake -DHELICS_BUILD_CXX_SHARED_LIB=True ../ \
    && make \
    && make install

# ----------------------------------------------------
# INSTALL Activemq c++ extensions
# ----------------------------------------------------
RUN apt install -y m4 \
       wget \
       libaprutil1-dev
       
RUN cd /build \
    && wget http://archive.apache.org/dist/activemq/activemq-cpp/3.9.5/activemq-cpp-library-3.9.5-src.tar.gz \
    && tar -xzf activemq-cpp-library-3.9.5-src.tar.gz \
    && cd activemq-cpp-library-3.9.5 \
    && ./configure \
    && make \
    && make install 

RUN apt install -y liblapack-dev \
       libblas-dev \
       libssl-dev

# ----------------------------------------------------
# INSTALL State Estimator
# ----------------------------------------------------
RUN cd /build \
    && git clone --depth 1 --branch OEDISI.1.1.0 https://github.com/GRIDAPPSD/gridappsd-state-estimator \
    && cd gridappsd-state-estimator \
    && git clone https://github.com/GRIDAPPSD/SuiteSparse \
    && git clone https://github.com/GRIDAPPSD/json \
    && LD_LIBRARY_PATH=/build/gridappsd-state-estimator/SuiteSparse/lib/ make -C SuiteSparse LAPACK=-llapack BLAS=-lblas \
    && make -C state-estimator \
    && rm -rf .git SuiteSparse/.git json/.git 
