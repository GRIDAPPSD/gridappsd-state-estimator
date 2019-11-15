# gridappsd-state-estimator

## Purpose

The state estimator service will produce and output the best available system state from measurements for use by other applications.

## State Estimator Service Layout

The following is the structure of the state estimator:

```` bash
.
├── README.md
├── LICENSE
└── state-estimator
    ├── include
    ├── src
    ├── bin
    ├── obj
    ├── state-estimator.cpp
    ├── Makefile
    └── state-estimator.config
````

## Requirements 

1. Docker ce version 17.12 or better.  You can install this via the docker_install_ubuntu.sh script.  (note for mint you will need to modify the file to work with xenial rather than ubuntu generically)

2. Please clone the repository <https://github.com/GRIDAPPSD/gridappsd-docker> (refered to as gridappsd-docker repository) next to this repository (they should both have the same parent folder, assumed to be ~/git in docker-compose.yml)

```` bash
~/git
├── gridappsd-docker
└── gridappsd-state-estimator	
````

## Adding the state estimator

In order to add the state estimator to the container you will need to modify the docker-compose.yml file included in the gridappsd-docker repository.  Under the gridappsd service there is an example volumes leaf that is commented out.  Uncomment and modify these lines to add the path for the state estimator and conf file.  Adding these lines will mount the state estimator on the container's filesystem when the container is started.

````
#    volumes:
#      - ~/git/gridappsd-state-estimator/state-estimator:/gridappsd/services/state-estimator
#      - ~/git/gridappsd-state-estimator/state-estimator/state-estimator.config:/gridappsd/services/state-estimator.config

    volumes:
       - ~/git/gridappsd-state-estimator/state-estimator:/gridappsd/services/state-estimator
       - ~/git/gridappsd-state-estimator/state-estimator/state-estimator.config:/gridappsd/services/state-estimator.config

````

## Building the state estimator

To build the state estimator, the following repositories should be cloned into the ~/git directory

````
	- https://github.com/GRIDAPPSD/gridappsd-state-estimator

````

Then the following two repositories should be cloned into the state-estimator directory under the git repository cloned above

````
	- https://github.com/GRIDAPPSD/SuiteSparse
	- https://github.com/GRIDAPPSD/json

````

Then ActiveMQ C++ client library, ActiveMQ-CPP, should be downloaded from the URL below as a Unix source code distrubtion.  This distribution should be extracted under the state-estimator directory, the same location as the SuiteSparse and json repositories.

````
    - https://activemq.apache.org/components/cms/download

````

Building prerequisite libraries requires some other packages to be installed first.  Specifically, SuiteSparse requires cmake, m4, liblapack-dev, and libblas-dev.  The following apt-get install commands should install those packages if they are not already installed:

```` bash
sudo apt-get install cmake
sudo apt-get install m4
sudo apt-get install liblapack-dev libblas-dev
````

From the state-estimator directory, run the following commands to build the prerequisite libraries and then the state-estimator executable:

```` bash
cd activemq-cpp-library-*
./configure
make
sudo make install

cd ../SuiteSparse
make static LAPACK=-llapack BLAS=-lblas

cd ..
make
````

The executable application will be placed in bin/state-estimator. Note that state-estimator has been run with ActiveMQ-CPP 3.9.4 and 3.9.5.  The Json distribution consists entirely of include files and therefore is not compiled separately from the application using it.

