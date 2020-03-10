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
└───(Prerequisite libraries--SuiteSparse, ActiveMQ-CPP, Json)
````

## Requirements 

1. Docker ce version 17.12 or better.  You can install this via the docker_install_ubuntu.sh script.  (note for mint you will need to modify the file to work with xenial rather than ubuntu generically)

2. Please clone the repository <https://github.com/GRIDAPPSD/gridappsd-docker> (refered to as gridappsd-docker repository).  The state estimator is distributed pre-built under the gridappsd-docker repository, but you may instead build the state estimator from source code from its own repository if you wish to modify it, run a different branch than master, or otherwise run it outside the gridappsd-docker container.

3. If you wish to run the state estimator provided with gridappsd-docker, follow the instructions in the following section, "Running state estimator from the container".

4. Alternatively, to build the state estimator from source code and then run that version from the command line, skip to the section titled "Building state estimator" below.

```` bash
~/git
└── gridappsd-docker
````

## Running state estimator from the container

1. Configure the simulation from the GRIDAPPSD platform web browser visualization.

2. Click on the "Service Configuration" tab and then click on the checkbox under the gridappsd-state-estimator section of the configuration user interface to specify that state estimator should be started with the simulation.

3. Click on the "Submit" button and then the play button to start the simulation.

4. The state estimator will process running simulation measurements producing state estimate messages for other applications.

The remainder of these instructions apply only when building the state estimator from source code and running that build from the command line.

## Building state estimator

Clone the repository <https://github.com/GRIDAPPSD/gridappsd-state-estimator> next to the gridappsd-docker repository (they should both have the same parent folder, assumed to be ~/git in docker-compose.yml)

```` bash
~/git
├── gridappsd-docker
└── gridappsd-state-estimator
````

Then the following two repositories should be cloned into the state-estimator directory under the gridappsd-state-estimator repository cloned above

````
	- https://github.com/GRIDAPPSD/SuiteSparse
	- https://github.com/GRIDAPPSD/json

````

The ActiveMQ C++ client library, ActiveMQ-CPP, should be downloaded from the URL below as a Unix source code distrubtion.  Both the 3.9.4 and 3.9.5 releases have been successfully used with state estimator.  The tar.gz or tar.bz2 distribution should be extracted under the state-estimator directory, the same location as the SuiteSparse and json repositories.

````
    - https://activemq.apache.org/components/cms/download

````

Building prerequisite libraries requires some other packages to be installed first.  The following apt-get install commands should install those packages if they are not already installed:

```` bash
sudo apt-get install cmake
sudo apt-get install m4
sudo apt-get install liblapack-dev libblas-dev
sudo apt-get install libapr1 libapr1-dev
````

From the state-estimator directory, run the following commands to build the prerequisite libraries and then the state-estimator executable:

```` bash
cd activemq-cpp-library-*
./configure
make
sudo make install

cd ../SuiteSparse
make LAPACK=-llapack BLAS=-lblas

cd ../state-estimator
make
````

The executable application will be placed in bin/state-estimator.  The Json distribution consists entirely of include files and therefore is not compiled separately from the application using it.

## Running state estimator from the command line

1. Edit the run-se.sh script in the state-estimator subdirectory of the top-level gridappsd-state-estimator repository, uncomment the appropriate SIMREQ variable for the model being run based on the comment at the end of each SIMREQ line denoting the corresponding model, and save changes.

2. Configure and start the simulation from the GRIDAPPSD platform web browser visualization, click on the "Simulation ID" value in the upper left corner of the simulation diagram to copy the value to the clipboard.

3. Invoke the script "./run-se.sh" from the command line with the Simulation ID value pasted from the clipboard as the command line argument to the script.

4. The state estimator will process running simulation measurements producing state estimate messages for other applications along with diagnostic log output to the terminal.

