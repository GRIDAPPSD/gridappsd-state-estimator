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
    ├── service
    │   ├── bin
    │   │   └── state-estimator.out
    │   ├── include
    │   ├── src
    │   ├── state-estimator.cpp
    │   └── build.sh
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

In order to add the state estimator to the container you will need to modify the docker-compose.yml file included in the gridappsd-docker repository.  Under the gridappsd service there is an example volumes leaf that is commented out.  Uncomment and modify these lines to add the path for the state estimator and conf file.  Adding these lines will mount the stat estimator on the container's filesystem when the container is started.

````
#    volumes:
#      - ~/git/gridappsd-state-estimator/state-estimator:/gridappsd/services/state-estimator
#      - ~/git/gridappsd-state-estimator/state-estimator/state-estimator.config:/gridappsd/services/state-estimator.config

    volumes:
       - ~/git/gridappsd-state-estimator/state-estimator:/gridappsd/services/state-estimator
       - ~/git/gridappsd-state-estimator/state-estimator/state-estimator.config:/gridappsd/services/state-estimator.config

````

## Building the state estimator

To build the state estimator, the following repositories should be cloned into ~/git

````
	- https://github.com/GRIDAPPSD/gridappsd-state-estimator
	- https://github.com/GRIDAPPSD/suitesparse-metis-for-windows
	- https://github.com/GRIDAPPSD/json

````
Navigate to the service directory and execute build.sh:

```` bash
cd gridasspd-state-estimator/state-estimator/service
./build.sh
````
The executable application will be placed in bin/state-estimator.out
