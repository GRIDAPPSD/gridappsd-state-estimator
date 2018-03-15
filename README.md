# gridappsd-state-estimator

## Purpose

The state estimator service will produce and output the best available system state from measurements for use by other applications.

## State Estimator Service Layout

The following is the recommended structure for applications working with gridappsd:

```` bash
.
├── README.md
├── LICENSE
└── state-estimator
    ├── service
    │   ├── bin
    │   │   └── state-estimator.out
    │   ├─- include
    │   ├── src
    │   ├── state-estimator.cpp
    │   └── build.sh
    └── state-estimator.config
````

# From Sample App

## Requirements 

1. Docker ce version 17.12 or better.  You can install this via the docker_install_ubuntu.sh script.  (note for mint you will need to modify the file to work with xenial rather than ubuntu generically)

1. Please clone the repository <https://github.com/GRIDAPPSD/gridappsd-docker> (refered to as gridappsd-docker repository) next to this repository (they should both have the same parent folder)

```` bash
.
├── gridappsd-docker
└── gridappsd-sample-app
````

## Adding your application

In order to add your application to the container you will need to modify the docker-compose.yml file included in the gridappsd-docker repository.  Under the gridappsd service there is an example volumes leaf that is commented out.  Uncomment and modify these lines to add the path for your application and conf file.  Adding these lines will mount the application on the container's filesystem when the container is started.

````
#    volumes:
#      - ~/git/gridappsd-sample-app/sample_app:/gridappsd/applications/sample_app
#      - ~/git/gridappsd-sample-app/sample_app/sample_app.config:/gridappsd/applications/sample_app.config

    volumes:
      - ~/git/[my_app_directory]/[my_app]:/gridappsd/applications/[my_app]
      - ~/git/[my_app_directory]/[my_app]/[my_app.config]:/gridappsd/applications/[my_app.config]

````

## Debugging your python applications

### PyCharm (In Progress)

### PyDev (In Progress

### Visual Studio Code (In Progress)

### Command line

In gridappsd we include a python package called remote_pdb.  This is the only BSD licensed product that we have found to be able to allow us to remotely debug our python based applicatoins.  To use it we need a telnet client and we need to modify the sample-applicatoin to break at a trace_point.

```` python

# The container allows ports between 8001-9000 to be used as anything
# you like.
from remote_pdb import RemotePdb

# Add the following where you would like to break within the python app.
RemotePdb('0.0.0.0', 8888).set_trace()

````

Connect to the remote debugging session via telnet 

```` bash

telnet '127.0.0.1' 8888
````

Once connected you can use any of the pdb commands to move to the next line set other breakpoints etc.  Documentation of those commands can be found at <https://docs.python.org/2/library/pdb.html#debugger-commands>.

To exit the telnet shell type 'quit' and press enter.
