#!/bin/bash
ulimit -c unlimited # make sure core files will be generated
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_5B816B93-7A5F-B64C-8460-47C17D6E4B0F\"}} # ieee13nodecktassets
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_C1C3E687-6FFD-C753-582B-632A27E28507\"}}  # ieee123
SIMREQ={\"power_system_config\":{\"Line_name\":\"_AAE94E4A-2465-6F5E-37B1-3E72183A4E44\"}}  # test9500new
pushd .
#cd ../../gridappsd-state-plotter/state-plotter; ./state-plotter.py $1 $SIMREQ -all >/dev/null &
cd ../../gridappsd-state-plotter/state-plotter; ./state-plotter.py $1 $SIMREQ -20 >/dev/null &
popd
#bin/state-estimator $1 $SIMREQ >/dev/null 2>/dev/null
bin/state-estimator $1 $SIMREQ
