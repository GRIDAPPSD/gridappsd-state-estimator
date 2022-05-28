#!/bin/bash
# ulimit -c unlimited # make sure core dump files are allowed to be generated

#SIMREQ={\"power_system_config\":{\"Line_name\":\"_5B816B93-7A5F-B64C-8460-47C17D6E4B0F\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":true}},{\"id\":\"gridappsd-sensor-simulator\",\"user_options\":{\"default-perunit-confidence-band\":0.02,\"simulate-all\":true,\"sensors-config\":{},\"default-normal-value\":100.0,\"random-seed\":0.0,\"default-aggregation-interval\":30.0,\"passthrough-if-not-specified\":false,\"default-perunit-drop-rate\":0.05}}]} # ieee13nodecktassets using sensors
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_5B816B93-7A5F-B64C-8460-47C17D6E4B0F\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # ieee13nodecktassets using simulation

#SIMREQ={\"power_system_config\":{\"Line_name\":\"_C1C3E687-6FFD-C753-582B-632A27E28507\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":true}},{\"id\":\"gridappsd-sensor-simulator\",\"user_options\":{\"default-perunit-confidence-band\":0.02,\"simulate-all\":true,\"sensors-config\":{},\"default-normal-value\":100.0,\"random-seed\":0.0,\"default-aggregation-interval\":30.0,\"passthrough-if-not-specified\":false,\"default-perunit-drop-rate\":0.05}}]} # ieee123 using sensors
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_C1C3E687-6FFD-C753-582B-632A27E28507\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # ieee123 using simulation

#SIMREQ={\"power_system_config\":{\"Line_name\":\"_AAE94E4A-2465-6F5E-37B1-3E72183A4E44\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":true}},{\"id\":\"gridappsd-sensor-simulator\",\"user_options\":{\"default-perunit-confidence-band\":0.02,\"simulate-all\":true,\"sensors-config\":{},\"default-normal-value\":100.0,\"random-seed\":0.0,\"default-aggregation-interval\":30.0,\"passthrough-if-not-specified\":false,\"default-perunit-drop-rate\":0.05}}]} # test9500new using sensors
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_AAE94E4A-2465-6F5E-37B1-3E72183A4E44\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # test9500new using simulation

#SIMREQ={\"power_system_config\":{\"Line_name\":\"_49AD8E07-3BF9-A4E2-CB8F-C3722F837B62\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # ieee13nodeckt using simulation

# no command line arguments means a test harness invocation so no simulation
# or plotter to invoke
if [ "$#" -gt 0 ]; then
    if [ -z "$SIMREQ" ]; then
    #   main.py invocation when sim_starter.py will start the simulation
        read -d "\n" SIMID SIMREQ <<< $(sim_starter/sim_starter.py $1)
    else
    #   main.py invocation when simulation is already started from platform viz
        SIMID=$1
    fi

    # comment out the following 5 lines to keep state-plotter from running
    pushd .
    cd ../../gridappsd-state-plotter/state-plotter
    #./state-plotter.py $SIMID "$SIMREQ" -all 2>&1 > spmagdbg.log &
    ./state-plotter.py $SIMID "$SIMREQ" -stats 2>&1 > spmagdbg.log &
    popd

    bin/state-estimator $SIMID "$SIMREQ" 2>&1 | tee sedbg.log
else
    bin/state-estimator-file 2>&1 | tee sedbg.log
fi

