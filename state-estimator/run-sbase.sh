#!/bin/bash
# wrapper for generating condition number values after convergence (by state
# estimator exiting after 20 or so estimate calls) vs. different sbase values
# requires state-estimator edits to limit the number of estimate calls and
# to interpret the 3rd command line argument as the sbase power

SIMID=$1
SIMREQ={\"power_system_config\":{\"Line_name\":\"_5B816B93-7A5F-B64C-8460-47C17D6E4B0F\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # ieee13nodecktassets using simulation
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_C1C3E687-6FFD-C753-582B-632A27E28507\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # ieee123 using simulation
#SIMREQ={\"power_system_config\":{\"Line_name\":\"_AAE94E4A-2465-6F5E-37B1-3E72183A4E44\"},\"service_configs\":[{\"id\":\"state-estimator\",\"user_options\":{\"use-sensors-for-estimates\":false}}]} # test9500new using simulation

rm sbase_data.log
touch sbase_data.log

#for spow in {0..18} # for 9500 node model
for spow in {-2..20} # for 13 and 123 node models
do
    echo -n 'sbase: 1e' >> sbase_data.log
    echo $spow >> sbase_data.log
    pushd .; cd ~/git/gridappsd-state-plotter/state-plotter
    ./state-plotter.py $SIMID $SIMREQ -stats 2>&1 | tee spmagdbg.out &
    popd
    bin/state-estimator $SIMID $SIMREQ $spow 2>&1 | tee sedbg.log
    grep "Supd condition number" sedbg.log | tail -n 1 >> sbase_data.log
    grep "mean magnitude" ~/git/gridappsd-state-plotter/state-plotter/spmagdbg.out | tail -n 1 >> sbase_data.log
    kill `pgrep -f state-plotter`
done

