# GridAPPS-D State Estimator Test Harness Files

## File Overview

1. nodelist.csv: list of node names composed of the bus name separated by a period, ".", with the phase given by a number.

2. ysparse.csv: Ybus in physical units. First line is "Row,Col,G,B" header. Row and column values correspond with the row or line numbers for nodes in the nodelist.csv file.

3. vnom.csv: Nominal voltages for each node in nodelist.csv. First line is "Nodename,Mag,Arg" header. Nodename values are the same as the names in nodelist.csv. Arg is angle given in degrees.

4. regid.csv: For each regulator (transformer) given by a CIM mrid value, gives associated primary and regulator nodes. First line is "Regid,Primnode,Regnode" header. Primnode and Regnode values are the same as the names in nodelist.csv.

5. measurements.csv: Defines the measurements (both actual and psuedo) used for computing state estimates. First line is "ztype,zid,znode1,znode2,zval,zsig,zpseudo,znomval" header.

6. measurement_data.csv: Simulation generated measurement values at each timestamp for each measurement given in the measurements.csv file. First line is header with timestamp followed by each measurement identifier. Subsequent lines give the timestamp value followed by the value of each measurement given in the header. Includes psuedo-measurements that are constant over the simulation. This data is used for computing state estimates.

7. simulation_data.csv: Voltage magnitude and angle values produced by the GridLAB-D simulation. First line is header with simulation timestamp followed by "vmag" and "varg" listing of nodes as given in nodelist.csv. Subsequent lines give the timestamp value followed by vmag and varg pairs for each of the nodes listed in the header line. varg angles are given in radians. This data is used for plotting (results comparison) rather than for computing state estimates.

8. results_data.csv: Computed state estimates output by app. First line is header with timestamp followed by voltage magnitude values for each node, followed by voltage angle values for each node.

