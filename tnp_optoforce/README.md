# optoforce

Badly modified ROS driver for the [Optoforce sensor](http://optoforce.com/3dsensor/), so that it can be used as a scale.
Based on the Github project from the Shadow Hand company.

## Quickstart

There is always one optoforce node per forceplate. See `tnp_weight_events.launch` for details and how they are enumerated. You need to define the number of sensors and the correct USB ports in there.

You can call the service /optoforce_node_X/zero_sensors to zero the sensors.

## Calibration

To calibrate, use the services:
/optoforce_node_X/initialize_calibration
/optoforce_node_X/add_measurement
/optoforce_node_X/finish_calibration

Start up the node and initialize the calibration with an empty tote. 

To add a measurement, put a weight/item into the tote and call the service. When adding a measurement, a target value needs to be given. Use the inserted weight in grams.

After enough measurements have been taken, call finish_calibration. The calibration file for that sensor will be written automatically and overwrite the old one.

## Outputs/Topics

- 'weight' (the force in grams. msg type: std_msgs/Float64)
- `sensors_z` (the recorded z-value of all the sensors. Message type is std_msgs/Float64MultiArray)