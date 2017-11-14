# tnp_weight_events node

## Required

 Hardware
* Optoforce sensors

 OS
* Ubuntu 16.04 or 14.04

 Dependecies
* ROS Kinetic or Indigo
* Python 2.7

 ROS package
* tnp_task_manager
* tnp_weignt_events
* optoforce

## How to use

Run these commands (with the relevant autocompletions)

`$ roslaunch tnp_task_manager tnp_task_manager.launch`
`$ rosservice call /tnp_task_manager/setup_before_round`
`$ rosservice call /tnp_weight_events/get_ready_for_pick`
`$ rosservice call /tnp_weight_events/recognize_items`

## Debugging

After running `rosservice call /tnp_weight_events/get_ready_for_pick`, you can plot the signal with this:

rqt_plot /optoforce_bin_A/adjusted_weight /optoforce_bin_A/sensor_z_sum

## Authors
Akishige Yuguchi/ Chika Shiogama/ Rodrigo Elizalde Zapata/Dr. Gustavo Alfonso Garcia Ricardez/Felix v. Drigalski