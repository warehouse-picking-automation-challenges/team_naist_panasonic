#Task Manager

Execute node

`roslaunch tnp_task_manager tnp_task_manager`

We need to set up all nodes before the round starts with 

`rosservice call /tnp_task_manager/setupBeforeRound`

This does the following:
* Load ROS parameters
* Loads data from files (box_sizes, item_location_file, trainingItems)
* Loads data for orders information (order_file, box_n_size parameter)
* Sends items information to weight node
* Sends items location to weight node

Then we execute the corresponding task with one of the following:

`rosservice call /tnp_task_manager/pickTask`

`rosservice call /tnp_task_manager/rapidStowTask`