# tnp_end_effector package

Version: 2017-07-31

## tnp_end_effector node

### How to Use  
$ roslaunch tnp_end_effector tnp_end_effector.launch

### Service
* /tnp_end_effector/set_ee_status  
-- GripperClose:  
--- true -> Gripper force control (close)  
--- false -> Gripper position control (Open)  
-- SuctionOn:  
--- true -> Suction On  
--- false -> Suction Off  
-- lin_act_gripper_extended:  
--- true -> Gripper Advance   
--- false -> Gripper Retract  
-- lin_act_suction_extended:  
--- true -> Suction Advance  
--- false -> Suction Retract  
-- gripper_control_parameter:  
--- 0~110 (milimeters, when position control)  
--- 0~1500 (millinewton?, when force_control)  
-- suction_force:  
--- 0~40 (-kPa)   


* /tnp_end_effector/blower  
-- setBlowerState:   
--- true -> On  
--- false -> Off  

* /tnp_end_effector/drawer  
-- setDrawerAction:  
--- 0 -> Close  
--- 1 -> Open  
--- 2 -> Shake  

* /tnp_end_effector/get_loggers  
* /tnp_end_effector/gripper  
-- gripper_control_method:  
--- position_control or 0 -> Position control (Open)  
--- force_control or 1 -> Force control (close)  
-- gripper_control_parameter:   
--- 0~110 (milimeters, when position control)  
--- 0~1500 (millinewton, when force_control)  

* /tnp_end_effector/lin_actuator_gripper  
-- setLinActuatorState:  
--- true -> Advance   
--- false -> Retract  

* /tnp_end_effector/lin_actuator_suction  
-- setLinActuatorState:  
--- true -> Advance  
--- false -> Retract  

* /tnp_end_effector/suction  
-- setSuctionState:  
--- true -> On  
--- false -> Off  
-- suction_force:  
--- 0~40 (-kPa)   


### Sending commands

This version uses a serial communication node called `serial_com_node` to send commands (strings) to the arduino.

The strings to be passed are in the format `set 1XXXXXX YYYY ZZ`.  
where `1XXXXXX` is controls the mechanisms:  
From the leftmost bit,  
* 1st bit: '1'
* 2nd bit: Gripper (Close'1' / Open'0')
* 3rd bit: Suction (On'1' / Off'0')
* 4th bit: Gripper Liner Actuator (Advance'1' / Retract'0')
* 5th bit: Suction Liner Actuator (Advance'1' / Retract'0')
* 6th bit: Drawer (Open'0' / Close'1' / Shake'2')  
* 7th bit: Blower (On'1' / Off'0')  

where `YYYY` is gripper parameter:
* Gripper open: Certain width (milimeters,0~110)
* Gripper close: Gripping force (millinewton,0~1500)

where `ZZ` is gripper parameter:
* Suction force (kPa, 0~40)

### Meaning of Hand Status Signal

Status 1XXXXXXXXXXX  
where `1XXXXXXXXXXX` is  
From the leftmost bit,  
* 1st bit: '1'
* 2nd bit: Gripper (1:Close / 0:Open)
* 3rd bit: Suction (1:On / 0:Off)
* 4th bit: Gripper Liner Actuator (1:Advance / 0:Retract)
* 5th bit: Suction Liner Actuator (1:Advance / 0:Retract)
* 6th bit: Drawer (1:Open / 0:Close)
* 7th bit: Blower (1:On / 0:Off)
* 8th bit: Grip task status (1:Grip Something / 0:Grip Nothing)
* 9th bit: Suction task status (1:Suction Something / 0:Suction Nothing)
* 10th bit: Gripper Contact Detect (1:Contact Something / 0:Contact Nothing)
* 11th bit: Suction Contact Detect (1:Contact Something / 0:Contact Nothing)
* 12th bit: Drawer stucked (1:Stucked / 0:Clear)

### Troubleshooting
If it has any problems, please contact Toyoshima and Okazaki

## tnp_shutters node

### Sending commands

This version uses a serial communication node called `serial_com_node` to send commands (strings) to the arduino.
In this node we use the arduino_shutters one.

The strings to be passed are in the format `set 0 X`, where `X` is an integer after converting a 5-bit number.

Each bit represents a camera:
b_4 b_3 b_2 b_1 b_0
E   L   C   B   R

where
* E: end efector
* L: left (from the robot perspective)
* C: center
* B: base (robot base)
* R: right (from the robot perspective)

The binary values meaning is
* 0: open
* 1: close

### Examples

From the "gripper" service definition file:

there are two control methods: force_control and position_control
force_control: closes the gripper using force control: 0 super weak!, 100 maximum force (can damage stuff)
position_control: closes the gripper to a certain width in millimeters (0 totally close, 110 fully open), use it before grasping an object (i.e., the space is too narrow)


rosservice call /tnp_end_effector/gripper "control_method:
  data: 'position_control'
control_parameter:
  data: 0" 

rosservice call /tnp_end_effector/gripper "control_method:
  data: 'force_control'
control_parameter:
  data: 0" 

