# tnp_serial_com
## Synopsis
Serial communication with something

## Code Example / How To Use
- Change permissions (if necessary)  
 $ sudo chmod a+rw /dev/ttyACM0  
- Launch  
 $ roslaunch tnp_serial_com TSC.launch   

## ROS Topic
- The device inputs from ROS(PC)  
-- */write* (std_msg/String)  
- The device outputs to ROS(PC)  
-- */read* (std_msg/String)  

## Other notes
Please rewrite TSC.launch (if necessary).  

## About port number
If we use *'/dev/ttyACMX'*, we will get confused.  
Please type and check  
 $ ls /dev/serial/by-id/  
and use *'/dev/serial/by-id/xxxxxxxxxxxxxxxx'*  

## Current usage example: End effector control
### ROS Topic
- send command to Arduino  
-- */tnp_arduino/write*  
- receive end effector's status from Arduino  
-- */tnp_arduino/read*  

### Command
The strings to be passed are in the format `set 1XXXXXX YYYY ZZ`.  
where `1XXXXXX` is controls the mechanisms:  
From the leftmost bit,  
* 1st bit: '1'
* 2nd bit: Gripper (Close'1' / Open'0')
* 3rd bit: Suction (On'1' / Off'0')
* 4th bit: Gripper Liner Actuator (Advance'1' / Retract'0')
* 5th bit: Suction Liner Actuator (Advance'1' / Retract'0')
* 6th bit: Drawer (Open'1' / Close'0' / Shake'2')  <
* 7th bit: Blower (On'1' / Off'0')

where `YYYY` is gripper parameter:
* Gripper open: Certain width (milimeters,0~110)
* Gripper close: Gripping force (millinewton?,0~1500)

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



## Current usage example: SR300 shutter control
### ROS Topic
- send command to Arduino  
-- */tnp_arduino_shutter/write*  

### Command
Please publish "set 0 XX" to the topic.  
XX is 0~31 (It is 5 bit binary number)  
- For example:  
-- "set 0 0"  -> All shutter open   
-- "set 0 31" -> All shutter close  
-- "set 0 1"  -> Shutter1 close and other shutter open  
-- "set 0 18" -> Shutter2 and shutter5 close and other shutter open  

# Extra:ROSPub.py  
ROSPub.py is easy publisher node.  
this node publish std_msg/String  

## How To Use
 $ rosrun tnp_serial_com ROSPub.py /XXXXX  
  (XXXXX is topic name)  
 enter the message, and the message is published to the topic.  

## Authors / Contributors

- Toyoshima Kenta (M2 2017)
- Yasunao Okazaki (Panasonic)
