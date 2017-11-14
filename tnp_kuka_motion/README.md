= tnp_kuka_motion node =


The tnp_kuka_motion node should never cause a collision with the environment, as long as you only use these services:

- goToHome 				This is the position at the start of the round.
- goToTote				Moves gripper/suction to above the tote.
- goToBinNum 			This moves the end effector above the bin in question.
- goToLookIntoBinNum	This centers the camera above the bin.
- goToBoxNum 			Goes to above the origin of the box.
- putItemIntoBoxNum 	Moves to box and drops the item in it. See below for box sizes. (this should be in global parameters later)
- suckItem 				Starts above a box/bin and approaches the pose vector with the gripper.

TODO:

- graspItem 			
- lookIntoTote 			Centers camera above tote.


The boxes have their origin at the fixed corner. The center of the box is found via its size.

Box sizes:  
A1 – 25.4 x 17.5  
1AD – 34 x 24  
1A5 – 34 x 28  
1B2 – 39.5 x 33  
K3 – 49.5 x 33  


== Launch files ==

**tnp_kuka_motions_with_TF.launch** starts up tnp_kuka_motion and tnp_end_effector, as well as the robotStatePublisher that updates the state of the KUKA.

To troubleshoot TF and this:

rosrun tf tf_echo tnp_ee_camera_frame kuka_link_0
rosrun tf tf_monitor