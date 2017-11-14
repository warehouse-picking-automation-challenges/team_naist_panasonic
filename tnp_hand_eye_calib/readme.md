This node offers automatic hand2eye calibration based on visp. (apt-get install ros-kinetic-vision-visp (http://wiki.ros.org/vision_visp?distro=kinetic))


# Calibration 

To use it, the SR300 end effector camera has to be running (normally on tnp-deep), via:

	roslaunch realsense_camera sr300_nodelet_rgbd.launch


To execute the calibration, launch:

	roslaunch tnp_hand_eye_calib tracklive_automatic.launch

And then launch the service for the calibration:

	rosservice call /tnp_hand_eye_calib/calibrateCamera...

It seems that you may have to then copy the results from the command line. Ask Okada about the details. 
The result of the calibration needs to be inserted as RPY values into tnp_tool/iiwa_tnptool_description/iiwa_tnptool.urdf.xacro, at the line: 

    <joint name="tnp_ee_camera_frame_joint" type="fixed">

# Calibration check

To confirm that the calibration is correct, launch via:

	roslaunch tnp_hand_eye_calib check_calibration.launch

This will publish the position of the marker it is seeing to /world_object
Afterwards, you can easily move the suction cup to that position by calling the service:

	rosservice call /tnp_hand_eye_calib/checkCalibration...

The default behavior moves the suction cup to the marker (make sure it is not on a hard surface.)



# Problems

There is about 1 cm error in x-direction. The reason may be low variation in x-direction, so we may have to adjust the motions.

# Example

[ INFO] [1495096475.297717844]: No.50 pose send!!
[ INFO] [1495096475.297755362]: 2) QUICK SERVICE:
[ INFO] [1495096475.352602196]: hand_camera: 
translation: 
  x: -0.1751
  y: 0.0450467
  z: 0.157739
rotation: 
  x: -0.00210599
  y: 0.030213
  z: -0.691422
  w: 0.721816