# Team NAIST-Panasonic (ARC 2017)

Software developed by the team NAIST-Panasonic at the Amazon Robotics Challenge 2017 (version 2017.7.31).
-----------------------------
## Packages
The ROS packages are organized in *online* (those used live during the competition), *offline* (those used for data collection, calibration, etc.) and *third-party*.

+ **Online**:
  - **tnp_task_manager**: Orchestrates all the nodes to complete the tasks.
  - **tnp_kuka_motion**: Controls the robot arm motion.
  - **tnp_moveit_planner**: Motion planner using MoveIt!
  - **tnp_tool**: End effector tool for motion planning.
  - **tnp_deep_vision**: Implements YOLOv2 for object recognition and grasping points estimation.
  - **tnp_feat_vision**: Object recognition based on color histogram, HOG, weight and volume.
  - **tnp_recognition_space**: Gets the RGB-D streams of four cameras.
  - **tnp_background_remover_bbox_extractor**: Removes background and extracts the bounding box.
  - **tnp_bbox_and_background_substractor**: Removes background and extracts the bounding box.
  - **tnp_end_effector**: Control and sensing for the gripper and suction fuctions of the end effector.
  - **tnp_serial_com**: Serial communication for the Arduinos.
  - **tnp_grasp_planner**: Calculates suckable surfaces and saves the octomaps of the storage system.
  - **tnp_weight_events**: Object recognition based on weight.
  - **tnp_optoforce**: Manages the OptoForce sensors.
  - **tnp_monitor**: GUI to visualize the whole system status, object recognition and grasping estimation results, voting results, task progress, etc.
  - **tnp_simple_state_machine**: Conveys the nodes' status to the tnp_monitor.
  - **tnp_msgs**: Messages to convey the status of some nodes for the tnp_monitor.
  - **tnp_led_control**: Controls the illumination.
  - **tnp_database_loader**: IO operations for json files.

+ **Offline**:
  - **tnp_image_taker**: Collects image data for training purposes.
  - **tnp_hand_eye_calib**: Hand-eye calibration for the end effector using multiple poses. 
  - **tnp_ee_calibration**: Hand-eye calibration for the end effector using a calibration cube.
  - **tnp_merge_capture_folder**: Merges two folders.
  - **tnp_scanner**: Implements a 3D scanner using the four cameras of the recognition space.
  - **tnp_recognition_space_calibration**: Calibrates the four cameras using a calibration cube with markers.
  - **tnp_recognition_space_configuration**: Configuration of the recognition space.
  - **tnp_simple_capture**: Takes a single frame using the four cameras of the recognition space.
  - **tnp_svm**: Implements a Support Vector Machine for the weight and volume.
  - **tnp_test_camera_orientation**: Simple test to verify the orientation of the cameras.

+ **Third-party**:
  - [iiwa_stack](https://github.com/SalvoVirga/iiwa_stack): ROS package for the KUKA LBR iiwa R820 developed by Salvo Virga (TUM).
  - [yolo_light](https://github.com/chrisgundling/yolo_light): ROS node implementing YOLOv2 for deep-learning-based object recognition.
  - [octomap_ros](https://github.com/OctoMap/octomap_ros): ROS package to provide conversion functions between ROS and OctoMap's native types.
  - [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins): RViz display plugins for visualizing octomap messages in ROS.
  - [phoxi_camera](http://www.photoneo.com/): Software for the time-of-flight camera developed by Photoneo.

The CAD models, desings and diagrams can be found in the `tnp_cads_and_diagrams` directory.

## Dependencies

The `docker` folder provides a `dockerfile` to create an image including all the dependencies necessary to run our system.

## License

The code in this repository is released under the [Apache License version 2.0](http://www.apache.org/licenses/LICENSE-2.0), with the exception of third-party code that keeps its original license.

## Authors

  - Gustavo A. Garcia R. (Captain)
  - Lotfi El Hafi
  - Felix von Drigalski
  - Wataru Yamazaki
  - Viktor Hoerig
  - Arnaud Delmotte
  - Akishige Yuguchi
  - Marcus Gall
  - Chika Shiogama
  - Kenta Toyoshima
  - Pedro Uriguen
  - Rodrigo Elizalde
  - Masaki Yamamoto
  - Yasunao Okazaki
  - Kazuo Inoue
  - Katsuhiko Asai
  - Ryutaro Futakuchi
  - Seigo Okada
  - Yusuke Kato
  - Pin-Chu Yang


