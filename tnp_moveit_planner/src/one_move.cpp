/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

#include "ros/ros.h"
#include "iiwa_ros/iiwa_ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "MoveOnce");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();
  
  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "manipulator");
  nh.param<std::string>("ee_link", ee_link, "tnptool_suction_link_ee");
  
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int direction = 1;

  ros::Duration(5.0).sleep(); // 5 seconds
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  bool success_plan = false, motion_done = false, new_pose = false;

  ros::Duration(5.0).sleep(); // 5 seconds
  ROS_INFO_STREAM("Robot is not connected...");
  
  while (ros::ok()) {
    if (my_iiwa.getRobotIsConnected()) {
      
      command_cartesian_position = group.getCurrentPose(ee_link);  
      command_cartesian_position.pose.position.z -= direction * 0.10;
  
      group.setStartStateToCurrentState();
      group.setPoseTarget(command_cartesian_position, ee_link);
      success_plan = group.plan(myplan);
      if (success_plan) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        break;
      }
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }  
}; 