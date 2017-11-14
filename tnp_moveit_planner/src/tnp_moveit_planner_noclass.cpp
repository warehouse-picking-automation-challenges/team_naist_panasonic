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

#include "tnp_moveit_planner/moveToCartesianPose.h"

bool moveToCartesianPose(tnp_moveit_planner::moveToCartesianPose::Request  &req,
         tnp_moveit_planner::moveToCartesianPose::Response &res)
{
  ROS_INFO_STREAM("Entered moveToCartesianPose service.");
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();
  
  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  movegroup_name = "manipulator";
  ee_link = "tnptool_suction_link_ee";
      
  int direction = 1;
  ROS_INFO_STREAM("Creating movegroup...");
  ros::Duration(1.0).sleep();
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  bool success_plan = false, motion_done = false, new_pose = false;

  ROS_INFO_STREAM("Waiting before moving to position...");
  ros::Duration(5.0).sleep(); // 5 seconds
  ROS_INFO_STREAM("Moving to position...");
  
  while (ros::ok()) {
    if (my_iiwa.getRobotIsConnected()) {
      
      command_cartesian_position = group.getCurrentPose(ee_link);  
      command_cartesian_position.pose = req.target_pose;
  
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
  return true;
}

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "tnp_moveit_planner");
  ros::NodeHandle nh("~");
  
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  ros::ServiceServer service = nh.advertiseService("moveToCartesianPose", moveToCartesianPose);

  // Without the AsyncSpinner, the iiwa_ros object does not seem to connect either here or in the service function.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();
  ros::Duration(3.0).sleep();
  if (my_iiwa.getRobotIsConnected()) 
  {
    ROS_INFO_STREAM("In the main, the iiwa_ros object connects.");
  }
  else {
    ROS_INFO_STREAM("In the main, the iiwa_ros object does not seem connected.");
  }
  
  ros::spin();      // This seems to be necessary to advertise the service?
}; 