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
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanRequest.h>

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "Cartesian move");
  ros::NodeHandle nh("~");
  ros::ServiceClient motion_plann_request_service_;

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();

  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "suction");
  nh.param<std::string>("ee_link", ee_link, "tnptool_suction_link_ee");
  
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int direction = 1;

  ros::Duration(3.0).sleep(); // 3 seconds

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  
  bool success_plan = false, motion_done = false, new_pose = false;
  geometry_msgs::Pose p1;

  ros::Duration(3.0).sleep(); // 3 seconds
  while (ros::ok()) {
    if (my_iiwa.getRobotIsConnected()) {
      std::vector<geometry_msgs::Pose> waypoints;

      command_cartesian_position = group.getCurrentPose(ee_link);  
      geometry_msgs::Pose p1 = command_cartesian_position.pose;
      p1.position.x -= direction*0.09;
      p1.position.z -= direction*0.09;
  
      waypoints.push_back(p1);
      group.setMaxVelocityScalingFactor(1);
 
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
     
      ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

      group.setStartStateToCurrentState();
      int point_num = trajectory.joint_trajectory.points.size();
      for(int i=0;i < point_num;i++){
        std::vector<double> joint_value = trajectory.joint_trajectory.points[i].positions;
        group.setJointValueTarget(joint_value);
      }
      success_plan = group.plan(myplan);
      if (success_plan) {
        
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        direction *= -1; // In the next iteration the motion will be on the opposite direction
        ROS_INFO_STREAM("Turned around direction. Sleeping 10s, allegedly.");
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
        waypoints.clear();
      }
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(3.0).sleep(); // 3 seconds
    }
  }  
}; 
