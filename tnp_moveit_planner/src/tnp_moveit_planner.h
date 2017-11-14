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

// For the kinematic calculations
#include <moveit/robot_model_loader/robot_model_loader.h>   
#include <tf_conversions/tf_eigen.h>

#include "tnp_moveit_planner/moveCameraTo.h"
#include "tnp_moveit_planner/moveGripperTo.h"
#include "tnp_moveit_planner/moveSuctionCupTo.h"
#include "tnp_moveit_planner/moveToCartesianPose.h"
#include "tnp_moveit_planner/setMotionParams.h"
#include "tnp_moveit_planner/getFlangePoseAtEEPose.h"
#include "tnp_moveit_planner/isEEPoseReachable.h"


class TNP_Moveit_Planning
{
public:
  //Constructor
  TNP_Moveit_Planning();

  bool initializePositions(); // Populates joint angles/positions we use. Called during startup.


  bool moveToPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &group);
  bool moveToPoseLIN(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &group);

  
  bool moveCameraToCallback(tnp_moveit_planner::moveCameraTo::Request &req,
                                 tnp_moveit_planner::moveCameraTo::Response &res);
  bool moveGripperToCallback(tnp_moveit_planner::moveGripperTo::Request &req,
                                 tnp_moveit_planner::moveGripperTo::Response &res);
  bool moveSuctionCupToCallback(tnp_moveit_planner::moveSuctionCupTo::Request &req,
                                 tnp_moveit_planner::moveSuctionCupTo::Response &res);
  bool moveToCartesianPoseCallback(tnp_moveit_planner::moveToCartesianPose::Request &req,
                                 tnp_moveit_planner::moveToCartesianPose::Response &res);
  bool setMotionParamsCallback(tnp_moveit_planner::setMotionParams::Request &req,
                                 tnp_moveit_planner::setMotionParams::Response &res);
  bool getFlangePoseAtEEPoseCallback(tnp_moveit_planner::getFlangePoseAtEEPose::Request &req,
                                 tnp_moveit_planner::getFlangePoseAtEEPose::Response &res);
  bool isEEPoseReachableCallback(tnp_moveit_planner::isEEPoseReachable::Request &req,
                                 tnp_moveit_planner::isEEPoseReachable::Response &res);

  bool getFlangePoseAtEEPose(geometry_msgs::Pose EE_target_pose, geometry_msgs::Pose& out_flange_pose, double& A7_angle, bool use_gripper_EE);

private:
  ros::NodeHandle n_;

  // Service declarations
  ros::ServiceServer moveCameraToService;
  ros::ServiceServer moveGripperToService;
  ros::ServiceServer moveSuctionCupToService;
  ros::ServiceServer moveToCartesianPoseService;
  ros::ServiceServer setMotionParamsService;
  ros::ServiceServer getFlangePoseAtEEPoseService;
  ros::ServiceServer isEEPoseReachableService;

  std::string target_ee_link;

  iiwa_ros::iiwaRos my_iiwa;

  // RobotModelLoader etc. for kinematic stuff
  robot_model_loader::RobotModelLoader robot_model_loader;

  // MoveGroup for planning
  moveit::planning_interface::MoveGroupInterface suction_group, gripper_group, camera_group, flange_group;
  moveit::planning_interface::MoveGroupInterface::Plan myplan;

};