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

#include "tnp_kuka_motion.h"
#include <stdexcept>

#include "tnp_kuka_motion_helper_functions.h"

using namespace std;

// ======= Actual class code

KukaMotionNode::KukaMotionNode() :
    moveToJointAnglesPTPActionServer_(n_, "tnp_kuka_motion/moveToJointAnglesPTPAction",
                                      boost::bind(&KukaMotionNode::execute, this, _1), false)
{
  initializePositions();

  //Topic you want to publish
  pubJointPos_ = n_.advertise<iiwa_msgs::JointPosition>("iiwa/command/JointPosition", 1000);
  pubCartPose_ = n_.advertise<geometry_msgs::PoseStamped>("iiwa/command/CartesianPose", 1000);

  // services you want to advertise
  calibrationMotionService = n_.advertiseService("tnp_kuka_motion/calibrationMotion", &KukaMotionNode::calibrationMotionCallback, this);
  goToContainerService = n_.advertiseService("tnp_kuka_motion/goToContainer", &KukaMotionNode::goToContainerCallback, this);
  goToHomeService = n_.advertiseService("tnp_kuka_motion/goToHome", &KukaMotionNode::goToHomeCallback, this);
  goToToteService = n_.advertiseService("tnp_kuka_motion/goToTote", &KukaMotionNode::goToToteCallback, this);
  goToLookIntoContainerService = n_.advertiseService("tnp_kuka_motion/goToLookIntoContainer", &KukaMotionNode::goToLookIntoContainerCallback, this);
  goToLookIntoToteService = n_.advertiseService("tnp_kuka_motion/goToLookIntoTote",
                                                &KukaMotionNode::goToLookIntoToteCallback, this);
  goToAmnestyService = n_.advertiseService("tnp_kuka_motion/goToAmnesty", &KukaMotionNode::goToAmnestyCallback, this);
  goToLookIntoAmnestyService = n_.advertiseService("tnp_kuka_motion/goToLookIntoAmnesty",
                                                   &KukaMotionNode::goToLookIntoAmnestyCallback, this);
  goToRecSpaceService = n_.advertiseService("tnp_kuka_motion/goToRecSpace", &KukaMotionNode::goToRecSpaceCallback,
                                            this);
  goToBinService = n_.advertiseService("tnp_kuka_motion/goToBin", &KukaMotionNode::goToBinCallback, this);
  goToLookIntoBinService = n_.advertiseService("tnp_kuka_motion/goToLookIntoBin",
                                               &KukaMotionNode::goToLookIntoBinCallback, this);
  goToBoxService = n_.advertiseService("tnp_kuka_motion/goToBox", &KukaMotionNode::goToBoxCallback, this);
  canRobotPlaceItemHereService = n_.advertiseService("tnp_kuka_motion/canRobotPlaceItemHere", &KukaMotionNode::canRobotPlaceItemHereCallback,
                                              this);
  putItemIntoContainerService = n_.advertiseService("tnp_kuka_motion/putItemIntoContainer", &KukaMotionNode::putItemIntoContainerCallback,
                                              this);
  putItemIntoBinService = n_.advertiseService("tnp_kuka_motion/putItemIntoBin", &KukaMotionNode::putItemIntoBinCallback,
                                              this);
  putItemIntoBoxService = n_.advertiseService("tnp_kuka_motion/putItemIntoBox", &KukaMotionNode::putItemIntoBoxCallback,
                                              this);
  putItemIntoToteService = n_.advertiseService("tnp_kuka_motion/putItemIntoTote", &KukaMotionNode::putItemIntoToteCallback,
                                              this);
  putItemIntoAmnestyService = n_.advertiseService("tnp_kuka_motion/putItemIntoAmnesty", &KukaMotionNode::putItemIntoAmnestyCallback,
                                              this);
  moveToJointAnglesPTPService = n_.advertiseService("tnp_kuka_motion/moveToJointAnglesPTP",
                                                    &KukaMotionNode::moveToJointAnglesPTPCallback, this);
  moveToCartPosePTPService = n_.advertiseService("tnp_kuka_motion/moveToCartPosePTP",
                                                 &KukaMotionNode::moveToCartPosePTPCallback, this);
  moveSuctionCupToService = n_.advertiseService("tnp_kuka_motion/moveSuctionCupTo",
                                                 &KukaMotionNode::moveSuctionCupToCallback, this);
  moveGripperToService = n_.advertiseService("tnp_kuka_motion/moveGripperTo",
                                                 &KukaMotionNode::moveGripperToCallback, this);
  sweepToteHorizontalService = n_.advertiseService("tnp_kuka_motion/sweepToteHorizontal", &KukaMotionNode::sweepToteHorizontalCallback, this);
  sweepToteCornersService = n_.advertiseService("tnp_kuka_motion/sweepToteCorners", &KukaMotionNode::sweepToteCornersCallback, this);
  nudgeItemsIntoContainerService = n_.advertiseService("tnp_kuka_motion/nudgeItemsIntoContainer", &KukaMotionNode::nudgeItemsIntoContainerCallback, this);
  suckItemService = n_.advertiseService("tnp_kuka_motion/suckItem", &KukaMotionNode::suckItemCallback, this);
  graspItemService = n_.advertiseService("tnp_kuka_motion/graspItem", &KukaMotionNode::graspItemCallback, this);
  wobbleForVisionService = n_.advertiseService("tnp_kuka_motion/wobbleForVision",
                                               &KukaMotionNode::wobbleForVisionCallback, this);

  // Services we subscribe to
  gripperClient = n_.serviceClient<tnp_end_effector::Gripper>("/tnp_end_effector/gripper");
  suctionClient = n_.serviceClient<tnp_end_effector::Suction>("/tnp_end_effector/suction");
  LinActuatorSuctionClient = n_.serviceClient<tnp_end_effector::LinActuatorSuction>(
      "/tnp_end_effector/lin_actuator_suction");
  EEControlClient = n_.serviceClient<tnp_end_effector::EEControl>(
      "/tnp_end_effector/set_ee_status");
  moveGripperToClient = n_.serviceClient<tnp_moveit_planner::moveGripperTo>("/iiwa/tnp_moveit_planner/moveGripperTo");
  moveSuctionCupToClient = n_.serviceClient<tnp_moveit_planner::moveSuctionCupTo>("/iiwa/tnp_moveit_planner/moveSuctionCupTo");
  getFlangePoseAtEEPoseClient = n_.serviceClient<tnp_moveit_planner::getFlangePoseAtEEPose>("/iiwa/tnp_moveit_planner/getFlangePoseAtEEPose");
  isEEPoseReachableClient = n_.serviceClient<tnp_moveit_planner::isEEPoseReachable>("/iiwa/tnp_moveit_planner/isEEPoseReachable");
  showPoseClient = n_.serviceClient<tnp_msgs::ShowPose>("/tnp_monitor/showPose");
  showGraspPoseClient = n_.serviceClient<tnp_msgs::ShowGraspPose>("/tnp_monitor/showGraspPose");
  setPathParametersLinClient = n_.serviceClient<iiwa_msgs::SetPathParametersLin>("/iiwa/configuration/pathParametersLin");
  LEDClient = n_.serviceClient<iiwa_msgs::LED>("/iiwa/configuration/LED");

  //Topic you want to subscribe to
  subItemIsSuctioned_ = n_.subscribe("tnp_end_effector/itemIsSuctioned", 1, &KukaMotionNode::itemIsSuctionedCallback, this);
  subItemIsGrasped_ = n_.subscribe("tnp_end_effector/itemIsGrasped", 1, &KukaMotionNode::itemIsGraspedCallback, this);
  subCupIsTouchingSomething_ = n_.subscribe("tnp_end_effector/cupIsTouchingSomething", 1, &KukaMotionNode::cupIsTouchingSomethingCallback, this);
  subGripperIsTouchingSomething_ = n_.subscribe("tnp_end_effector/gripperIsTouchingSomething", 1, &KukaMotionNode::gripperIsTouchingSomethingCallback, this);

  // Actions we serve
  moveToJointAnglesPTPActionServer_.start();

  // Initialize the helper class for time_to_destination and torque data
  my_iiwa_ros_object.init("iiwa");

  // Set up the robot
  my_iiwa_ros_object.getPathParametersService().setJointRelativeAcceleration(1.0);
  my_iiwa_ros_object.getPathParametersService().setJointRelativeVelocity(1.0);
  my_iiwa_ros_object.getPathParametersService().setOverrideJointAcceleration(1.0);

  n_.getParam("arc_task_type_is_stow", arc_task_type_is_stow);
  n_.getParam("arc_task_type_is_pick", arc_task_type_is_pick);
}

bool KukaMotionNode::waitUntilArrived(bool checkForOverload, bool checkForSuction, bool checkForSuctionContact, bool checkForGripperContact)
{
  // ROS_INFO("Waiting for robot to reach its destination");

  // Wait until movement has started
  double ttd = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
  ros::Time start_wait = ros::Time::now();
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(1.0)) {
    ros::Duration(0.1).sleep();
    ttd = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
  }

  double time = 5.;
  double force = 0.0;
  double forcelimit = 60.0; // Newtons. This has to be connected to the outside of this function somehow.
  forcelimit = forcelimit * forcelimit; // Squared once here so we don't have to do sqrt in the loop
  geometry_msgs::WrenchStamped ws;
  ros::Rate loop_rate(100);
  ROS_INFO_STREAM("Entering the waitUntilArrived loop at ttd: " << ttd);
  ros::Duration(0.5).sleep();
  while (time > 0.)
  {
    time = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
    if (time > 0.)
    {
      //ROS_INFO_STREAM(time << " seconds till the robot reaches its destination");
    }
    else if (time == -999.)
    {
      ROS_WARN_STREAM("Something went wrong! Ttd received: " << time << ".");
      ros::Duration(1).sleep();
    }
    else
    {
      ROS_INFO_STREAM(time << " seconds since the robot reached its destination");
    }

    // Check for overload/suction
    ros::spinOnce(); // This is required for iiwa_ros to get updated force data / receive data in the subscribers, probably
    if (checkForOverload)
    {
      my_iiwa_ros_object.getCartesianWrench(ws);
      force = ws.wrench.force.x * ws.wrench.force.x + ws.wrench.force.y * ws.wrench.force.y
          + ws.wrench.force.z * ws.wrench.force.z;
      if (force > forcelimit)
      {
        ROS_INFO_STREAM("Robot encountered too much force!!! Force seen (squared): " << force << " N^2. Stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return false;
      }
    }
    if (checkForSuction)
    {
      if (itemIsSuctioned)
      {
        ROS_INFO_STREAM("Item has been sucked; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return true;
      }
    }
    if (checkForSuctionContact)
    {
      if (cupIsTouchingSomething)
      {
        ROS_INFO_STREAM("Cup has come into contact with an item; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return true;
      }
    }
    if (checkForGripperContact)
    {
      if (gripperIsTouchingSomething)
      {
        ROS_INFO_STREAM("Gripper has come into contact with an item; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return true;
      }
    }
    loop_rate.sleep();
  }
  return true;
}

// This returns false if suction or contact
bool KukaMotionNode::waitUntilArrived2(bool checkForOverload, bool checkForSuction, bool checkForSuctionContact, bool checkForGripperContact)
{
  // ROS_INFO("Waiting for robot to reach its destination");

  // Wait until movement has started
  double ttd = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
  ros::Time start_wait = ros::Time::now();
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(1.0)) {
    ros::Duration(0.1).sleep();
    ttd = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
  }

  double time = 5.;
  double force = 0.0;
  double forcelimit = 60.0; // Newtons. This has to be connected to the outside of this function somehow.
  forcelimit = forcelimit * forcelimit; // Squared once here so we don't have to do sqrt in the loop
  geometry_msgs::WrenchStamped ws;
  ros::Rate loop_rate(100);
  ROS_INFO_STREAM("Entering the waitUntilArrived loop at ttd: " << ttd);
  ros::Duration(0.5).sleep();
  while (time > 0.)
  {
    time = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
    if (time > 0.)
    {
      //ROS_INFO_STREAM(time << " seconds till the robot reaches its destination");
    }
    else if (time == -999.)
    {
      ROS_WARN_STREAM("Something went wrong! Ttd received: " << time << ".");
      ros::Duration(1).sleep();
    }
    else
    {
      ROS_INFO_STREAM(time << " seconds since the robot reached its destination");
    }

    // Check for overload/suction
    ros::spinOnce(); // This is required for iiwa_ros to get updated force data / receive data in the subscribers, probably
    if (checkForOverload)
    {
      my_iiwa_ros_object.getCartesianWrench(ws);
      force = ws.wrench.force.x * ws.wrench.force.x + ws.wrench.force.y * ws.wrench.force.y
          + ws.wrench.force.z * ws.wrench.force.z;
      if (force > forcelimit)
      {
        ROS_INFO_STREAM("Robot encountered too much force!!! Force seen (squared): " << force << " N^2. Stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return false;
      }
    }
    if (checkForSuction)
    {
      if (itemIsSuctioned)
      {
        ROS_INFO_STREAM("Item has been sucked; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return false;
      }
    }
    if (checkForSuctionContact)
    {
      if (cupIsTouchingSomething)
      {
        ROS_INFO_STREAM("Cup has come into contact with an item; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return false;
      }
    }
    if (checkForGripperContact)
    {
      if (gripperIsTouchingSomething)
      {
        ROS_INFO_STREAM("Gripper has come into contact with an item; stopping and waiting 500 ms.");
        stop();
        ros::Duration(1).sleep();
        return false;
      }
    }
    loop_rate.sleep();
  }
  return true;
}

// The service currently accepts values in millimeters and radians (this will probably be fixed in a future iiwa_stack release)
// We convert input values to millimeters  Convert to ROS (meters) convention if values are input in millimeters 
// KUKA API limits: Probably aroud 1 m/s for cartesian values
bool KukaMotionNode::setLinSpeed(double x_vel_lim, double y_vel_lim, double z_vel_lim, double x_ang_lim, double y_ang_lim, double z_ang_lim)
{
  // Delete this section when iiwa_stack fixed its service
  if (x_vel_lim < 2)  // This means it is almost certainly given in meters, but the service currently takes millimeters
  {
    x_vel_lim = x_vel_lim * 1000;
  }
  if (y_vel_lim < 2)  // This means it is almost certainly given in meters, but the service currently takes millimeters
  {
    y_vel_lim = y_vel_lim * 1000;
  }
  if (z_vel_lim < 2)  // This means it is almost certainly given in meters, but the service currently takes millimeters
  {
    z_vel_lim = z_vel_lim * 1000;
  }

  geometry_msgs::Twist limits;
  limits.linear.x = x_vel_lim;
  limits.linear.y = y_vel_lim;
  limits.linear.z = z_vel_lim;
  limits.angular.x = x_ang_lim;
  limits.angular.y = y_ang_lim;
  limits.angular.z = z_ang_lim;

  if (twistEqual(limits, last_sent_vel_limits))
  {
    ROS_INFO_STREAM("Speed limits were already set to the requested speed: " << x_vel_lim << ", " << y_vel_lim << ", " << z_vel_lim << ".");
    return true;
  }

  ROS_INFO_STREAM("Trying to set max cartesian velocity to: " << x_vel_lim << ", " << y_vel_lim << ", " << z_vel_lim);
  iiwa_msgs::SetPathParametersLin srv;
  srv.request.max_cartesian_velocity = limits;
  bool success = false;
  int counter = 0;
  while (!success)
  {
    setPathParametersLinClient.call(srv);
    success = srv.response.success;
    if (success)
    {
      ROS_INFO_STREAM("Set max cartesian velocity to: " << x_vel_lim << ", " << y_vel_lim << ", " << z_vel_lim);
    }
    else
    {
      ROS_WARN("Failed to set cartesian velocity. Waiting 1 second.");
      ros::Duration(1).sleep();
      counter++;
      if (counter > 2)
      {
        return false;
      }
    }
  }
  ros::Duration(1).sleep();
  return success;
}

bool KukaMotionNode::sendLinMotionUntilNearTarget(geometry_msgs::PoseStamped& target_pose)
{
  my_iiwa_ros_object.setCartesianPoseLin(target_pose);
  bool moveSuccess = waitUntilArrived(); // This also checks for suction & contact.

  if (moveSuccess == true){
    int repetitions = 2;
    while (!isKUKANearTarget(target_pose.pose, my_iiwa_ros_object) && repetitions < 5)
    {
      ROS_WARN_STREAM("KUKA was found not to be near target_pose: " << target_pose.pose.position << ". Republishing the command, try nr: "<< repetitions);
      my_iiwa_ros_object.setCartesianPoseLin(target_pose);
      moveSuccess = waitUntilArrived();
    }
    return isKUKANearTarget(target_pose.pose, my_iiwa_ros_object);
  }
}

bool KukaMotionNode::sendLinMotionUntilNearTarget(geometry_msgs::Pose& target_pose)
{
  geometry_msgs::PoseStamped ps;
  my_iiwa_ros_object.getCartesianPose(ps);
  ps.pose = target_pose;
  return sendLinMotionUntilNearTarget(ps);
}

bool KukaMotionNode::moveToJointAnglesPTP(iiwa_msgs::JointPosition jp)
{
  if (kuka_is_dead)
  {
    ROS_ERROR_STREAM("moveToCartPosePTP was called, but all motions are stopped.");
    ros::Duration(20).sleep();
    return false;
  }
  //wait until the connection is completed
  ROS_INFO("Waiting until the connection is completed");
  ros::Rate loop_rate(100);
  while (pubJointPos_.getNumSubscribers() == 0)
  {
    loop_rate.sleep();
  }

  // once connection is ok, send the command
  ROS_INFO_STREAM("Publishing joint angle command: " << jp);
  if (ros::ok())
  {
    pubJointPos_.publish(jp);
    ros::spinOnce();
  }
  else
  {
    ROS_INFO("ros not ok");
    return false;
  }
  bool response = waitUntilArrived();
  if (response == true){
    response = isKUKANearTarget(jp, my_iiwa_ros_object);
  }

  return response;
}

bool KukaMotionNode::moveToJointAnglesPTP(const double& j1, const double& j2, const double& j3, const double& j4,
                                          const double& j5, const double& j6, const double& j7)
{
  iiwa_msgs::JointPosition my_joint_position;
  my_joint_position.position.a1 = j1;
  my_joint_position.position.a2 = j2;
  my_joint_position.position.a3 = j3;
  my_joint_position.position.a4 = j4;
  my_joint_position.position.a5 = j5;
  my_joint_position.position.a6 = j6;
  my_joint_position.position.a7 = j7;
  const bool res = moveToJointAnglesPTP(my_joint_position);
  return res;
}

// Publishes a cartesian pose for the robot to move to.
bool KukaMotionNode::moveToCartPosePTP(const double& x, const double& y, const double& z, const double& u,
                                       const double& v, const double& w)
{
  // Create the pose from the input parameters
  // Prepare components
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;

  geometry_msgs::Quaternion q;
  tf::Quaternion qt;
  qt.setRPY(u, v, w);
  tf::quaternionTFToMsg(qt, q);

  // Create the pose
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  return moveToCartPosePTP(pose);
}

bool KukaMotionNode::moveToCartPosePTP(geometry_msgs::Pose pose, bool wait)
{
  if (kuka_is_dead)
  {
    ROS_ERROR_STREAM("moveToCartPosePTP was called, but all motions are stopped.");
    ros::Duration(20).sleep();
    return false;
  }
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time();
  ps.pose = pose;

  my_iiwa_ros_object.getCartesianPose(ps);
  ps.pose = pose;
  my_iiwa_ros_object.setCartesianPoseLin(ps);
  bool response = true;
  if (wait)
  {
    bool response = waitUntilArrived();
    if (response == true){
      response = isKUKANearTarget(pose, my_iiwa_ros_object);
    }
  }
  ROS_INFO("done publishing cartesian pose");

  return response;
}

bool KukaMotionNode::stop(std::string mode)
{
  if (mode.compare("lin") == 0)
  {
    ROS_INFO("Trying to stop the robot with linear cartesian pose command.");
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
  }
  else if (mode.compare("cartesian") == 0)
  {
    ROS_INFO("Trying to stop the robot with cartesian pose command.");
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    my_iiwa_ros_object.setCartesianPose(ps);
  }
  else
  {
    ROS_INFO("Trying to stop the robot with joint angle command.");
    iiwa_msgs::JointPosition jp;
    my_iiwa_ros_object.getJointPosition(jp);
    my_iiwa_ros_object.setJointPosition(jp);
  }
  ROS_INFO("Sent stop command to robot.");
  return true;
}

bool KukaMotionNode::isEEPoseReachable(geometry_msgs::Pose EE_pose, bool use_gripper_EE)
{
  tnp_moveit_planner::isEEPoseReachable srv;
  srv.request.EE_target_pose = EE_pose;
  srv.request.use_gripper_EE = use_gripper_EE;
  if (isEEPoseReachableClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_moveit_planner/isEEPoseReachable");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_moveit_planner/isEEPoseReachable");
    ROS_ERROR("Failed to call service tnp_moveit_planner/isEEPoseReachable");
    return false;
  }
  ROS_INFO_STREAM("isEEPoseReachable was called. Result: " << std::to_string(srv.response.pose_is_reachable));
  return srv.response.pose_is_reachable;
}

geometry_msgs::Pose KukaMotionNode::getFlangePoseAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE, bool& pose_is_reachable)
{
  tnp_moveit_planner::getFlangePoseAtEEPose srv;
  srv.request.EE_target_pose = EE_pose;
  srv.request.use_gripper_EE = use_gripper_EE;
  if (getFlangePoseAtEEPoseClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_moveit_planner/getFlangePoseAtEEPose");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_moveit_planner/getFlangePoseAtEEPose");
    ROS_ERROR("Failed to call service tnp_moveit_planner/getFlangePoseAtEEPose");
    geometry_msgs::Pose p_null;
    pose_is_reachable = false;
    return p_null;
  }
  ROS_INFO_STREAM("getFlangePoseAtEEPose was called. Pose is " << (srv.response.pose_is_reachable ? "reachable." : "not reachable."));
  pose_is_reachable = srv.response.pose_is_reachable;
  return srv.response.flange_target_pose;
}

bool KukaMotionNode::getFlangePoseAndA7AngleAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE, double& A7_angle)
{
  tnp_moveit_planner::getFlangePoseAtEEPose srv;
  srv.request.EE_target_pose = EE_pose;
  srv.request.use_gripper_EE = use_gripper_EE;
  if (getFlangePoseAtEEPoseClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_moveit_planner/getFlangePoseAtEEPose");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_moveit_planner/getFlangePoseAtEEPose");
    ROS_ERROR("Failed to call service tnp_moveit_planner/getFlangePoseAtEEPose");
    return false;
  }
  ROS_INFO_STREAM("getFlangePoseAtEEPose was called. Result: " << std::to_string(srv.response.pose_is_reachable));
  A7_angle = srv.response.A7_angle;
  return srv.response.pose_is_reachable;
}

double KukaMotionNode::getA7AngleAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE)
{
  tnp_moveit_planner::getFlangePoseAtEEPose srv;
  srv.request.EE_target_pose = EE_pose;
  srv.request.use_gripper_EE = use_gripper_EE;
  if (getFlangePoseAtEEPoseClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_moveit_planner/getFlangePoseAtEEPose to get A7 angle");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_moveit_planner/getFlangePoseAtEEPose to get A7 angle");
    ROS_ERROR("Failed to call service tnp_moveit_planner/getFlangePoseAtEEPose to get A7 angle");
    return false;
  }
  ROS_INFO_STREAM("getFlangePoseAtEEPose was called for A7 angle. Result: " << std::to_string(srv.response.A7_angle));
  return srv.response.A7_angle;
}

void KukaMotionNode::transformPositionFromSuctionCupToFlange(geometry_msgs::Pose& inpose, geometry_msgs::Pose& outpose)
{
  // ROS_INFO_STREAM("Transforming pose's position from suction cup to flange");

  //////// Below is my hacky old way of offsetting the flange pose so that the EE is at a certain point.
  tf::StampedTransform transform1, transform2;
  ros::Time t = ros::Time::now();
  tflistener.waitForTransform("/iiwa_link_0", "/tnptool_suction_link_ee", t, ros::Duration(2.0));
  tflistener.lookupTransform("/iiwa_link_0", "/tnptool_suction_link_ee", t, transform1);
  tflistener.lookupTransform("/iiwa_link_0", "/iiwa_link_ee", t, transform2);

  outpose.position.x = inpose.position.x + (transform2.getOrigin().x() - transform1.getOrigin().x());
  outpose.position.y = inpose.position.y + (transform2.getOrigin().y() - transform1.getOrigin().y());
  outpose.position.z = inpose.position.z + (transform2.getOrigin().z() - transform1.getOrigin().z());

  ROS_INFO_STREAM("Transformed pose's position from suction cup to flange. z_suction_cup, z_flange are: " << inpose.position.z << ", " << outpose.position.z);
  outpose.orientation = inpose.orientation;
}

geometry_msgs::Pose KukaMotionNode::getCurrentSuctionCupPose()
{
  geometry_msgs::PoseStamped ps;
  ps.pose.orientation.w = 1.0;
  ps.header.frame_id = "/tnptool_suction_link_ee";
  transform_pose_now(ps, "/iiwa_link_0", tflistener);
}

void KukaMotionNode::transformPositionFromGripperTipToFlange(geometry_msgs::Pose& inpose, geometry_msgs::Pose& outpose)
{
  // ROS_INFO_STREAM("Transforming pose's position from gripper tip to flange");

  // //////// Below is my hacky old way of offsetting the flange pose so that the EE is at a certain point.
  tf::StampedTransform transform1, transform2;
  ros::Time t = ros::Time::now();
  tflistener.waitForTransform("/iiwa_link_0", "/tnptool_gripper_link_ee", t, ros::Duration(2.0));
  tflistener.lookupTransform("/iiwa_link_0", "/tnptool_gripper_link_ee", t, transform1);
  tflistener.lookupTransform("/iiwa_link_0", "/iiwa_link_ee", t, transform2);

  outpose.position.x = inpose.position.x + (transform2.getOrigin().x() - transform1.getOrigin().x());
  outpose.position.y = inpose.position.y + (transform2.getOrigin().y() - transform1.getOrigin().y());
  outpose.position.z = inpose.position.z + (transform2.getOrigin().z() - transform1.getOrigin().z());

  ROS_INFO_STREAM("Transformed pose's position from gripper tip to flange. z_gripper_tip, z_flange are: " << inpose.position.z << ", " << outpose.position.z);
  outpose.orientation = inpose.orientation;
}

// Checks if a point (in world coordinates) is inside a container, and far away enoug from the borders.
bool KukaMotionNode::isPointSafelyInsideContainer(geometry_msgs::Point p_world, std::string container_name, double safety_margin)
{
  geometry_msgs::Point p_moved = movePointIntoContainerBounds(p_world, container_name, safety_margin);
  if (p_moved.x == p_world.x)
  {
    if (p_moved.y == p_world.y)
    {
      return true;    // This ignores the z-coordinate!
      if (p_moved.z == p_world.z)
      {
        return true;
      } 
    }   
  }
  return false;
}

// Move a point (in world coordinates) so that it will not collide with the container
geometry_msgs::Point KukaMotionNode::movePointIntoContainerBounds(geometry_msgs::Point p_world, std::string container_name, double safety_margin)
{
  ROS_INFO_STREAM("Entering movePointIntoContainerBounds for " << container_name << ", safety margin " << safety_margin << " and inpoint: " << p_world);
  double container_width_x = 0.0, container_width_y = 0.0, container_h = 0.0;

  n_.getParam("tnp_environment/"+container_name+"_w", container_width_x);    // Width of the inside of the container
  n_.getParam("tnp_environment/"+container_name+"_l", container_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/"+container_name+"_h", container_h);          // Container height

  geometry_msgs::Point point_in_container = worldPointToContainer(container_name, p_world, tflistener);

  if (point_in_container.x > (container_width_x - safety_margin))
  {
    ROS_ERROR_STREAM(
        "Target pose x-position too high for for " << container_name << "! Adjusting from " << point_in_container.x << " to " << container_width_x - safety_margin);
    point_in_container.x = container_width_x - safety_margin;
  }
  else if (point_in_container.x < safety_margin)
  {
    ROS_ERROR_STREAM(
        "Target pose x-position too low for " << container_name << "! Adjusting from " << point_in_container.x << " to " << safety_margin);
    point_in_container.x = safety_margin;
  }

  if (point_in_container.y > (container_width_y - safety_margin))
  {
    ROS_ERROR_STREAM(
        "Target pose y-position too high for " << container_name << "! Adjusting from " << point_in_container.y << " to " << container_width_y - safety_margin);
    point_in_container.y = container_width_y - safety_margin;
  }
  else if (point_in_container.y < safety_margin)
  {
    ROS_ERROR_STREAM(
        "Target pose y-position too low for " << container_name << "! Adjusting from " << point_in_container.y << " to " << safety_margin);
    point_in_container.y = safety_margin;
  }
  
  if (point_in_container.z < .01)
  {
    ROS_ERROR_STREAM(
        "Target pose z-position too low for " << container_name << "! Adjusting from " << point_in_container.z << " to " << .01);
    point_in_container.z = .01;
  }
  else if (point_in_container.z > container_h)
  {
    ROS_ERROR_STREAM(
        "Warning: Target pose z-position above container limit! Adjusting from " << point_in_container.z << " to " << container_h);
    point_in_container.z = container_h;
  }

  return containerPointToWorld(container_name, point_in_container, tflistener);
}

// container_name has to be "tote", "amnesty", "box_1", bin_A", "bin_B"... like the TF frames
void KukaMotionNode::containerPoseToWorld(std::string container_name, geometry_msgs::Pose& pose_in_container, geometry_msgs::Pose& pose_in_world)
{
  ROS_INFO_STREAM("Transforming item pose of container to world.");
  geometry_msgs::PoseStamped container_pstamped, world_pstamped;
  container_pstamped.pose = pose_in_container;
  container_pstamped.header.frame_id = "/" + container_name;
  world_pstamped = transform_pose_now(container_pstamped, "/iiwa_link_0", tflistener);
  pose_in_world = world_pstamped.pose;
  ROS_INFO_STREAM("Transformed pose of container to world. Container pose stamped:" << container_pstamped.pose.position << "; and in world: " << world_pstamped.pose.position);
}

// Returns the borders of the container that the point is close to as a string of vectors; "left", "right", "top", "bottom"
std::vector<std::string> KukaMotionNode::getCloseBorders(geometry_msgs::Point p_world, std::string container_name, double detection_margin)
{
  double container_width_x = 0.0, container_width_y = 0.0, container_h = 0.0;

  n_.getParam("tnp_environment/"+container_name+"_w", container_width_x);    // Width of the inside of the container
  n_.getParam("tnp_environment/"+container_name+"_l", container_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/"+container_name+"_h", container_h);          // Container height

  geometry_msgs::Point point_in_container = worldPointToContainer(container_name, p_world, tflistener);
  std::vector<std::string> outvec;

  if (point_in_container.x > (container_width_x - detection_margin))
  {
    outvec.push_back("top");
  }
  else if (point_in_container.x < detection_margin)
  {
    outvec.push_back("bottom");
  }

  if (point_in_container.y > (container_width_y - detection_margin))
  {
    outvec.push_back("left");
  }
  else if (point_in_container.y < detection_margin)
  {
    outvec.push_back("right");
  }

  return outvec;
}

// Input: a desired gripper rotation, defined as a rotation around starting from A7 being neutral
// Output: a gripper orientation where the suction cup does not hit the container and no lock-up occurs
// Theta = 0 means that the gripper is on the right side of the flange. Rotation is about global z.
// Theta is relative to the neutral pose in the container.
double KukaMotionNode::getSafeGraspRotation(geometry_msgs::Point p_world, double theta, std::string container_name)
{
  // Possibility: Check quadrant of the box, allow only a certain range of angles in each. Flip the rotation if the current angle is different.
  // E.g.: Top right quadrant should only be [0, pi/2].
  // Bottom right: [-pi/2, 0]
  // Top left: [pi/2, pi]
  // Bottom left: [-pi/2, -pi]

  // Possibility 2: Ignore it unless the point is within X cm of a border. If yes, check if the orientation is ok for that border.
  // Permissible rotations from the box neutral for the gripper:
  // Near right border: [-pi/2, pi/2] = RB1
  // Near top border: RB1 + pi/2
  // Near bottom border: RB1 - pi/2
  // Near left border: RB1 - pi

  // Possibility 3: Only check if point is close to border. Check the coordinates of the box center in the coordinate system of the EE gripper at the target point.
  // This should always give a correct response.
  // Required: A transform to the target point.
  // CON: This will involve the most sign uncertainties and development time.

  // Approach 2 was insufficient. For corners, a separate check is necessary.
  // Further, a second check for rotations close to the axis maximum is required.
  // In the close corners of a container, A7 needs up to ~35 degrees of additional freedom to be able to grasp the item without locking up
  // It is easiest to just limit the possible range.
  
  std::vector<std::string> borders;
  borders = getCloseBorders(p_world, container_name, 0.15);    // 15 cm detection margin should be enough

  ROS_INFO_STREAM("Evaluating if grasp angle " << theta << " is safe for container " << container_name << "and point " << p_world);
  double safe_theta = theta;
  // theta is certainly [-pi, pi], and should even be [-pi/2, pi/2] in our case
  if (borders.size() == 1)
  {
      if (borders[0].compare("right") == 0)
      {
        safe_theta = flipGraspRotationIfNecessary(theta, (0.0/180.0)*M_PI, (90.0/180.0)*M_PI);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
      else if (borders[0].compare("top") == 0)
      {
        safe_theta = flipGraspRotationIfNecessary(theta, (90.0/180.0)*M_PI, (90.0/180.0)*M_PI);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
      else if (borders[0].compare("bottom") == 0)
      {
        safe_theta = flipGraspRotationIfNecessary(theta, (-90.0/180.0)*M_PI, (90.0/180.0)*M_PI);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
      else if (borders[0].compare("left") == 0)
      {
        safe_theta = flipGraspRotationIfNecessary(theta, (180.0/180.0)*M_PI, (90.0/180.0)*M_PI);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
  }

  // Do a different check in the corners
  // Reminder: at theta = 0 the gripper is horizontal and on the right
  else if (borders.size() == 2)
  {
    double bottomleft = (-135.0/180.0)*M_PI,
          bottomright = (-45.0/180.0) *M_PI,
          topright =    (45.0/180.0)  *M_PI,
          topleft =     (135.0/180.0) *M_PI;
    if (borders[0].compare("right") == 0)
    {
      if (borders[1].compare("top") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, topright);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
      else if (borders[1].compare("bottom") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, bottomright);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
    }
    else if (borders[0].compare("top") == 0)
    {
      if (borders[1].compare("left") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, topleft);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
      else if (borders[1].compare("right") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, topright);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
    }
    else if (borders[0].compare("bottom") == 0)
    {
      if (borders[1].compare("left") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, bottomleft);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
      else if (borders[1].compare("right") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, bottomright);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
    }
    else if (borders[0].compare("left") == 0)
    {
      if (borders[1].compare("top") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, topleft);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
      else if (borders[1].compare("bottom") == 0)
      {
        safe_theta = getSafeRotationValueInCorner(theta, bottomleft);
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
    }
  }

  // Restrict the rotation to safe values:
  if (abs(theta) > (120.0/180.0)*M_PI )
  {
    double oldtheta = theta;
    theta = restrictValueToInterval(theta, -(120.0/180.0)*M_PI, (120.0/180.0)*M_PI);
    ROS_WARN_STREAM("The calculated ideal grasp rotation is " << oldtheta*180.0/M_PI << " degrees, but to avoid lock-up, we limit it to: " << theta*180.0/M_PI);
  }

  ROS_INFO_STREAM("The grasp angle " << safe_theta << " was returned as safe.");
  return safe_theta;
}

double KukaMotionNode::getSafeSuckRotation(geometry_msgs::Point p_world, double theta, std::string container_name)
{
  // Copied from SafeGraspRotation
  std::vector<std::string> borders;
  borders = getCloseBorders(p_world, container_name, 0.15);    // 15 cm detection margin should be enough

  ROS_INFO_STREAM("Evaluating if grasp angle " << theta << " is safe for container " << container_name << "and point " << p_world);
  double safe_theta = theta;
  // Don't change near left border, give tilted rotation near top, give top/bottom rotation near right
  if (borders.size() == 1)
  {
      if (borders[0].compare("top") == 0)
      {
        safe_theta = (0.0/180.0)*M_PI;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
      else if (borders[0].compare("bottom") == 0)
      {
        safe_theta = (0.0/180.0)*M_PI;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }      
      else if (borders[0].compare("right") == 0)
      {
        safe_theta = (-90.0/180.0)*M_PI;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " border.");
      }
  }
   // Do a different check in the corners
  else if (borders.size() == 2)
  {
    double bottomleft = 0.0,
          bottomright = (140.0/180.0)*M_PI,
          topright =    (-90.0/180.0)*M_PI,
          topleft =     0.0;
    if (borders[0].compare("right") == 0)
    {
      if (borders[1].compare("top") == 0)
      {
        safe_theta = topright;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
      else if (borders[1].compare("bottom") == 0)
      {
        safe_theta = bottomright;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
    }
    else if (borders[0].compare("top") == 0)
    {
      if (borders[1].compare("left") == 0)
      {
        safe_theta = topleft;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
      else if (borders[1].compare("right") == 0)
      {
        safe_theta = topright;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
    }
    else if (borders[0].compare("bottom") == 0)
    {
      if (borders[1].compare("left") == 0)
      {
        safe_theta = bottomleft;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
      else if (borders[1].compare("right") == 0)
      {
        safe_theta = bottomright;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[0] << " " << borders[1] << " border.");
      }
    }
    else if (borders[0].compare("left") == 0)
    {
      if (borders[1].compare("top") == 0)
      {
        safe_theta = topleft;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
      else if (borders[1].compare("bottom") == 0)
      {
        safe_theta = bottomleft;
        ROS_INFO_STREAM("The point was found to be close to the " << borders[1] << " " << borders[0] << " border.");
      }
    }
  }

  return safe_theta;
}

geometry_msgs::Pose KukaMotionNode::getSuctionCupPoseToSuckItemInContainer(geometry_msgs::Point p_world, std::string container_name)
{
  geometry_msgs::Pose outpose;
  outpose.position = movePointIntoContainerBounds(p_world, container_name, suction_safety_margin_to_borders_); //*safety_margin*/);   

  // The above fixes the point to something sensible. Now to choose the rotation.
  // At start position, the gripper is aligned to grasp the y-axis (and on the right side of the flange)
  tf::Quaternion q_start = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);   

  // Rotate to the container we're in
  tf::Quaternion q_turn_to_box;
  if (container_name.compare("tote") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI);

  }
  if (container_name.compare("bin_A") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

  }
  if (container_name.compare("bin_B") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);

  }
  if (container_name.compare("bin_C") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);

  }

  // For faraway objects, tilt the head
  geometry_msgs::Point robot_origin, item_point_in_plane;
  item_point_in_plane.x = outpose.position.x;
  item_point_in_plane.y = outpose.position.y;
  ROS_WARN_STREAM("target suck pose in plane: " << item_point_in_plane.x << ", " << item_point_in_plane.y);
  ROS_WARN_STREAM("Distance of target suck pose to robot origin: " << pointDistance(robot_origin, item_point_in_plane));
  if (pointDistance(robot_origin, item_point_in_plane) < .50)
  {
    ROS_INFO_STREAM("Pose is close to the robot. Setting bottom rotation.");
    // Create the pose quaternion from the grasp rotation
    tf::Quaternion q_turn_suction_cup_to_bottom = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0*M_PI));   
    tf::Quaternion q_container_neutral = q_turn_to_box * q_start;
    tf::Quaternion q_final = q_turn_suction_cup_to_bottom * q_turn_to_box * q_start;
    tf::quaternionTFToMsg(q_final, outpose.orientation);
  }
  else
  {
    // Theta is the rotation angle around z starting from the container_neutral pose
    double safe_theta = getSafeSuckRotation(outpose.position, 0.0, container_name);

    ROS_INFO_STREAM("Received theta = " << safe_theta << " for target suck point " << outpose.position.x << ", " << outpose.position.y << ", " << outpose.position.z);
    // Create the pose quaternion from the grasp rotation
    tf::Quaternion q_turn_to_safe_suck_pose = tf::createQuaternionFromRPY(0.0, 0.0, safe_theta);   
    tf::Quaternion q_container_neutral = q_turn_to_box * q_start;
    tf::Quaternion q_final = q_turn_to_safe_suck_pose * q_turn_to_box * q_start;
    tf::quaternionTFToMsg(q_final, outpose.orientation);
  }

  if (pointDistance(robot_origin, item_point_in_plane) > .70)
  {
    if (!checkIfSuctionTargetOK(outpose, container_name))
    {
      ROS_WARN("Found an invalid suction target pose in getSuctionCupPoseToSuckItemInContainer. Getting turned orientation.");
      outpose.orientation = getTiltedEEOrientation(p_world.x, p_world.y, true);
    }
  }  

  // From this distance, the robot would definitely lock up
  if (pointDistance(robot_origin, item_point_in_plane) > .78)
  {
    ROS_WARN("Found suction target pose in getSuctionCupPoseToSuckItemInContainer that is too extended. Getting turned orientation.");
    outpose.orientation = getTiltedEEOrientation(p_world.x, p_world.y, true);
  }
  
  return outpose;
}

// TODO: Replace with double getSuctionCupOrientation(geometry_msgs::Point p, std::string container_name);
// TODO: Rewrite using the readable helper functions.
// Checks if an item-pick-pose is inside the container, returns a "corrected" pose if not, and displays a warning.
// The item_pose is in world.
geometry_msgs::Pose KukaMotionNode::checkIfPoseInContainer(geometry_msgs::Pose& item_pose, int container_id, bool useSuction)
{
  geometry_msgs::Pose outpose = item_pose;
  double container_x = 0.0, container_y = 0.0, container_h = 0.0, container_floor_z = 0.0, container_width_x = 0.0, container_width_y = 0.0;
  double container_center_x = 0.0, container_center_y = 0.0;

  const double safetymargin = .03; // Distance of point to walls

  double tmp = 0.0;

  ROS_INFO_STREAM("Entering checkIfPoseInContainer for container_id " << container_id << " and inpose: " << item_pose);

  // Define EE rotations in "home" stance (above bin A)
  tf::Quaternion suction_cup_near(-.707, .707, 0.0, 0.0), // Suction cup close to the robot
  suction_cup_far(.707, .707, 0.0, 0.0),                // Suction cup away from the robot
  suction_cup_left(0.0, 1.0, 0.0, 0.0),                  // Suction cup to the left of the robot
  suction_cup_right(1.0, 0.0, 0.0, 0.0);                 // Suction cup on the right of the robot
  tf::Quaternion q_rotate_to_container, q_rotate_tool_to_front;
  tf::Quaternion q_res;

  std::string container_name = getContainerName(container_id);

  n_.getParam("tnp_environment/"+container_name+"_x", container_x);    // X-position of the binA center.
  n_.getParam("tnp_environment/"+container_name+"_y", container_y);  // y-position of the binA center.
  n_.getParam("tnp_environment/"+container_name+"_z", container_floor_z);   // Z-position of the binA bottom (inside surface).
  n_.getParam("tnp_environment/"+container_name+"_w", container_width_x);    // Width of the inside of the binA
  n_.getParam("tnp_environment/"+container_name+"_l", container_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/"+container_name+"_h", container_h);          // Container height

  // Get the container's center point
  geometry_msgs::Point opposite_corner_point_container, opposite_corner_point_world;
  opposite_corner_point_container.x = container_width_x;
  opposite_corner_point_container.y = container_width_y;
  opposite_corner_point_world = containerPointToWorld(container_name, opposite_corner_point_container, tflistener);

  container_center_y = (container_y + opposite_corner_point_world.y) / 2.0;
  container_center_x = (container_x + opposite_corner_point_world.x) / 2.0;

  // Switch x/y so that the bin is rotated by 90 degrees for the position check. This is so unbelievably unelegant. Goddamn.
  if ((container_id == 0) || (container_id == 2) || (container_id == 3))
  {
    tmp = container_width_x;
    container_width_x = container_width_y;
    container_width_y = tmp;
  }

  // TODO: Do this cleanly with a rotation from home, and according to the tool used. UGH.
  if (container_id == 0) // Tote
  {
    if (item_pose.position.y > container_center_y)
    {
      // Suction cup towards the robot
      q_rotate_to_container = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI);     // I expected this to be the other way round, but it checks out. What??
      q_res = q_rotate_to_container*suction_cup_near;
      tf::quaternionTFToMsg(q_res, outpose.orientation);
    }
    else
    {
      // Suction cup away from the robot
      outpose.orientation = getTiltedEEOrientation(item_pose.position.x, item_pose.position.y, useSuction);
    }

  }
  else if (container_id == 1) // Bin A
  {
    if (item_pose.position.x > container_center_x)
    {
      // Suction cup at the front
      outpose.orientation = getTiltedEEOrientation(item_pose.position.x, item_pose.position.y, useSuction);
    }
    else  // Side closer to the robot
    {
      // Suction cup at the back
      outpose.orientation.x = -.707;
      outpose.orientation.y = .707;
      outpose.orientation.z = 0.0;
      outpose.orientation.w = 0.0;

      // Incline anyway if the position is too far away from origin
      geometry_msgs::Point robot_origin, item_point_in_plane;
      item_point_in_plane.x = item_pose.position.x;
      item_point_in_plane.y = item_pose.position.y;
      if (pointDistance(robot_origin, item_point_in_plane) > .76)
      {
        ROS_WARN("Inclining end effector in bottom half of the bin");
        outpose.orientation = getTiltedEEOrientation(item_pose.position.x, item_pose.position.y, true);  
      }  
    }
  }
  else if (container_id == 2) // Bin B
  {
    if (item_pose.position.y < container_center_y)
    {
      // Suction cup towards the robot
      outpose.orientation.x = 1.0;
      outpose.orientation.y = 0.0;
      outpose.orientation.z = 0.0;
      outpose.orientation.w = 0.0;
    }
    else
    {
      // // Suction cup away from the robot
      outpose.orientation = getTiltedEEOrientation(item_pose.position.x, item_pose.position.y, useSuction);
    }
  }
  else if (container_id == 3) // Bin C
  {
    if (item_pose.position.y < container_center_y)
    {
      // Suction cup towards the robot
      outpose.orientation.x = 1.0;
      outpose.orientation.y = 0.0;
      outpose.orientation.z = 0.0;
      outpose.orientation.w = 0.0;
    }
    else
    {
      // // Suction cup away from the robot
      outpose.orientation = getTiltedEEOrientation(item_pose.position.x, item_pose.position.y, useSuction);
    }
  }
  else if (container_id == 4) // Amnesty tote
  {
    if (item_pose.position.x > container_center_x)
    {
      // Suction cup at the front
      outpose.orientation.x = -.707;
      outpose.orientation.y = .707;
      outpose.orientation.z = 0.0;
      outpose.orientation.w = 0.0;
    }
    else
    {
      // // Suction cup at the back
      // Suction cup at the front
      outpose.orientation.x = .707;
      outpose.orientation.y = .707;
      outpose.orientation.z = 0.0;
      outpose.orientation.w = 0.0;
    }
  }

  bool pose_corrected = false;

  if (item_pose.position.x > (container_center_x + container_width_x / 2.0 - safetymargin))
  {
    outpose.position.x = container_center_x + container_width_x / 2.0 - safetymargin;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Target pose x-position too high! Adjusting from " << item_pose.position.x << " to " << outpose.position.x);
  }
  else if (item_pose.position.x < (container_center_x - container_width_x / 2.0 + safetymargin))
  {
    outpose.position.x = container_center_x - container_width_x / 2.0 + safetymargin;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Target pose x-position too low! Adjusting from " << item_pose.position.x << " to " << outpose.position.x);
  }

  if (item_pose.position.y > (container_center_y + container_width_y / 2.0 - safetymargin))
  {
    outpose.position.y = container_center_y + container_width_y / 2.0 - safetymargin;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Target pose y-position too high! Adjusting from " << item_pose.position.y << " to " << outpose.position.y);
    ROS_ERROR_STREAM("(Error case 1: container_center_y = " << container_center_y << "; container_width_y / 2.0 = " << (container_width_y / 2.0));
  }
  else if (item_pose.position.y < (container_center_y - container_width_y / 2.0 + safetymargin))
  {
    outpose.position.y = container_center_y - container_width_y / 2.0 + safetymargin;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Target pose y-position too low! Adjusting from " << item_pose.position.y << " to " << outpose.position.y);
    ROS_ERROR_STREAM("(Error case 2: container_center_y = " << container_center_y << "; container_width_y / 2.0 = " << (container_width_y / 2.0));
  }

  if (item_pose.position.z < (container_floor_z + safetymargin / 2.0))
  {
    outpose.position.z = container_floor_z - +safetymargin / 2.0;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Target pose z-position too low! Adjusting from " << item_pose.position.z << " to " << outpose.position.z);
  }
  else if (item_pose.position.z > (container_floor_z + container_h + .00))
  {
    outpose.position.z = container_floor_z + container_h + .00;
    pose_corrected = true;
    ROS_ERROR_STREAM(
        "Warning: Target pose z-position too high! Adjusting from " << item_pose.position.z << " to " << outpose.position.z);
  }

  if (pose_corrected)
  {
    publishPoseMarker(outpose, "refused_item_pose");
  }

  ROS_INFO_STREAM("Returning from checkIfPoseInContainer: " << outpose);
  return outpose;
}

// Get a pose that is in the center above the the container
// height is in container coordinates, measured from the TOP of the container!
geometry_msgs::Point KukaMotionNode::getPointHighAboveContainer(std::string container_name, const double height)
{
  float container_x = 0.0, container_y = 0.0, container_h = 0.0, container_floor_z = 0.0, container_width_x = 0.0, container_width_y = 0.0;
  n_.getParam("tnp_environment/"+container_name+"_w", container_width_x);
  n_.getParam("tnp_environment/"+container_name+"_l", container_width_y);
  n_.getParam("tnp_environment/"+container_name+"_h", container_h);

  geometry_msgs::Pose p;
  p.position.x = container_width_x / 2.0;
  p.position.y = container_width_y / 2.0;
  p.position.z = container_h + height;
  p.orientation.w = 1.0;    // Only to make the Quaternion valid. The orientation is unused.

  containerPoseToWorld(container_name, p, p);
  return p.position;
}

// TODO: This does not catch all positions that lock up, because KUKA's driver does not permit changing speeds near singularities
bool KukaMotionNode::checkIfSuctionTargetOK(geometry_msgs::Pose target_pose, std::string container_name)
{
  bool gripper_EE_is_used = false;
  double container_floor_z, container_h;
  n_.getParam("tnp_environment/" + container_name + "_z", container_floor_z);
  n_.getParam("tnp_environment/" + container_name + "_h", container_h);

  // Prepare the poses
  geometry_msgs::Pose pose_close_above_container = target_pose;
  geometry_msgs::Pose pose_EE_bottom = target_pose;
  geometry_msgs::Pose pose_higher_above_container = pose_close_above_container;

  pose_close_above_container.position.z = container_floor_z + container_h + .05;
  pose_EE_bottom.position.z = container_floor_z + .01;
  pose_higher_above_container.position.z = container_floor_z + container_h + .01;
  if (! ((isEEPoseReachable(pose_close_above_container, gripper_EE_is_used)) &&
       (isEEPoseReachable(pose_EE_bottom, gripper_EE_is_used)) ) )
  {
    ROS_ERROR("This suction target was not reported as feasible.");
    return false;
  }
  return true;
}

// This is supposed to be the tool-agnostic version of suck/graspItem
// In this function, the target_pose has to contain the correct orientation
// Returns true only if an item has been sucked/grasped
bool KukaMotionNode::pickUpItem(geometry_msgs::Pose target_pose, bool gripper_EE_is_used, int force, int width, bool set_up_EE)
{
  ROS_INFO("---------------------------------------------------------");
  ROS_INFO_STREAM("pickUpItem was called with gripper_is_used = " << gripper_EE_is_used << " and target_pose = " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << " (orientation:) "<< target_pose.orientation.x << ", " << target_pose.orientation.y << ", " << target_pose.orientation.z << " " << target_pose.orientation.w << " " << " and force " << force);
  
  // --- Defining all the parameters and variables used later
  geometry_msgs::Pose target_item_pose = target_pose;
  publishPoseMarker(target_item_pose, "target_item_pose");
  int target_container = guessTargetContainer(target_item_pose);
  std::string container_name = getContainerName(target_container);

  bool moveSuccess = false;
  bool DEBUGFLAG = false, ee_live = true;

  double container_floor_z, container_h;
  n_.getParam("tnp_environment/" + container_name + "_z", container_floor_z);
  n_.getParam("tnp_environment/" + container_name + "_h", container_h);

  // Prepare the poses
  geometry_msgs::Pose pose_close_above_container = target_item_pose;
  geometry_msgs::Pose pose_EE_bottom = target_item_pose;
  geometry_msgs::Pose pose_higher_above_container = pose_close_above_container;

  pose_close_above_container.position.z = container_floor_z + container_h + .05;
  pose_EE_bottom.position.z = container_floor_z + .01;
  pose_higher_above_container.position.z = container_floor_z + container_h + .01;

  // Check if all poses are permissible
  ROS_INFO("Checking if pickUpItem command is feasible.");

  if (! ((isEEPoseReachable(pose_close_above_container, gripper_EE_is_used)) &&
       (isEEPoseReachable(pose_EE_bottom, gripper_EE_is_used)) ) )
  {
    ROS_ERROR("This pickUpItem command was not reported as feasible.");
    return false;
  }

  double A7_now, A7_target = getA7AngleAtEEPose(pose_EE_bottom, false);
  iiwa_msgs::JointPosition jp;
  my_iiwa_ros_object.getJointPosition(jp);
  A7_now = jp.position.a7;
  ROS_INFO_STREAM("Checking if A7 angle of target pickUpItem pose is safe to go to from current pose. Current: " <<  (A7_now/M_PI)*180.0 << ", target: " <<  (A7_target/M_PI)*180.0);
  if (A7_now * A7_target < 0)   // If there is a sign change
  {
    // Go to neutral pose first to reset position
    ROS_WARN_STREAM("Dangerous change of A7 from " <<  (A7_now/M_PI)*180.0 << " to " <<  (A7_target/M_PI)*180.0 << " detected! Turning to neutral position before going to target, so no lock-up occurs.");
    jp.position.a7 = 0.0;
    my_iiwa_ros_object.setJointPosition(jp);
    waitUntilArrived();
  }

  // --- Perform the actual motion:
  // 0) Prepare the end effector
  // 1) Go to start position (5 cm above container)
  // 2) Move down to target (while grasping/sucking)
  // 3) Go back up to start position
  // 4) If successful, lift the item by moving to safe position 15 cm above the container

  setLED("off");
  if (set_up_EE)    // This is a separate block so that we just keep moving the suction cup around while sucking
  {
    if (gripper_EE_is_used){
      ROS_INFO("Extending gripper and retracting suction");
      setEEStatus(0, 0, 1, 0, width);
      // bool gripper_closed, bool suction_on, bool gripper_extended, bool suction_extended, int gripper_control_param, int suction_force)
    }
    else {
      ROS_INFO("Extending suction and retracting gripper");
      setEEStatus(1, 0, 0, 1, 5, 40);
      // bool gripper_closed, bool suction_on, bool gripper_extended, bool suction_extended, int gripper_control_param, int suction_force)
    }
  }

  // 1) Go to start position (5 cm above container)
  ROS_INFO("---------------------------------------------------------");
  ROS_INFO("PickUpItem_1) Moving target end effector to target position at container_h + 5 cm");
  moveSuccess = moveEETo(pose_close_above_container, gripper_EE_is_used);
  if (!moveSuccess) {setLinSpeed(); return false;}
  if (DEBUGFLAG) {ros::Duration(1).sleep();}

  // 2) Move down to target (while grasping/sucking)
  if (setLinSpeed(1.0, 1.0, .1)) // 100 mm/s in z
  {
    if (gripper_EE_is_used)
    {
      ROS_INFO_STREAM("PickUpItem_2) Moving gripper down until collision: " << pose_EE_bottom.position.x << ", " << pose_EE_bottom.position.y << ", " << pose_EE_bottom.position.z);
      moveGripperTo(pose_EE_bottom, true, false, true);  // The last parameter is for MoveIt planning
      moveSuccess = waitUntilArrived(1, 1, 1, 1); // This also checks for suction & contact of cup and gripper

      if (force < 100)
      {
        ROS_WARN("Force was set too low. Adjusting to FULL POWER!");
        force = 1500;
      }
      CloseGripperRelativeForceControl(force);
      ros::Duration(1.0).sleep();
    }
    else
    {
      ROS_INFO("Turning on suction.");
      ROS_WARN("Implement force_data for suction in this function!");
      StartSuction(force);

      // Move down towards the item
      ROS_INFO("---------------------------------------------------------");
      ROS_INFO_STREAM("PickUpItem_2) Moving towards bottom of container until suction success or cup collision: " << pose_EE_bottom.position.x << ", " << pose_EE_bottom.position.y << ", " << pose_EE_bottom.position.z); 
      moveSuctionCupTo(pose_EE_bottom, true, false, true);  // The last parameter is for MoveIt planning
      publishPoseMarker(pose_EE_bottom, "target_robot_pose");
      moveSuccess = waitUntilArrived(1, 1, 1, 1); // This also checks for suction & contact of cup and gripper
      
      // Repeat the movement if it gets lost
      if (moveSuccess == true){
        int repetitions = 2;
        while (!isKUKANearTarget(pose_EE_bottom, my_iiwa_ros_object) && repetitions < 5)
        {
          ROS_WARN_STREAM("KUKA was found not to be near target_pose: " << pose_EE_bottom.position.x << ", " << pose_EE_bottom.position.y << ", " << pose_EE_bottom.position.z << ". Republishing the command, try nr: " << repetitions);
          moveSuctionCupTo(pose_EE_bottom, true, false, true);  // The last parameter is for MoveIt planning
          moveSuccess = waitUntilArrived(1, 1, 1, 1); // This also checks for suction & contact of cup and gripper
        }
      }
      // TODO: The waitUntilArrived function may cause a machine violation limit when it sets a position the robot considers unreasonable. Hm.
      // TODO: If either ItemIsSuctioned or cupIsTouchingSomething is already true, this causes a SmartServo error we cannot recover from currently.
      // Maybe a fix would be to check those variables because calling the wait.

      if (!moveSuccess) {
        ROS_WARN("Waiting 1 s because something went wrong.");
        ros::Duration(1.0).sleep();
      }
    }
  }
  
  if (itemIsSuctioned)
  {
    ROS_INFO("Item was reported as suctioned.");
    setLED("blue");
  }
  else if (itemIsGrasped)
  {
    ROS_INFO("Item was reported as grasped.");
    setLED("blue");
  }
  else if (cupIsTouchingSomething)
  {
    ROS_INFO("The cup has collided with something.");
  }
  else if (gripperIsTouchingSomething)
  {
    ROS_INFO("The gripper has collided with something.");
  }

  // It is unconfirmed if this is really necessary
  if (DEBUGFLAG) {ros::Duration(1).sleep();}

  // 3) Go back up to start position
  ROS_INFO_STREAM("PickUpItem_3) Moving back to close above the container: " << pose_close_above_container.position.x << ", " << pose_close_above_container.position.y << ", " << pose_close_above_container.position.z);
  
  moveSuccess = moveEETo(pose_close_above_container, gripper_EE_is_used);
  if (!moveSuccess) {setLinSpeed(); return false;}
  if (DEBUGFLAG) {ros::Duration(1).sleep();}

  // 4) If successful, lift the item by moving to safe position 15 cm above the container
  if (itemIsSuctioned && !gripper_EE_is_used)
  {
    // ROS_INFO("PickUpItem_4) Lifting sucked item above container (while maintaining orientation)");
    return true;
  }

  ROS_WARN("No item was successfully picked. Stopping at container + 5 cm height.");
  setLinSpeed();
  return false;
}

bool KukaMotionNode::moveEETo(geometry_msgs::Pose target_pose, bool use_gripper_EE, bool use_lin_motion, bool wait, bool use_moveit_planning)
{
  publishPoseMarker(target_pose, "target_robot_pose");

  if (kuka_is_dead)
  {
    ROS_ERROR_STREAM("moveEETo was called (use_gripper_EE: " << use_gripper_EE << "), but all motions are stopped.");
    ros::Duration(20).sleep();
    return false;
  }

  if (use_moveit_planning)
  {
    // /////// This uses MoveIt for planning, but not for moving the robot
    bool pose_is_reachable;
    geometry_msgs::Pose flange_pose = getFlangePoseAtEEPose(target_pose, use_gripper_EE, pose_is_reachable);
    if (!pose_is_reachable)
    {
      ROS_ERROR_STREAM("moveEETo received request to move " << (use_gripper_EE ? "gripper" : "suction") << ", but aborted. Pose is unreachable: " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ".");
      return false;
    }
    bool response = false;
    publishPoseMarker(flange_pose, "target_robot_pose_flange");

    geometry_msgs::PoseStamped flange_ps;
    my_iiwa_ros_object.getCartesianPose(flange_ps);
    flange_ps.pose = flange_pose;
    if (wait) 
    {
      return sendLinMotionUntilNearTarget(flange_ps);
    }
    else
    {
      my_iiwa_ros_object.setCartesianPoseLin(flange_ps);
      return 1;
    }  
  }
  else
  {
    /////// This uses the KUKA API and TF, and rotates before moving
    // First we have to change the orientation of the robot if necessary
    geometry_msgs::PoseStamped ps_start;
    my_iiwa_ros_object.getCartesianPose(ps_start);
    if (quaternionDistance(ps_start.pose.orientation, target_pose.orientation) > .05)
    {
      ps_start.pose.orientation = target_pose.orientation;
      my_iiwa_ros_object.setCartesianPose(ps_start);
      ROS_INFO("EE orientation was found to be different from target. Rotating."); // TODO: Do this properly!!!
      waitUntilArrived(1);
    }
    ros::spinOnce();

    bool response = false;
    geometry_msgs::PoseStamped flange_ps;
    my_iiwa_ros_object.getCartesianPose(flange_ps);
    if (use_gripper_EE)
    {
      transformPositionFromGripperTipToFlange(target_pose, flange_ps.pose);
    }
    else
    {
      transformPositionFromSuctionCupToFlange(target_pose, flange_ps.pose);
    }
    publishPoseMarker(flange_ps.pose, "target_robot_pose_flange");

    if (wait) 
    {
      return sendLinMotionUntilNearTarget(flange_ps);
    }
    else
    {
      my_iiwa_ros_object.setCartesianPoseLin(flange_ps);
      return 1;
    }
  }
}

bool KukaMotionNode::moveGripperTo(geometry_msgs::Pose target_pose, bool use_lin_motion, bool wait, bool use_moveit_planning)
{
  return moveEETo(target_pose, 1, use_lin_motion, wait, use_moveit_planning);
}

bool KukaMotionNode::moveSuctionCupTo(geometry_msgs::Pose target_pose, bool use_lin_motion, bool wait, bool use_moveit_planning)
{
  return moveEETo(target_pose, 0, use_lin_motion, wait, use_moveit_planning);
}

void KukaMotionNode::publishPoseMarker(geometry_msgs::Pose pose, std::string name)
{
  tnp_msgs::ShowPose srv;
  srv.request.pose = pose;
  srv.request.name = name;
  showPoseClient.call(srv);
}

void KukaMotionNode::publishGraspPoseMarker(geometry_msgs::Quaternion pad_orientation, geometry_msgs::Point pad_point_1, geometry_msgs::Point pad_point_2)
{
  tnp_msgs::ShowGraspPose srv;
  srv.request.pad_orientation = pad_orientation;
  srv.request.pad_point_1 = pad_point_1;
  srv.request.pad_point_2 = pad_point_2;
  showGraspPoseClient.call(srv);
}

void KukaMotionNode::setLED(std::string color)
{
  iiwa_msgs::LED srv;
  if (color.compare("blue"))
  {
    srv.request.led_blue = true;
  }
  else if (color.compare("green"))
  {
    srv.request.led_green = true;
  }
  else if (color.compare("red"))
  {
    srv.request.led_red = true;
  }
  else if (color.compare("yellow"))
  {
    srv.request.led_red = true;
    srv.request.led_green = true;
  }
  else if (color.compare("cyan"))
  {
    srv.request.led_blue = true;
    srv.request.led_green = true;
  }
  else if (color.compare("purple"))
  {
    srv.request.led_blue = true;
    srv.request.led_red = true;
  }
  else if (color.compare("white"))
  {
    srv.request.led_blue = true;
    srv.request.led_red = true;
    srv.request.led_green = true;
  }
  else if (color.compare("off"))
  {
    srv.request.led_blue = false;
    srv.request.led_red = false;
    srv.request.led_green = false;
  }
  // If no match --> LEDs turn off.

  ROS_WARN_STREAM("Tried to turn LED to " << color << ", but LEDs are not fully working until we fix KUKA ^_^");
  LEDClient.call(srv);
}

bool KukaMotionNode::StartSuction(const int& force/*from 0 (min) to 40 (max)*/)
{
  tnp_end_effector::Suction srv;
  srv.request.setSuctionState = true;
  srv.request.suction_force.data = force;
  if (suctionClient.call(srv))
  {
    ROS_INFO("SuctionService is being called");
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/suction Start the suction");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionService");
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/suction Error when starting the suction");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/suction Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::SetSuctionRotation(const double& rotation /*from 0 to M_PI/2 [rad]*/)
{
  ROS_INFO("Setting suction rotation");
  tnp_end_effector::SuctionRotation srv;
  srv.request.angle_in_rad.data = rotation;
  if (suctionRotationClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/suction_rotation Set suction rotation");
    ROS_INFO("SuctionRotationService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/suction_rotation Set suction rotation");
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionRotationService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/suction_rotation Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::StopSuction()
{
  ROS_INFO("Stopping suction");
  tnp_end_effector::Suction srv;
  srv.request.setSuctionState = false;
  srv.request.suction_force.data = 40;
  if (suctionClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/suction Stop the suction");
    ROS_INFO("SuctionService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/suction Error when stopping the suction");
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/suction Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::AdvanceSuction()
{
  ROS_INFO("Advancing Suction");
  tnp_end_effector::LinActuatorSuction srv;
  srv.request.setLinActuatorState = true;
  if (LinActuatorSuctionClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/LinActuatorSuction Calling advance linear actuator");
    ROS_INFO("LinActuatorSuctionService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/LinActuatorSuction Error in advancing linear actuator suction");
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorSuctionService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/LinActuatorSuction Service called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::RetractSuction()
{
  ROS_INFO("Retracting Suction");
  tnp_end_effector::LinActuatorSuction srv;
  srv.request.setLinActuatorState = false;
  if (LinActuatorSuctionClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/LinActuatorSuction Calling retract linear actuator");
    ROS_INFO("LinActuatorSuctionService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/LinActuatorSuction Error in retracting linear actuator");
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorSuctionService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/LinActuatorSuction Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::CloseGripperRelativeForceControl(const int& force /*from 0 to 100 [%]*/)
{
  ROS_INFO("Closing gripper with force control");
  tnp_end_effector::Gripper srv;
  srv.request.gripper_control_method.data = "force_control";
  srv.request.gripper_control_parameter.data = force;
  if (gripperClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/Gripper Closing gripper");
    ROS_INFO("GripperService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/Gripper Error when closing gripper");
    ROS_ERROR("Failed to call service tnp_end_effector/GripperService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/Gripper Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::SetGripperOpeningWidth(const int& width /*from 0 to 110 [mm]*/)
{
  ROS_INFO("Setting gripper width with position control");
  tnp_end_effector::Gripper srv;
  srv.request.gripper_control_method.data = "position_control";
  srv.request.gripper_control_parameter.data = width;
  if (gripperClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/Gripper Closing gripper");
    ROS_INFO("GripperService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/Gripper Error when closing gripper");
    ROS_ERROR("Failed to call service tnp_end_effector/GripperService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/Gripper Called successfully");
  ROS_INFO("done");
  return true;
}


bool KukaMotionNode::AdvanceGripper()
{
  ROS_INFO("Advancing gripper");
  tnp_end_effector::LinActuatorGripper srv;
  srv.request.setLinActuatorState = true;
  if (LinActuatorGripperClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/LinActuatorGripper Calling advance linear actuator");
    ROS_INFO("LinActuatorGripperService is being called");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/LinActuatorGripper Error in advancing linear actuator");
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorGripperService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/LinActuatorGripper Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::RetractGripper()
{
  ROS_INFO("Retracting gripper");
  tnp_end_effector::LinActuatorGripper srv;
  srv.request.setLinActuatorState = false;
  if (LinActuatorGripperClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/LinActuatorGripper Calling retract linear actuator");
    ROS_INFO("LinActuatorGripperService was called successfully");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/LinActuatorGripper Error in retracting linear actuator");
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorGripperService");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/LinActuatorGripper Called successfully");
  ROS_INFO("done");
  return true;
}

bool KukaMotionNode::setEEStatus(bool gripper_closed, bool suction_on, bool gripper_extended, bool suction_extended,
                  int gripper_control_param, int suction_force)
{
  tnp_end_effector::EEControl srv;
  srv.request.GripperClose = gripper_closed;
  srv.request.SuctionOn = suction_on;
  srv.request.lin_act_suction_extended = suction_extended;
  srv.request.lin_act_gripper_extended = gripper_extended;

  std_msgs::Int16 gripper_control_param_msg, suction_force_msg;
  gripper_control_param_msg.data = gripper_control_param;
  suction_force_msg.data = suction_force;
  srv.request.gripper_control_parameter = gripper_control_param_msg;
  srv.request.suction_force = suction_force_msg;

  if (EEControlClient.call(srv))
  {
    ROS_INFO("SuctionService is being called");
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_end_effector/suction Start the suction");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionService");
    ROS_DEBUG("[TNP_STATE L3] Error tnp_end_effector/suction Error when starting the suction");
    return false;
  }
  ros::spinOnce();
  ROS_DEBUG("[TNP_STATE L3] Done tnp_end_effector/suction Called successfully");
  ROS_INFO("done");
  return true;
}

// Service definitions

bool KukaMotionNode::calibrationMotionCallback(tnp_kuka_motion::calibrationMotion::Request &req, tnp_kuka_motion::calibrationMotion::Response &res)
{
  ROS_INFO("calibrationMotionCallback was called. This function is not finished and mostly used for testing.");
  bool response = false;

  // Same numbering as target_container enumerator:
  // 0 = tote
  // 1 = bin A
  // 2 = bin B
  // 3 = bin C
  // 4 = amnesty
  // 5 = box 1
  // 6 = box 2
  // 7 = box 3

  // Defined with reference to "home" position in world.
  geometry_msgs::Quaternion suction_front, suction_back, suction_left, suction_right, suction_cup_orientation;

  tf::Quaternion q_bin_A = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);
  tf::Quaternion q_rotate_left = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);
  tf::Quaternion q_rotate_right = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI);
  tf::Quaternion q_rotate_right_80 = tf::createQuaternionFromRPY(0.0, 0.0, (-80.0/180.0)*M_PI);
  tf::Quaternion q_container;

  std::string container_name;

  if (req.motionNumber == 0)
  {
    container_name = "tote";
    q_container = q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 1)
  {
    container_name = "bin_A";
    q_container = q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 2)
  {
    container_name = "bin_B";
    q_container = q_rotate_left * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 3)
  {
    container_name = "bin_C";
    q_container = q_rotate_left * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 4)
  {
    container_name = "amnesty";
    q_container = q_rotate_right * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 5)
  {
    container_name = "box_1";
    q_container = q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 6)
  {
    container_name = "box_2";
    q_container = q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 7)
  {
    container_name = "box_3";
    q_container = q_rotate_right_80 * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, suction_cup_orientation);
  }
  else if (req.motionNumber == 8)     // Temporary test for the orientation in bin A.
  {
    geometry_msgs::Pose p;
    p.position.x = .55;
    p.position.y = .0;
    p.position.z = -.1;
    p.orientation = getTiltedEEOrientation(p.position.x, p.position.y, true);
    ROS_INFO("Moving to front.");
    moveSuctionCupTo(p);
    ros::Duration(.5).sleep();

    p.position.y = -.3;
    p.orientation = getTiltedEEOrientation(p.position.x, p.position.y, true);
    moveSuctionCupTo(p);
    ROS_INFO("Moving to right.");
    moveSuctionCupTo(p);
    ros::Duration(3).sleep();

    p.position.y = .3;
    p.orientation = getTiltedEEOrientation(p.position.x, p.position.y, true);
    moveSuctionCupTo(p);
    ROS_INFO("Moving to left.");
    moveSuctionCupTo(p);
    ros::Duration(3).sleep();
    return true;
  }
  else if (req.motionNumber == 9)     // Test if cartesianLIN works with iiwa_ros by moving forward a bit.
  {
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps);
    my_iiwa_ros_object.setCartesianPoseLin(ps);

    return waitUntilArrived(1);
  }
  else if (req.motionNumber == 10)     // Test if regular cartesian motion works with iiwa_ros by moving forward a bit.
  {
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPose with poseStamped:" << ps);
    my_iiwa_ros_object.setCartesianPose(ps);

    return waitUntilArrived(1);
  }
  else if (req.motionNumber == 11)     // Test if the new stop() motion works without causing a machine violation error
  {
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    ps.pose.position.x = .6;
    ps.pose.position.y = -.3;
    ps.pose.position.z = .55;
    ROS_INFO_STREAM("Going to start pose:" << ps);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    ps.pose.position.y = .3;
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    return true;
  }
  else if (req.motionNumber == 12)     // Test if cartesianLIN works with iiwa_ros by moving down a bit.
  {
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    ps.pose.position.z -= .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps);
    my_iiwa_ros_object.setCartesianPoseLin(ps);

    return waitUntilArrived(1);
  }
  else if (req.motionNumber == 13)     // Test if cartesianLIN works with iiwa_ros by moving down/forward diagonally.
  {
    geometry_msgs::PoseStamped ps;
    my_iiwa_ros_object.getCartesianPose(ps);
    ps.pose.position.x += .1;
    ps.pose.position.z -= .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps);
    my_iiwa_ros_object.setCartesianPoseLin(ps);

    return waitUntilArrived(1);
  }
  else if (req.motionNumber == 14)     // LIN Motion Testing Suite. Go to Bin C, do 2 lin motions, go back PTP, do 1 lin motion, change velocity limit, do 1 more lin motion, then the reverse
  {
    geometry_msgs::PoseStamped ps;
    moveToJointAnglesPTP(atBinC);

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    moveToJointAnglesPTP(atBinC);

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    ROS_INFO_STREAM("Reducing speed to 50 mm/s");
    setLinSpeed(.05, .05, .05);
    ros::Duration(0.5).sleep();

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    moveToJointAnglesPTP(atBinC);

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    ROS_INFO_STREAM("Lifting speed limit");
    setLinSpeed();
    ros::Duration(0.5).sleep();

    my_iiwa_ros_object.getCartesianPose(ps);
    ros::Duration(0.5).sleep();
    ps.pose.position.x += .1;
    ROS_INFO_STREAM("Calling CartesianPoseLin with poseStamped:" << ps.pose.position);
    my_iiwa_ros_object.setCartesianPoseLin(ps);
    waitUntilArrived(1);

    moveToJointAnglesPTP(atBinC);

    return true;
  }
  else if (req.motionNumber == 15)     // Go to the calibration pose for the rec space
  {
    ROS_INFO_STREAM("Going to the recognition space calibration position (looking at the cube).");
    moveToJointAnglesPTP(lookAtRecSpaceCube);
    return waitUntilArrived();
  }
  else if (req.motionNumber == 16)     // Only publish some markers
  {
    ROS_INFO_STREAM("Publishing two markers above bin A for rviz.");
    geometry_msgs::Pose p;
    p.position.x = .55 + rand() % 100 / 1000.0;
    p.position.y = .0 + rand() % 400 / 1000.0;
    p.position.z = .0 + rand() % 300 / 1000.0;
    p.orientation.w = 1.0;
    publishPoseMarker(p, "target_robot_pose");
    p.position.x = .55 + rand() % 100 / 1000.0;
    p.position.y = .0 + rand() % 400 / 1000.0;
    p.position.z = .0 + rand() % 300 / 1000.0;
    publishPoseMarker(p, "refused_item_pose");
    p.position.x = .55 + rand() % 100 / 1000.0;
    p.position.y = .0 + rand() % 400 / 1000.0;
    p.position.z = .0 + rand() % 300 / 1000.0;
    publishPoseMarker(p, "target_item_pose");
    return true;
  }
  else if (req.motionNumber == 17)     // Cycle through all the PTP motions and print out the cartesian poses
  {
    ROS_INFO_STREAM("Going to all PTP positions to record the cartesian pose. What a dumb way to do this.");

    std::vector<iiwa_msgs::JointPosition> jpvec;
    std::vector<std::string> namevec;

    jpvec.push_back(atHome);               namevec.push_back("atHome");
    jpvec.push_back(atTote);               namevec.push_back("atTote");
    jpvec.push_back(lookIntoTote);         namevec.push_back("lookIntoTote");
    jpvec.push_back(atAmnesty);            namevec.push_back("atAmnesty");
    jpvec.push_back(atRecSpace);           namevec.push_back("atRecSpace");
    jpvec.push_back(atBinA);               namevec.push_back("atBinA");
    jpvec.push_back(atBinB);               namevec.push_back("atBinB");
    jpvec.push_back(atBinC);               namevec.push_back("atBinC");
    jpvec.push_back(atBox1);               namevec.push_back("atBox1");
    jpvec.push_back(atBox2);               namevec.push_back("atBox2");
    jpvec.push_back(atBox3);               namevec.push_back("atBox3");
    jpvec.push_back(lookIntoBinA);         namevec.push_back("lookIntoBinA");
    jpvec.push_back(lookIntoBinA1);         namevec.push_back("lookIntoBinA1");
    jpvec.push_back(lookIntoBinA2);         namevec.push_back("lookIntoBinA2");
    jpvec.push_back(lookIntoBinB);         namevec.push_back("lookIntoBinB");
    jpvec.push_back(lookIntoBinC);         namevec.push_back("lookIntoBinC");
    jpvec.push_back(lookAtRecSpaceCube);   namevec.push_back("lookAtRecSpaceCube");

    ofstream myfile;
    myfile.open ("ptp_cartpose_log_kukamotion.txt");
    geometry_msgs::PoseStamped ps;
    for (int i = 0; i < jpvec.size(); ++i)
    {
      moveToJointAnglesPTP(jpvec[i]);
      waitUntilArrived();
      ros::Duration(0.5).sleep();
      my_iiwa_ros_object.getCartesianPose(ps);
      myfile << "Flange's cartesian pose at " << namevec[i] << ": " << std::endl;
      myfile << ps.pose;
    }

    myfile.close();
    return waitUntilArrived();
  }
  else if (req.motionNumber == 19)     // Test motions
  {
    geometry_msgs::Pose p;
    p.position.x = .65;
    p.position.y = .0;
    p.position.z = -.4;
    p.orientation = suction_front;
    putItemIntoContainer("bin_A", 0, p.position);
    return true;
  }
  else if (req.motionNumber == 20)     // Test putItemIntoContainer
  {
    geometry_msgs::Pose p;
    p.position.x = .65;
    p.position.y = .0;
    p.position.z = -.4;
    p.orientation = suction_front;
    putItemIntoContainer("bin_A", 0, p.position);
    return true;
  }


  // The "standard" calibration motion for each box

  double container_length, container_width, container_height;
  n_.param<double>("tnp_environment/"+container_name+"_w", container_width, .39);
  n_.param<double>("tnp_environment/"+container_name+"_l", container_length, .34);
  n_.param<double>("tnp_environment/"+container_name+"_h", container_height, .25);

  ROS_INFO("Doing calibration motion. Moving to 5 cm above the center, then to bottom right, and then to bottom left of container.");
  geometry_msgs::Pose abovepose, boxpose, worldpose;
  boxpose.position.y = container_length/2;
  boxpose.position.x = container_width/2;
  boxpose.position.z = container_height + .15;
  boxpose.orientation = suction_cup_orientation;
  abovepose = boxpose;
  containerPoseToWorld(container_name, abovepose, abovepose);

  ROS_INFO("Moving to above center of container.");
  moveSuctionCupTo(abovepose);
  ros::Duration(.5).sleep();
  containerPoseToWorld(container_name, boxpose, worldpose);
  worldpose.orientation = suction_cup_orientation;
  response = moveSuctionCupTo(worldpose);
  ROS_WARN_STREAM("===== SUCTION CUP SHOULD BE ABOVE CONTAINER: " << worldpose);
  setLED("white");
  getch();
  ros::Duration(2).sleep();
  setLED("off");

  ROS_INFO_STREAM("Moving suction cup to above container: " << worldpose);
  boxpose.position.y = 0.0;
  boxpose.position.x = 0.0;
  containerPoseToWorld(container_name, boxpose, worldpose);
  worldpose.orientation = suction_cup_orientation;
  response = moveSuctionCupTo(worldpose);
  ROS_WARN_STREAM("===== SUCTION CUP SHOULD BE AT BOTTOM RIGHT CORNER OF CONTAINER: " << worldpose);
  setLED("white");
  getch();
  ros::Duration(2).sleep();
  setLED("off");

  ROS_INFO("Moving to above bottom left corner of container.");
  moveSuctionCupTo(abovepose);
  ros::Duration(.5).sleep();
  boxpose.position.y = container_length;
  containerPoseToWorld(container_name, boxpose, worldpose);
  worldpose.orientation = suction_cup_orientation;
  response = moveSuctionCupTo(worldpose);
  ROS_WARN_STREAM("===== SUCTION CUP SHOULD BE AT BOTTOM LEFT CORNER OF CONTAINER: " << worldpose);
  setLED("white");
  getch();
  ros::Duration(2).sleep();
  setLED("off");


  ROS_INFO("Moving to above top left corner of container.");
  
  // Create another pose a bit closer to the robot to give room for tilting
  boxpose.position.y = container_length/2;
  boxpose.position.x = container_width/4;
  boxpose.position.z = container_height + .15;
  boxpose.orientation = suction_cup_orientation;
  abovepose = boxpose;
  containerPoseToWorld(container_name, abovepose, abovepose);
  moveSuctionCupTo(abovepose);

  ros::Duration(.5).sleep();
  boxpose.position.x = container_width;
  boxpose.position.y = container_length;
  containerPoseToWorld(container_name, boxpose, worldpose);
  worldpose.orientation = getTiltedEEOrientation(worldpose.position.x, worldpose.position.y, true);
  response = moveSuctionCupTo(worldpose);
  ROS_WARN_STREAM("===== SUCTION CUP SHOULD BE AT TOP LEFT CORNER OF CONTAINER: " << worldpose);
  setLED("white");
  getch();
  ros::Duration(2).sleep();
  setLED("off");


  return response;
}

bool KukaMotionNode::goToContainerCallback(tnp_kuka_motion::goToContainer::Request &req, tnp_kuka_motion::goToContainer::Response &res)
{
  ROS_INFO("goToContainerCallback was called");
  if (req.keep_A7_fixed) 
  {
    return goToContainerWithFixedA7(req.container_name);
  }
  else
  {
    return goToContainer(req.container_name);
  }
}


bool KukaMotionNode::goToHomeCallback(tnp_kuka_motion::goToHome::Request &req, tnp_kuka_motion::goToHome::Response &res)
{
  ROS_INFO("goToHomeCallback was called");
  bool response = moveToJointAnglesPTP(atHome);
  publishPoseMarker(atHome_pose, "target_robot_pose_flange");
  return response;
}

bool KukaMotionNode::goToToteCallback(tnp_kuka_motion::goToTote::Request &req, tnp_kuka_motion::goToTote::Response &res)
{
  ROS_INFO("goToToteCallback was called");
  ROS_WARN("This function is deprecated! Use goToContainer instead.");
  bool response = moveToJointAnglesPTP(atTote);
  publishPoseMarker(atTote_pose, "target_robot_pose_flange");
  return response;
}

bool KukaMotionNode::goToAmnestyCallback(tnp_kuka_motion::goToAmnesty::Request &req,
                                         tnp_kuka_motion::goToAmnesty::Response &res)
{
  ROS_INFO("goToAmnestyCallback was called");
  ROS_WARN("This function is deprecated! Use goToContainer instead.");
  bool response = moveToJointAnglesPTP(atAmnesty);
  publishPoseMarker(atAmnesty_pose, "target_robot_pose_flange");
  return true;
}

bool KukaMotionNode::goToRecSpaceCallback(tnp_kuka_motion::goToRecSpace::Request &req,
                                          tnp_kuka_motion::goToRecSpace::Response &res)
{
  ROS_INFO_STREAM("goToRecSpaceCallback was called with rotation angle " << req.rotation_angle_z/M_PI*180.0);

  bool response;
  if (req.rotation_angle_z == 0)
  {
    response = moveToJointAnglesPTP(atRecSpace);  
    publishPoseMarker(atRecSpace_pose, "target_robot_pose_flange");
  }
  else // rotation angle is not 0
  {
    // Make sure rotation angle makes sense
    double adjusted_angle_z = restrictValueToInterval(req.rotation_angle_z, 0, (150.0/180.0)*M_PI);
    if (adjusted_angle_z != req.rotation_angle_z)
    {
      ROS_WARN_STREAM("goToRecSpace adjusted rotation angle from " << req.rotation_angle_z << " to " << adjusted_angle_z);
    }

    tf::Quaternion q_neutral = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0), 
    q_turn = tf::createQuaternionFromRPY(0.0, 0.0, -adjusted_angle_z), 
    q_final;
    q_final = q_turn * q_neutral;

    geometry_msgs::Pose atRecSpace_pose_EE_rotated;
    atRecSpace_pose_EE_rotated.position = atRecSpace_pose_EE.position;
    tf::quaternionTFToMsg(q_final, atRecSpace_pose_EE_rotated.orientation);
    ROS_INFO_STREAM("Moving to RecSpace position rotated by " << adjusted_angle_z << " rad:" << atRecSpace_pose_EE);
    response = moveSuctionCupTo(atRecSpace_pose_EE_rotated);
    publishPoseMarker(atRecSpace_pose_EE_rotated, "target_robot_pose");
  }
  
  return response;
}

bool KukaMotionNode::goToBinCallback(tnp_kuka_motion::goToBin::Request &req, tnp_kuka_motion::goToBin::Response &res)
{
  ROS_INFO("goToBinCallback was called");
  ROS_WARN("This function is deprecated! Use goToContainer instead.");
  ROS_DEBUG_STREAM("[TNP_STATE L3] Executing tnp_kuka_motion/goToBinCallback Going to bin " << req.bin_id);
  ROS_INFO_STREAM("goToBinCallback was called. Going to bin ID " << req.bin_id.data);
  bool response;
  if (req.bin_id.data.compare("A") == 0 || req.bin_id.data.compare("bin_A") == 0)
  {
    response = moveToJointAnglesPTP(atBinA);
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (req.bin_id.data.compare("B") == 0 || req.bin_id.data.compare("bin_B") == 0)
  {
    response = moveToJointAnglesPTP(atBinB);
    publishPoseMarker(atBinB_pose, "target_robot_pose_flange");
  }
  else if (req.bin_id.data.compare("C") == 0 || req.bin_id.data.compare("bin_C") == 0)
  {
    response = moveToJointAnglesPTP(atBinC);
    publishPoseMarker(atBinC_pose, "target_robot_pose_flange");
  }
  else
  {
    ROS_DEBUG_STREAM("[TNP_STATE L3] Error tnp_kuka_motion/gotoBinNum Error when going to bin " << req.bin_id);
    ROS_ERROR_STREAM("Bin number cannot be found! Received: " << req.bin_id);
    ROS_ERROR_STREAM("Bin number cannot be found! Received: " << req.bin_id.data);
  }

  return response;
}

bool KukaMotionNode::goToBoxCallback(tnp_kuka_motion::goToBox::Request &req, tnp_kuka_motion::goToBox::Response &res)
{
  ROS_INFO("goToBoxCallback was called");
  ROS_WARN("This function is deprecated! Use goToContainer instead.");
  ROS_INFO("Going to box number %d", req.box_id);
  ROS_DEBUG("[TNP_STATE L3] Executing tnp_kuka_motion/goToBoxCallback Going to box %d", req.box_id);
  ROS_INFO_STREAM("goToBoxCallback was called. Going to box number " << req.box_id);
  bool response;
  if (req.box_id == 1)
  {
    response = moveToJointAnglesPTP(atBox1);
    publishPoseMarker(atBox1_pose, "target_robot_pose_flange");
  }
  else if (req.box_id == 2)
  {
    response = moveToJointAnglesPTP(atBox2);
    publishPoseMarker(atBox2_pose, "target_robot_pose_flange");
  }
  else if (req.box_id == 3)
  {
    response = moveToJointAnglesPTP(atBox3);
    publishPoseMarker(atBox3_pose, "target_robot_pose_flange");
  }

  return response;
}

bool KukaMotionNode::goToLookIntoContainerCallback(tnp_kuka_motion::goToLookIntoContainer::Request &req,
                                              tnp_kuka_motion::goToLookIntoContainer::Response &res)
{
  ROS_INFO("goToLookIntoContainerCallback was called");
  return goToLookIntoContainer(req.container_name, req.height);
}


bool KukaMotionNode::goToLookIntoToteCallback(tnp_kuka_motion::goToLookIntoTote::Request &req,
                                              tnp_kuka_motion::goToLookIntoTote::Response &res)
{
  ROS_INFO("goToLookIntoToteCallback was called");
  ROS_WARN("This function is deprecated! Use goToLookIntoContainerCallback instead!");

  int height = req.height;
  if (req.height == 0) {
    ROS_WARN_STREAM("No height was set for the goToLookAt service. Setting to 1 (high).");
    height = 1;
  }

  bool response = false;
  if (height == 1) {response = moveToJointAnglesPTP(lookIntoToteHigh);}
  else if (height == 2) {response = moveToJointAnglesPTP(lookIntoToteMid);}
  else if (height == 3) {response = moveToJointAnglesPTP(lookIntoToteLow);}
  publishPoseMarker(lookIntoTote_pose, "target_robot_pose_flange");
  return response;
}

bool KukaMotionNode::goToLookIntoAmnestyCallback(tnp_kuka_motion::goToLookIntoAmnesty::Request &req,
                                                 tnp_kuka_motion::goToLookIntoAmnesty::Response &res)
{
  ROS_INFO("goToLookIntoAmnestyCallback was called");
  ROS_WARN("This function is deprecated! Use goToLookIntoContainerCallback instead!");
  ROS_ERROR_STREAM("No height was set for the goToLookAt service. Going to the default position.");
  return moveToJointAnglesPTP(lookIntoAmnesty);
}

bool KukaMotionNode::goToLookIntoBinCallback(tnp_kuka_motion::goToLookIntoBin::Request &req,
                                             tnp_kuka_motion::goToLookIntoBin::Response &res)
{
  ROS_INFO("goToLookIntoBinCallback was called");
  ROS_WARN("This function is deprecated! Use goToLookIntoContainerCallback instead!");
  ROS_INFO_STREAM("Moving to look at bin ID " << req.bin_id);
  ROS_DEBUG_STREAM("[TNP_STATE L3] Executing tnp_kuka_motion/goToLookIntoBinCallback Going to look item into bin " << req.bin_id );

  bool response = false;
  if (req.bin_id.data.compare("A") == 0)
  {
    if (req.height == 0){ response = moveToJointAnglesPTP(lookIntoBinA); }
    if (req.height == 1){ response = moveToJointAnglesPTP(lookIntoBinAhigh); }
    if (req.height == 2){ response = moveToJointAnglesPTP(lookIntoBinAlow); }
    publishPoseMarker(lookIntoBinA_pose, "target_robot_pose_flange");
  }
  else if (req.bin_id.data.compare("B") == 0)
  {
    ROS_WARN_STREAM("No heights are implemented for this bin.");
    response = moveToJointAnglesPTP(lookIntoBinB);
    publishPoseMarker(lookIntoBinB_pose, "target_robot_pose_flange");
  }
  else if (req.bin_id.data.compare("C") == 0)
  {
    ROS_WARN_STREAM("No heights are implemented for this bin.");
    response = moveToJointAnglesPTP(lookIntoBinC);
    publishPoseMarker(lookIntoBinC_pose, "target_robot_pose_flange");
  }
  else
  {
    ROS_DEBUG_STREAM("[TNP_STATE L3] Error tnp_kuka_motion/goToLookIntoBinCallback Error when going to look item into bin " << req.bin_id );
    ROS_INFO_STREAM("goToLookIntoBinCallback error: this bin has not been set up: " << req.bin_id);
  }
  return response;
}


bool KukaMotionNode::goToLookIntoBoxCallback(tnp_kuka_motion::goToLookIntoBox::Request &req,
                                             tnp_kuka_motion::goToLookIntoBox::Response &res)
{
  ROS_INFO_STREAM("Moving to look at box number " << req.box_id);
  ROS_WARN("This function is deprecated! Use goToLookIntoContainerCallback instead!");
  ROS_DEBUG_STREAM("[TNP_STATE L3] Executing tnp_kuka_motion/goToLookIntoBoxCallback Going to look item into box " << req.box_id );
  ROS_INFO("goToLookIntoBoxCallback was called");

  bool response = false;
  if (req.box_id == 1)
  {
    response = moveToJointAnglesPTP(lookIntoBox1);
    publishPoseMarker(lookIntoBox1_pose, "target_robot_pose_flange");
  }
  if (req.box_id == 2)
  {
    response = moveToJointAnglesPTP(lookIntoBox2);
    publishPoseMarker(lookIntoBox2_pose, "target_robot_pose_flange");
  }
  if (req.box_id == 3)
  {
    response = moveToJointAnglesPTP(lookIntoBox3);
    publishPoseMarker(lookIntoBox3_pose, "target_robot_pose_flange");
  }
  else
  {
    ROS_DEBUG_STREAM("[TNP_STATE L3] Error tnp_kuka_motion/goToLookIntoBoxCallback Error when going to look item into box " << req.box_id );
    ROS_INFO_STREAM("goToLookIntoBoxCallback error: box number %d has not been setup" << req.box_id);
  }
  return response;
}

bool KukaMotionNode::canRobotPlaceItemHereCallback(tnp_kuka_motion::canRobotPlaceItemHere::Request &req,
                                            tnp_kuka_motion::canRobotPlaceItemHere::Response &res)
{
  ROS_INFO_STREAM("canRobotPlaceItemHereCallback was called. Checking: " << req.container_name << " with input pose: " << req.EE_target_pose.position);
  int calculation_mode = 0;
  res.motion_is_possible = canRobotPlaceItemHere(req.EE_target_pose, req.container_name, req.use_gripper_EE, req.calculation_mode);
  return true;
}

bool KukaMotionNode::putItemIntoContainerCallback(tnp_kuka_motion::putItemIntoContainer::Request &req,
                                            tnp_kuka_motion::putItemIntoContainer::Response &res)
{
  ROS_INFO_STREAM("putItemIntoContainerCallback was called. Going to: " << req.container_name << " with target_pose " << req.target_pose_in_world.position);
  return putItemIntoContainer(req.container_name, req.use_gripper_EE, req.target_pose_in_world.position, req.rotation_angle_z);
}

bool KukaMotionNode::putItemIntoToteCallback(tnp_kuka_motion::putItemIntoTote::Request &req,
                                            tnp_kuka_motion::putItemIntoTote::Response &res)
{
  ROS_WARN("putItemIntoToteCallback was called, but is deprecated. Use putItemIntoContainer.");

  geometry_msgs::Point box_point;
  geometry_msgs::Pose target_pose;
  double container_w, container_l;
  n_.getParam("tnp_environment/tote_w", container_w);
  n_.getParam("tnp_environment/tote_l", container_l);

  box_point.x = 0.5 * container_w;
  box_point.y = 0.5 * container_l;
  box_point.z = .2;

  target_pose.position = containerPointToWorld("tote", box_point, tflistener);
  target_pose.orientation.x = 1.0;

  return putItemIntoContainer("tote", req.use_gripper_EE, target_pose.position);
}

bool KukaMotionNode::putItemIntoAmnestyCallback(tnp_kuka_motion::putItemIntoAmnesty::Request &req,
                                            tnp_kuka_motion::putItemIntoAmnesty::Response &res)
{
  ROS_WARN_STREAM("putItemIntoAmnestyCallback was called, but is deprecated. Use putItemIntoContainer.");
  geometry_msgs::Point box_point;
  geometry_msgs::Pose target_pose;
  double container_w, container_l;

  n_.getParam("tnp_environment/amnesty_w", container_w);
  n_.getParam("tnp_environment/amnesty_l", container_l);

  box_point.x = 0.5 * container_w;
  box_point.y = 0.5 * container_l;
  box_point.z = .1;

  target_pose.position = containerPointToWorld("amnesty", box_point, tflistener);
  target_pose.orientation.x = 1.0;

  return putItemIntoContainer("amnesty", req.use_gripper_EE, target_pose.position);
}

bool KukaMotionNode::putItemIntoBinCallback(tnp_kuka_motion::putItemIntoBin::Request &req,
                                            tnp_kuka_motion::putItemIntoBin::Response &res)
{
  ROS_WARN("THIS IS DEPRECATED. Passing on to putItemIntoContainer.");
  ROS_DEBUG_STREAM("[TNP_STATE L3] Executing tnp_kuka_motion/putItemIntoBinNum Putting item into bin " << req.bin_id );
  
  std::stringstream binname;
  binname.clear();
  binname << "box_" << req.bin_id;
  geometry_msgs::Pose defaultpose = makePose();
  return putItemIntoContainer(binname.str(), req.use_gripper_EE, defaultpose.position);
}

bool KukaMotionNode::putItemIntoBoxCallback(tnp_kuka_motion::putItemIntoBox::Request &req,
                                            tnp_kuka_motion::putItemIntoBox::Response &res)
{
  ROS_ERROR("THIS IS DEPRECATED. USE putItemIntoContainer! This function uses a fixed pose to place the item.");

  ROS_INFO_STREAM("putItemIntoBoxCallback was called. Going to box number " << req.box_id);
  bool response;

  std::stringstream boxname;
  boxname.clear();
  boxname << "box_" << req.box_id;
  geometry_msgs::Pose defaultpose = makePose();
  return putItemIntoContainer(boxname.str(), req.use_gripper_EE, defaultpose.position);
}

bool KukaMotionNode::moveToJointAnglesPTPCallback(tnp_kuka_motion::moveToJointAnglesPTP::Request &req,
                                                  tnp_kuka_motion::moveToJointAnglesPTP::Response &res)
{
  ROS_INFO("moveToJointAnglesPTPCallback was called");

  bool response = moveToJointAnglesPTP(req.a1, req.a2, req.a3, req.a4, req.a5, req.a6, req.a7);
  res.motionComplete = response;
  return response;
}

bool KukaMotionNode::moveToCartPosePTPCallback(tnp_kuka_motion::moveToCartPosePTP::Request &req,
                                               tnp_kuka_motion::moveToCartPosePTP::Response &res)
{
  ROS_INFO("moveToCartPosePTPCallback was called");

  bool response = moveToCartPosePTP(req.target_pose);
  res.motionComplete = response;
  return response;
}

bool KukaMotionNode::moveSuctionCupToCallback(tnp_kuka_motion::moveSuctionCupTo::Request &req,
                                               tnp_kuka_motion::moveSuctionCupTo::Response &res)
{
  return moveSuctionCupTo(req.target_pose, req.use_lin_motion);
}

bool KukaMotionNode::moveGripperToCallback(tnp_kuka_motion::moveGripperTo::Request &req,
                                               tnp_kuka_motion::moveGripperTo::Response &res)
{
  return moveGripperTo(req.target_pose, req.use_lin_motion);
}

bool KukaMotionNode::sweepToteHorizontalCallback(tnp_kuka_motion::sweepToteHorizontal::Request &req, tnp_kuka_motion::sweepToteHorizontal::Response &res)
{
  ROS_INFO_STREAM("sweepToteHorizontal was called");

  double tote_width_x, tote_width_y, tote_h;
  n_.getParam("tnp_environment/tote_w", tote_width_x);    // Width of the inside of the tote
  n_.getParam("tnp_environment/tote_l", tote_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/tote_h", tote_h);          // tote height

  double x_dist_to_border = .015;
  double sweep_height = req.height;
  if (req.height < 0.03)
  {
    ROS_WARN_STREAM("Heights below 0.03 are not safe for horizontal sweeping! Setting to " << .03);
    sweep_height = .03;
  }
  else if (req.height > tote_h)
  {
    ROS_WARN_STREAM("Sweeping above the container may not be safe for the joints. Limited to " << tote_h);
    sweep_height = tote_h;
  }

  tf::Quaternion q_start = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);
  tf::Quaternion q_turn_to_tote = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI); 
  tf::Quaternion q_tote_neutral = q_turn_to_tote*q_start;
  double tiltangle, far_turn_angle;
  if (req.sweepToLeft)  
  { tiltangle = -30.0;
    far_turn_angle = -30.0;
  }
  else  // sweep to the right
  { 
    tiltangle = 30.0; 
    far_turn_angle = tiltangle;
  }
  tf::Quaternion q_tilt = tf::createQuaternionFromRPY(0.0, (tiltangle/180.0)*M_PI, 0.0); 
  tf::Quaternion q_extra_turn_for_far_places = tf::createQuaternionFromRPY(0.0, 0.0, (far_turn_angle/180.0)*M_PI); 
  tf::Quaternion q_final;
  geometry_msgs::Pose start_pose, end_pose;


  // Parametrize the motion
  geometry_msgs::Point start_point_container, start_point_world, end_point_container, end_point_world;
  double start_pos_y_offset = .05;
  if (req.sweepToLeft) {start_pos_y_offset = .1;}
  if (req.sweepToLeft)
  {
    start_point_container.y = start_pos_y_offset + abs(sin((tiltangle/180.0)*M_PI)* (tote_h-sweep_height));
    end_point_container.y = tote_width_y;
  }
  else
  {
    start_point_container.y = tote_width_y - start_pos_y_offset - abs(sin((tiltangle/180.0)*M_PI)* (tote_h-sweep_height));
    end_point_container.y = 0.0;
  }
  start_point_container.z = sweep_height;
  end_point_container.z = start_point_container.z;
  

  // Execute the motion in a loop
  setEEStatus(1, 1, 0, 1, 100, 40);

  setLinSpeed(.12, .12, .12, 10, 10, 10);
  ros::Duration(1.0).sleep();
  int numcycles = 5;
  double x_interval = (tote_width_x - 2*x_dist_to_border) / (numcycles-1);

  // Initial declaration of previous_orientation
  q_final = q_tilt*q_tote_neutral;
  if (req.sweepToLeft) {q_final = q_tilt * tf::createQuaternionFromRPY(0.0, 0.0, (120/180.0)*M_PI) * q_tote_neutral;}
  geometry_msgs::Quaternion previous_orientation;
  tf::quaternionTFToMsg(q_final, previous_orientation);

  // The sweep movements
  bool there_was_no_contact = true;
  for (int i = 0; i<numcycles; i++)
  {
    start_point_container.x = x_dist_to_border + i*x_interval;
    end_point_container.x = start_point_container.x;
    start_pose.position = containerPointToWorld("tote", start_point_container, tflistener);
    end_pose.position = containerPointToWorld("tote", end_point_container, tflistener);
    q_final = q_tilt*q_tote_neutral;
    if (req.sweepToLeft) {q_final = q_tilt * tf::createQuaternionFromRPY(0.0, 0.0, (120/180.0)*M_PI) * q_tote_neutral;}
    if (start_point_container.x >= tote_width_x * .3)
    {
      q_final = q_extra_turn_for_far_places * q_tilt*q_tote_neutral;
      if (req.sweepToLeft) {q_final = q_extra_turn_for_far_places * q_tilt * tf::createQuaternionFromRPY(0.0, 0.0, (-120/180.0)*M_PI) * q_tote_neutral;}
    }

    tf::quaternionTFToMsg(q_final, start_pose.orientation);
    tf::quaternionTFToMsg(q_final, end_pose.orientation);

    if ((quaternionDistance(start_pose.orientation, previous_orientation) > 0.01)   // If the end effector will have to tilt
      && (req.sweepToLeft))
    {
      // Do a movement upwards and turn A7 to avoid axis lockup. to tilt safely
      geometry_msgs::Pose higher_pose;
      higher_pose.position = start_pose.position;
      higher_pose.position.y += .1;
      higher_pose.position.z += .1;
      higher_pose.orientation = previous_orientation;
      moveSuctionCupTo(higher_pose);
      waitUntilArrived();

      iiwa_msgs::JointPosition jp;
      my_iiwa_ros_object.getJointPosition(jp);
      jp.position.a7 = 1.0;
      my_iiwa_ros_object.setJointPosition(jp);
      waitUntilArrived();

      higher_pose.orientation = start_pose.orientation;
      moveSuctionCupTo(higher_pose, true, false);
      waitUntilArrived(1,1,1,1);
    }

    moveSuctionCupTo(start_pose, true, false);
    there_was_no_contact = waitUntilArrived2(1,1,1,1);
    if (req.palpation_mode && !there_was_no_contact) {break;}
    if (itemIsSuctioned)
    {
      end_pose.position = getPointHighAboveContainer("tote", .15);
      moveSuctionCupTo(end_pose,1,1,0);
      res.success = true;
      return true;
    }

    moveSuctionCupTo(end_pose, true, false);
    there_was_no_contact = waitUntilArrived2(1,1,1,1);
    if (req.palpation_mode && !there_was_no_contact) {break;}
    if (itemIsSuctioned)
    {
      end_pose.position = getPointHighAboveContainer("tote", .15);
      moveSuctionCupTo(end_pose,1,1,0);
      res.success = true;
      return true;
    }
    // TODO: If only contact, then move towards the middle of the container (to move the item there))
    moveSuctionCupTo(start_pose,1,1,0);
    previous_orientation = start_pose.orientation;
  }
    
  if (req.palpation_mode && !there_was_no_contact) 
  {
    geometry_msgs::Pose p_contact = getCurrentSuctionCupPose();

    // Move out first
    start_pose.position = getPointHighAboveContainer("tote", .15);
    moveSuctionCupTo(start_pose,1,1,0);
    goToLookIntoContainer("tote", 0);
    
    if (req.sweepToLeft) {p_contact.position.x += .04;}
    else {p_contact.position.x -= .04;}
    tnp_kuka_motion::suckItem::Request suckReq;
    tnp_kuka_motion::suckItem::Response suckRes;
    suckReq.fuzzyMode = true;
    suckReq.target_pose = p_contact;
    std_msgs::Int16 force;
    force.data = 40.0;
    suckReq.force = force;
    ROS_INFO_STREAM("Detected contact. Trying suckItem at: " << p_contact.position.x << ", " << p_contact.position.y << ", " << p_contact.position.z );
    return suckItemCallback(suckReq, suckRes);
  }

  setLinSpeed();
  ros::Duration(1.0).sleep();
  start_pose.position = getPointHighAboveContainer("tote", .15);
  moveSuctionCupTo(start_pose,1,1,0);
  res.success = false;
  return false;
}

bool KukaMotionNode::sweepToteCornersCallback(tnp_kuka_motion::sweepToteCorners::Request &req, tnp_kuka_motion::sweepToteCorners::Response &res)
{
  ROS_INFO_STREAM("sweepToteCorners was called");
  double x_dist_to_border = -.05;

  tf::Quaternion q_bin_A_neutral = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);   
  tf::Quaternion q_turn_to_tote = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI); 
  tf::Quaternion q_tote_neutral = q_turn_to_tote*q_bin_A_neutral;
  tf::Quaternion q_bottom_right = tf::createQuaternionFromRPY(0.0, 0.0, (135.0/180.0)*M_PI) * q_tote_neutral; 
  tf::Quaternion q_bottom_left = tf::createQuaternionFromRPY(0.0, 0.0, (45.0/180.0)*M_PI) * q_tote_neutral; 

  double tote_width_x, tote_width_y, tote_h, tote_floor_z;
  n_.getParam("tnp_environment/tote_w", tote_width_x);    // Width of the inside of the tote
  n_.getParam("tnp_environment/tote_l", tote_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/tote_h", tote_h);          // tote height
  n_.getParam("tnp_environment/tote_z", tote_floor_z);          // tote floor z-coordinate

  // Define the points
  // This assumes that the tote side with the empty corners is on the left side of the robot
  geometry_msgs::Point 
  totepoint_bottom_left =   makePoint(0.01,               tote_width_y + .01, 0.01), 
  totepoint_top_left =      makePoint(tote_width_x,       tote_width_y + .01, 0.01), 
  totepoint_top_right =     makePoint(tote_width_x-0.06,  -0.01,              0.01), 
  totepoint_bottom_right =  makePoint(0.06,               -0.01,              0.01);

  geometry_msgs::Pose pose_bottom_left, pose_top_left, pose_top_right, pose_bottom_right;
  pose_bottom_left.position = containerPointToWorld("tote", totepoint_bottom_left, tflistener);
  pose_top_left.position = containerPointToWorld("tote", totepoint_top_left, tflistener);
  pose_top_right.position = containerPointToWorld("tote", totepoint_top_right, tflistener);
  pose_bottom_right.position = containerPointToWorld("tote", totepoint_bottom_right, tflistener);
  
  tf::quaternionTFToMsg(q_bottom_left, pose_bottom_left.orientation);
  tf::quaternionTFToMsg(q_bottom_right, pose_bottom_right.orientation);
  pose_top_left.orientation = getTiltedEEOrientation(pose_top_left.position.x, pose_top_left.position.y, true);
  pose_top_right.orientation = getTiltedEEOrientation(pose_top_right.position.x, pose_top_right.position.y, true);
  
  // Execute the motions
  setEEStatus(1, 1, 0, 1, 100, 40);

  geometry_msgs::Pose pose_above;
  std::vector<geometry_msgs::Pose> poses {pose_bottom_right, pose_bottom_left, pose_top_left, pose_top_right};
  
  for (int i = 0; i < poses.size(); i++)
  {
    pose_above = poses[i];
    pose_above.position.z = tote_floor_z + tote_h + .05;
    moveSuctionCupTo(pose_above, true, false);
    waitUntilArrived(1,1,1,1);
    moveSuctionCupTo(poses[i], true, false);
    waitUntilArrived(1,1,1,1);
    if (itemIsSuctioned)
    {
      pose_above.position = getPointHighAboveContainer("tote", .15);
      moveSuctionCupTo(pose_above);
      res.success = true;
      return true;
    }
    moveSuctionCupTo(pose_above);
  }

  pose_above.position = getPointHighAboveContainer("tote", .15);
  tf::quaternionTFToMsg(q_tote_neutral, pose_above.orientation);
  moveSuctionCupTo(pose_above);
  res.success = false;
  return false;
}


bool KukaMotionNode::nudgeItemsIntoContainerCallback(tnp_kuka_motion::nudgeItemsIntoContainer::Request &req, tnp_kuka_motion::nudgeItemsIntoContainer::Response &res)
{
  // Possibility 1: move from corners to middle for each side.

  // Possibility 2: Sweep outside -> inside like a brush, at different positions 
  // Try this first.
  // One motion consists of 2 points
  // Get two corners.

  tf::Quaternion q_tilt,
  q_tilt_left = tf::createQuaternionFromRPY(0.0, (30.0/180.0)*M_PI, 0.0), 
  q_tilt_right = tf::createQuaternionFromRPY(0.0, (-30.0/180.0)*M_PI, 0.0), 
  q_tilt_back = tf::createQuaternionFromRPY(0.0, (-30.0/180.0)*M_PI, 0.0), 
  q_tilt_forward = tf::createQuaternionFromRPY(0.0, (20.0/180.0)*M_PI, 0.0);

  // Get the two corner points
  double container_width_x, container_width_y, container_h;
  n_.getParam("tnp_environment/"+req.container_name+"_w", container_width_x);    // Width of the inside of the container
  n_.getParam("tnp_environment/"+req.container_name+"_l", container_width_y);    // Wall-to-wall measurement (inside)
  n_.getParam("tnp_environment/"+req.container_name+"_h", container_h);          // Container height

  geometry_msgs::Point start_point_container, start_point_world, end_point_container, end_point_world;
  if (req.side.compare("right") == 0)
  {
    start_point_container.x = container_width_x;
    start_point_container.y = 0;
    end_point_container.x = 0;
    end_point_container.y = 0;
    q_tilt = q_tilt_right;
  }
  else if (req.side.compare("top") == 0)
  {
    start_point_container.x = container_width_x;
    start_point_container.y = 0;
    end_point_container.x = container_width_x;
    end_point_container.y = container_width_y; 
    q_tilt = q_tilt_back;
  }
  else if (req.side.compare("bottom") == 0)
  {
    start_point_container.x = 0;
    start_point_container.y = 0;
    end_point_container.x = 0;
    end_point_container.y = container_width_y;
  }
  else if (req.side.compare("left") == 0)
  {
    start_point_container.x = container_width_x;
    start_point_container.y = 0;
    end_point_container.x = 0;
    end_point_container.y = 0;
    q_tilt = q_tilt_left;
  }

  // Get X points in between the corners that will be swept over. (probably best: at Y distance between the points unitl the corner is done)
  // For every point Px, create start and end pose for the motion
  // Go to each pair like this: start-end-start

  return true;

  // Possibility 3: Go to a pose until contact, then move towards the center? I don't know
}

bool KukaMotionNode::suckItemCallback(tnp_kuka_motion::suckItem::Request &req, tnp_kuka_motion::suckItem::Response &res)
{
  ROS_INFO_STREAM("suckItem was called");

  // This sets the correct tool orientation and moves the target item's position into the container if it's too close to the borders
  int target_container = guessTargetContainer(req.target_pose);
  std::string container_name = getContainerName(target_container);
  if (req.container_name.compare("") != 0) 
  {
    container_name = req.container_name;
  }
  
  const bool useSuction = true;
  geometry_msgs::Pose adjusted_target_pose = getSuctionCupPoseToSuckItemInContainer(req.target_pose.position, container_name);    // The new version that is good enough for stow only, apparently
  bool success;

  success = pickUpItem(adjusted_target_pose, false /*gripper_EE_is_used*/, req.force.data);  
  if (req.fuzzyMode) // FUZZY MODE! // FUZZY MODE!
  {
    ROS_INFO("Entering fuzzy mode for suction.");
    // Create adjacent points in a circle around the original target 
    // Currently: a hexagon
    double distance = .05;   // Distance between points and center
    std::vector<geometry_msgs::Pose> fuzzy_poses;
    geometry_msgs::Pose fuzzy_target_pose = adjusted_target_pose;
    for (int i = 0; i < 3; ++i)
    {
      fuzzy_target_pose.position.x = adjusted_target_pose.position.x + sin( (i*120/180.0)*M_PI ) * distance;
      fuzzy_target_pose.position.y = adjusted_target_pose.position.y + cos( (i*120/180.0)*M_PI ) * distance;
      fuzzy_target_pose.position = movePointIntoContainerBounds(fuzzy_target_pose.position, container_name, suction_safety_margin_to_borders_);
      fuzzy_poses.push_back(fuzzy_target_pose);
    }

    // Trim the poses to remove ones that are too close to each other
    for (int i = 0; i < fuzzy_poses.size()-1; ++i)
    {

      for (int j = i+1; j < fuzzy_poses.size(); ++j)
      {
        if (pointDistance(fuzzy_poses[i].position, fuzzy_poses[j].position) < .02)
        {
          // Remove the element j from vector
          ROS_INFO_STREAM("Removed fuzzy pose Nr. " << j << " from vector");
          fuzzy_poses.erase(fuzzy_poses.begin()+j);
          j--;
        }
      }      
    }

    // Go to each pose
    for (int i = 0; i < fuzzy_poses.size(); ++i)
    {
      if (itemIsSuctioned)
      {
        ROS_INFO_STREAM("Suction detected. Breaking out of fuzzy mode loop.");
        break;
      }
      fuzzy_target_pose = fuzzy_poses[i];
      ROS_INFO_STREAM("Fuzzy mode: Trying to suck at fuzzy pose nr. " << i << ": " << fuzzy_target_pose.position.x << ", " << fuzzy_target_pose.position.y << ", " << fuzzy_target_pose.position.z);
      success = pickUpItem(fuzzy_target_pose, false /*gripper_EE_is_used*/, req.force.data, 10, false /*set_up_EE*/);        
    }

  }
  res.success = itemIsSuctioned;
  return itemIsSuctioned;
}

bool KukaMotionNode::graspItemCallback(tnp_kuka_motion::graspItem::Request &req,
                                       tnp_kuka_motion::graspItem::Response &res)
{
  ROS_INFO("graspItem was called");

  // This sets the correct tool orientation and moves the target item's position into the container if it's too close to the borders
  int target_container = guessTargetContainer(req.target_pose);
  std::string container_name = getContainerName(target_container);
  
  geometry_msgs::Pose adjusted_target_pose;
  adjusted_target_pose.position = movePointIntoContainerBounds(req.target_pose.position, container_name, grasping_safety_margin_to_borders_); //*safety_margin*/);   

  // The above fixes the point to something sensible. Now to choose the rotation.
  // At start position, the gripper is aligned to grasp the y-axis (and on the right side of the flange)
  tf::Quaternion q_start = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);   

  // Rotate to the container we're in
  tf::Quaternion q_turn_to_box;
  tf::Vector3 yaxis;    // This is the y-axis of the container in the global coordinate system. Kind of ugly, but ohwell.
  if (container_name.compare("tote") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI);
    yaxis = tf::Vector3(1.0, 0.0, 0.0);
  }
  if (container_name.compare("bin_A") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    yaxis = tf::Vector3(0.0, 1.0, 0.0);
  }
  if (container_name.compare("bin_B") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);
    yaxis = tf::Vector3(-1.0, 0.0, 0.0);
  }
  if (container_name.compare("bin_C") == 0) { 
    q_turn_to_box = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);
    yaxis = tf::Vector3(-1.0, 0.0, 0.0);
  }
  
  // Calculate the angle between the object axis and the y-axis
  // It should probably be the y-axis of the container, so I defined it just above
  tf::Vector3 object_axis(req.pad_point_1.x - req.pad_point_2.x, req.pad_point_1.y - req.pad_point_2.y, req.pad_point_1.z - req.pad_point_2.z);
  double theta = acos( object_axis.dot(yaxis) / object_axis.length());  // [0, M_PI]
  // Theta is the angle between the y-axis and the object axis
  if (theta > M_PI/2) 
   {
    theta -= M_PI;  // To limit the range of theta to [-pi/2, pi/2]   (This is preferable to reduce unnecessary rotations from the camera pose)
  }
  double safe_theta = getSafeGraspRotation(adjusted_target_pose.position, theta, container_name);   // This flips the rotation if there is a collision

  // Create the pose quaternion from the grasp rotation
  tf::Quaternion q_turn_to_object = tf::createQuaternionFromRPY(0.0, 0.0, safe_theta);   
  tf::Quaternion q_container_neutral = q_turn_to_box * q_start;
  tf::Quaternion q_final = q_turn_to_object * q_turn_to_box * q_start;
  tf::quaternionTFToMsg(q_final, adjusted_target_pose.orientation);
  publishGraspPoseMarker(adjusted_target_pose.orientation, req.pad_point_1, req.pad_point_2);

  double width = 110; // Always max width, always max.

  ROS_WARN_STREAM("Going to grasp with safe_theta = " << safe_theta << ". Theta was: " << theta);    
  bool success = pickUpItem(adjusted_target_pose, true /*gripper_EE_is_used*/, req.force.data, width);
  if (req.fuzzyMode)
  {
    if (!itemIsGrasped)
    {
      // First, reset to the original pose, so A7 does not lock up.
      ROS_INFO_STREAM("Resetting by PTP to container neutral to reset A7 for the second grasp attempt (fuzzy mode).");
      geometry_msgs::PoseStamped ps;
      my_iiwa_ros_object.getCartesianPose(ps);
      tf::quaternionTFToMsg(q_final, ps.pose.orientation);
      my_iiwa_ros_object.setCartesianPose(ps);
      waitUntilArrived();

      // Rotate original theta by 90 degrees.
      double theta_2 = theta + M_PI/2.0; 
      if (theta_2 > M_PI/2.0)
      {
        theta_2 -= M_PI;  // To limit the range of theta_2 to [-pi/2, pi/2]   (This is preferable to reduce unnecessary rotations from the camera pose)
      }

      double safe_theta_2 = getSafeGraspRotation(adjusted_target_pose.position, theta_2, container_name);   // This flips the rotation if there is a collision
      tf::Quaternion q_turn_to_object = tf::createQuaternionFromRPY(0.0, 0.0, safe_theta_2);   
      tf::Quaternion q_final = q_turn_to_object * q_turn_to_box * q_start;
      tf::quaternionTFToMsg(q_final, adjusted_target_pose.orientation);
      ROS_INFO_STREAM("Fuzzy mode! Trying again with 90 degrees rotated (as far as safety allows). safe_theta_2 = " << safe_theta_2);

      publishGraspPoseMarker(adjusted_target_pose.orientation, req.pad_point_1, req.pad_point_2);
      bool success = pickUpItem(adjusted_target_pose, true /*gripper_EE_is_used*/, req.force.data, width);
    }
  }
  res.success = success;
  return success;
}

bool KukaMotionNode::wobbleForVisionCallback(tnp_kuka_motion::wobbleForVision::Request &req,
                                             tnp_kuka_motion::wobbleForVision::Response &res)
{
  ROS_INFO("wobbleForVision was called");
  // Increase speed
  my_iiwa_ros_object.getPathParametersService().setJointRelativeVelocity(1);
  my_iiwa_ros_object.getPathParametersService().setOverrideJointAcceleration(10);

  // Take current pose
  geometry_msgs::PoseStamped posenow;
  iiwa_msgs::JointPosition jpnow;
  my_iiwa_ros_object.getCartesianPose(posenow);
  my_iiwa_ros_object.getJointPosition(jpnow);

  // Create some poses to go to
  geometry_msgs::Pose pleft, pright, pup, pdown;
  pleft=posenow.pose; pright=posenow.pose; pup=posenow.pose; pdown=posenow.pose;
  float s = req.scalingFactor;
  if (s > 3.0)
  {
    ROS_INFO_STREAM("Scaling factor set too high (" << s << "). Reducing to 3.");
    s = 3.0;
  }
  rotatePoseByRPY(0, ((-10*s)/180.0)*M_PI, 0, pup);
  rotatePoseByRPY(0, ((+10*s)/180.0)*M_PI, 0, pdown);
  rotatePoseByRPY(((-10*s)/180.0)*M_PI, 0, 0, pleft);
  rotatePoseByRPY(((+10*s)/180.0)*M_PI, 0, 0, pright);

  pup.position.x = pup.position.x - (.05*s);
  pdown.position.x = pdown.position.x + (.05*s);
  pleft.position.y = pleft.position.y - (.05*s);
  pright.position.y = pright.position.y + (.05*s);


  // Go to all motions one after the other
  if (req.motionNumber == 0)
  {
    moveToCartPosePTP(pup);       ros::Duration(0.5).sleep();
    moveToJointAnglesPTP(jpnow);  ros::Duration(0.5).sleep();
    moveToCartPosePTP(pright);    ros::Duration(0.5).sleep();
    moveToJointAnglesPTP(jpnow);  ros::Duration(0.5).sleep();
    moveToCartPosePTP(pdown);     ros::Duration(0.5).sleep();
    moveToJointAnglesPTP(jpnow);  ros::Duration(0.5).sleep();
    moveToCartPosePTP(pleft);     ros::Duration(0.5).sleep();
    moveToJointAnglesPTP(jpnow);  ros::Duration(0.5).sleep();
  }
  // Go to
  if (req.motionNumber == 1) { moveToCartPosePTP(pup); }
  if (req.motionNumber == 2) { moveToCartPosePTP(pright); }
  if (req.motionNumber == 3) { moveToCartPosePTP(pdown); }
  if (req.motionNumber == 4) { moveToCartPosePTP(pleft); }

  // Decrease speed again
  my_iiwa_ros_object.getPathParametersService().setJointRelativeVelocity(1);
  my_iiwa_ros_object.getPathParametersService().setOverrideJointAcceleration(1);

  bool response;
  response = true;
  return response;
}

void KukaMotionNode::itemIsSuctionedCallback(const std_msgs::Int16::ConstPtr& input)
{
  /// @todo TODO itemIsSuctioned variable is prone to data race as other callbacks are trying to read it at the same time
  if (input->data == 0)
  {
    itemIsSuctioned = false;
    setLED("blue");
  }
  else if (input->data == 1)
  {
    itemIsSuctioned = true;
    setLED("off");
  }
  else
  {
    ROS_INFO("itemIsSuctionedCallback error: unexpected value (only 0 and 1 are valid)");
    itemIsSuctioned = false;
  }
  ROS_DEBUG("itemIsSuctioned is %s", itemIsSuctioned ? "true" : "false");
}

void KukaMotionNode::itemIsGraspedCallback(const std_msgs::Int16::ConstPtr& input)
{
  /// @todo TODO itemIsGrasped variable is prone to data race as other callbacks are trying to read it at the same time
  if (input->data == 0)
  {
    itemIsGrasped = false;
  }
  else if (input->data == 1)
  {
    itemIsGrasped = true;
  }
  else
  {
    ROS_INFO("itemIsGraspedCallback error: unexpected value (only 0 and 1 are valid)");
    itemIsGrasped = false;
  }
  ROS_DEBUG("itemIsGrasped is %s", itemIsGrasped ? "true" : "false");
}

void KukaMotionNode::cupIsTouchingSomethingCallback(const std_msgs::Int16::ConstPtr& input)
{
  /// @todo TODO cupIsTouchingSomething variable is prone to data race as other callbacks are trying to read it at the same time
  if (input->data == 0)
  {
    cupIsTouchingSomething = false;
  }
  else if (input->data == 1)
  {
    cupIsTouchingSomething = true;
  }
  else
  {
    ROS_INFO("cupIsTouchingSomethingCallback error: unexpected value (only 0 and 1 are valid)");
    cupIsTouchingSomething = false;
  }
  ROS_DEBUG("cupIsTouchingSomething is %s", cupIsTouchingSomething ? "true" : "false");
}

void KukaMotionNode::gripperIsTouchingSomethingCallback(const std_msgs::Int16::ConstPtr& input)
{
  /// @todo TODO gripperIsTouchingSomething variable is prone to data race as other callbacks are trying to read it at the same time
  if (input->data == 0)
  {
    gripperIsTouchingSomething = false;
  }
  else if (input->data == 1)
  {
    gripperIsTouchingSomething = true;
  }
  else
  {
    ROS_INFO("gripperIsTouchingSomethingCallback error: unexpected value (only 0 and 1 are valid)");
    gripperIsTouchingSomething = false;
  }
  ROS_DEBUG("gripperIsTouchingSomething is %s", gripperIsTouchingSomething ? "true" : "false");
}

// Does a PTP motion to the target container. The joint angles are fixed.
bool KukaMotionNode::goToContainer(std::string container_name)
{
  if (container_name.compare("tote") == 0)
  {
    moveToJointAnglesPTP(atTote);
    publishPoseMarker(atTote_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A") == 0)
  {
    moveToJointAnglesPTP(atBinA);
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A_1") == 0)
  {
    moveToJointAnglesPTP(atBinA1);
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A_2") == 0)
  {
    moveToJointAnglesPTP(atBinA2);
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_B") == 0)
  {
    moveToJointAnglesPTP(atBinB);
    publishPoseMarker(atBinB_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_C") == 0)
  {
    moveToJointAnglesPTP(atBinC);
    publishPoseMarker(atBinC_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("amnesty") == 0)
  {
    moveToJointAnglesPTP(atAmnesty);
    publishPoseMarker(atAmnesty_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_1") == 0)
  {
    moveToJointAnglesPTP(atBox1);
    publishPoseMarker(atBox1_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_2") == 0)
  {
    moveToJointAnglesPTP(atBox2);
    publishPoseMarker(atBox2_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_3") == 0)
  {
    moveToJointAnglesPTP(atBox3);
    publishPoseMarker(atBox3_pose, "target_robot_pose_flange");
  }
  else
  {
    ROS_ERROR_STREAM("Unknown container name! I received: " << container_name);
    ROS_ERROR_STREAM("Valid container names are: tote, bin_A, bin_A_1, bin_A_2, bin_B, bin_C, amnesty, box_1, box_2, box_3");
    return false;
  }

  return true;
}


bool KukaMotionNode::goToContainerWithFixedA7(std::string container_name)
{
  iiwa_msgs::JointPosition jp_now, jp_target;
  my_iiwa_ros_object.getJointPosition(jp_now);


  if (container_name.compare("tote") == 0)
  {
    jp_target = atTote;
    publishPoseMarker(atTote_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A") == 0)
  {
    jp_target = atBinA;
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A_1") == 0)
  {
    jp_target = atBinA1;
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_A_2") == 0)
  {
    jp_target = atBinA2;
    publishPoseMarker(atBinA_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_B") == 0)
  {
    jp_target = atBinB;
    publishPoseMarker(atBinB_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("bin_C") == 0)
  {
    jp_target = atBinC;
    publishPoseMarker(atBinC_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("amnesty") == 0)
  {
    jp_target = atAmnesty;
    publishPoseMarker(atAmnesty_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_1") == 0)
  {
    jp_target = atBox1;
    publishPoseMarker(atBox1_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_2") == 0)
  {
    jp_target = atBox2;
    publishPoseMarker(atBox2_pose, "target_robot_pose_flange");
  }
  else if (container_name.compare("box_3") == 0)
  {
    jp_target = atBox3;
    publishPoseMarker(atBox3_pose, "target_robot_pose_flange");
  }
  else
  {
    ROS_ERROR_STREAM("Unknown container name! I received: " << container_name);
    ROS_ERROR_STREAM("Valid container names are: tote, bin_A, bin_A_1, bin_A_2, bin_B, bin_C, amnesty, box_1, box_2, box_3");
    return false;
  }

  jp_target.position.a7 = jp_now.position.a7;
  moveToJointAnglesPTP(jp_target);
  return true;
}

// Does a PTP motion to the target container. The joint angles are fixed.
bool KukaMotionNode::goToLookIntoContainer(std::string container_name, int height)
{
  ROS_INFO("Ignoring height! Haha!");
  if (container_name.compare("tote") == 0)
  {
    moveToJointAnglesPTP(lookIntoTote);
  }
  else if (container_name.compare("bin_A") == 0)
  {
    moveToJointAnglesPTP(lookIntoBinA);
  }
  else if (container_name.compare("bin_A_1") == 0)
  {
    moveToJointAnglesPTP(lookIntoBinA1);
  }
  else if (container_name.compare("bin_A_2") == 0)
  {
    moveToJointAnglesPTP(lookIntoBinA2);
  }
  else if (container_name.compare("bin_B") == 0)
  {
    moveToJointAnglesPTP(lookIntoBinB);
  }
  else if (container_name.compare("bin_C") == 0)
  {
    moveToJointAnglesPTP(lookIntoBinC);
  }
  else if (container_name.compare("amnesty") == 0)
  {
    moveToJointAnglesPTP(lookIntoAmnesty);
  }
  else if (container_name.compare("box_1") == 0)
  {
    moveToJointAnglesPTP(lookIntoBox1);
  }
  else if (container_name.compare("box_2") == 0)
  {
    moveToJointAnglesPTP(lookIntoBox2);
  }
  else if (container_name.compare("box_3") == 0)
  {
    moveToJointAnglesPTP(lookIntoBox3);
  }
  else
  {
    ROS_ERROR_STREAM("Unknown container name! I received: " << container_name);
    ROS_ERROR_STREAM("Valid container names are: tote, bin_A, bin_B, bin_C, amnesty, box_1, box_2, box_3");
    return false;
  }

  return true;
}

bool KukaMotionNode::canRobotPlaceItemHere(geometry_msgs::Pose EE_target_pose, std::string container_name, bool use_gripper_EE, int calculation_mode)
{
  // Calculation mode 0: Use MoveIt.
  // Calculation mode 1: Use my manual calculation

  // We assume that the critical point is when the EE is above the container.
  // It is hard to reach because of the height.
  // Get the critical point. 
  double container_h, container_floor_z;
  n_.getParam("tnp_environment/"+container_name+"_h", container_h);
  n_.getParam("tnp_environment/"+container_name+"_z", container_floor_z);

  if ((EE_target_pose.orientation.x < 1e-6) && 
    (EE_target_pose.orientation.y < 1e-6) && 
    (EE_target_pose.orientation.z < 1e-6) && 
    (EE_target_pose.orientation.w < 1e-6))
  {
    ROS_WARN("No feasible orientation given to canRobotPlaceItemHere. Assuming the current one.");
    EE_target_pose.orientation = getRotatedNeutralOrientationForContainer(container_name, 0);
  }

  geometry_msgs::Pose EE_pose_at_hcrit = EE_target_pose;
  EE_pose_at_hcrit.position.z = container_floor_z + container_h + .10;

  if (calculation_mode == 0)
  {
    return isEEPoseReachable(EE_pose_at_hcrit, use_gripper_EE);
  }
  else if (calculation_mode == 1)
  {
    // The reachable area from the neutral position for suction is a sphere around the A2 joint, shifted by the EE_tool's offset

    // Get the offset from EE tip to flange at the critical point
    geometry_msgs::Pose flange_pose_at_hcrit;
    if (use_gripper_EE)
    {
      transformPositionFromGripperTipToFlange(EE_pose_at_hcrit, flange_pose_at_hcrit);
    }
    else
    {
      transformPositionFromSuctionCupToFlange(EE_pose_at_hcrit, flange_pose_at_hcrit);
    }

    ROS_INFO_STREAM("flange_pose_at_hcrit: " << flange_pose_at_hcrit.position);

    // Use the offset from flange to A6/A7 intersection to get the reachable area
    geometry_msgs::Point p_axis_67 = flange_pose_at_hcrit.position;
    p_axis_67.z += .1528;   // Taken from the iiwa14mft.xacro definition file. Official doc for iiwa14 says .126, so .1528 for mft souds right

    geometry_msgs::Point p_axis_2;  // A1/A2 intersection
    p_axis_2.z = 0.36;    // From the official doc

    ROS_INFO_STREAM("p_axis_67: " << p_axis_67);
    ROS_INFO_STREAM("p_axis_2: " << p_axis_2);
    // Check if p_c is within reach for the robot (absolute max: 820 mm)
    if (pointDistance(p_axis_67, p_axis_2) > .76) 
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

bool KukaMotionNode::putItemIntoContainer(std::string container_name, bool gripper_EE_is_used, geometry_msgs::Point target_point_in_world, double rotation_angle_z)
{
  ROS_DEBUG_STREAM("[TNP_STATE L3] Executing tnp_kuka_motion/putItemIntoContainer Putting item into container " << container_name);
  ROS_INFO_STREAM("putItemIntoContainer was called. Going to container " << container_name);
  // Record start pose.
  geometry_msgs::PoseStamped ps_start;
  my_iiwa_ros_object.getCartesianPose(ps_start);

  if ( (container_name.compare("bin_A_1") == 0) || (container_name.compare("bin_A_2") == 0))
  {
    container_name = "bin_A";
  }

  int target_container = getContainerID(container_name);
  // This sets the orientation and moves the target pose into the container if it's too close to the borders
  bool useSuction = !gripper_EE_is_used;
  geometry_msgs::Pose target_pose_in_world = makePose(target_point_in_world);   // THIS IS ONLY BECAUSE checkIfPoseInContainer IS DEPRECATED
  geometry_msgs::Pose target_pose_adjusted = checkIfPoseInContainer(target_pose_in_world, target_container, useSuction);

  target_pose_adjusted.orientation = getRotatedNeutralOrientationForContainer(container_name, rotation_angle_z);

  // Publish the poses in RViz
  publishPoseMarker(target_pose_adjusted, "target_item_pose");
  if ((target_pose_in_world.position.x != target_pose_adjusted.position.x) || 
      (target_pose_in_world.position.y != target_pose_adjusted.position.y) || 
      (target_pose_in_world.position.z != target_pose_adjusted.position.z))
  {
    ROS_INFO_STREAM("putItemIntoContainer received target_pose_in_world: " << target_pose_in_world.position.x << ", " << target_pose_in_world.position.y << ", " << target_pose_in_world.position.z);
    ROS_INFO_STREAM("But had to change it to target_pose_adjusted: " << target_pose_adjusted.position.x << ", " << target_pose_adjusted.position.y << ", " << target_pose_adjusted.position.z);
    publishPoseMarker(target_pose_adjusted, "refused_item_pose");
  }
  setLinSpeed();      // Go at full speed

  geometry_msgs::Pose pose_above_target = target_pose_adjusted;
  ROS_WARN("Using fixed height 10 cm above item.");
  pose_above_target.position.z += .1;

  try
  {
    // Move to above the target pose
    ROS_INFO_STREAM("Moving to above target item pose in putItemIntoContainer.");
    moveEETo(pose_above_target, gripper_EE_is_used);

    // Lower item until target pose
    ROS_INFO_STREAM("Moving down to target item pose in putItemIntoContainer.");
    moveEETo(target_pose_adjusted, gripper_EE_is_used);
    
    // Drop item
    ROS_INFO_STREAM("Dropping item.");
    setEEStatus(0, 0, 0, 0, 100, 40);

    // Move back up
    ROS_INFO_STREAM("Moving back to above target item pose of putItemIntoContainer.");
    moveEETo(pose_above_target, gripper_EE_is_used);
 
    setEEStatus(1, 0, 0, 0, 100, 40);
  }
  catch(std::runtime_error exc)
  {
    ROS_ERROR("Something went wrong in putItemIntoContainer.");
    return false;
  }
  return true;
}

// Actions
void KukaMotionNode::execute(const tnp_kuka_motion::moveToJointAnglesPTPGoalConstPtr& goal)
{
  ROS_INFO("moveToJointAnglesPTPAction was called");
  moveToJointAnglesPTP(goal->a1, goal->a2, goal->a3, goal->a4, goal->a5, goal->a6, goal->a7);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("moveToJointAnglesPTPAction is set as succeeded");
  moveToJointAnglesPTPActionServer_.setSucceeded();
}

// This is only called once to define the big important positions
bool KukaMotionNode::initializePositions()
{
  atHome.position.a1 = 0 / 180.0 * M_PI;
  atHome.position.a2 = 32 / 180.0 * M_PI;
  atHome.position.a3 = 0 / 180.0 * M_PI;
  atHome.position.a4 = -53 / 180.0 * M_PI;
  atHome.position.a5 = 0 / 180.0 * M_PI;
  atHome.position.a6 = 95 / 180.0 * M_PI;
  atHome.position.a7 = 0 / 180.0 * M_PI;

  atTote.position.a1 = -90 / 180.0 * M_PI;
  atTote.position.a2 = +17 / 180.0 * M_PI;
  atTote.position.a3 = 0 / 180.0 * M_PI;
  atTote.position.a4 = -69 / 180.0 * M_PI;
  atTote.position.a5 = 0 / 180.0 * M_PI;
  atTote.position.a6 = +94 / 180.0 * M_PI;
  atTote.position.a7 = 0 / 180.0 * M_PI;

  lookIntoTote.position.a1 = -92.5 / 180.0 * M_PI;
  lookIntoTote.position.a2 = +24 / 180.0 * M_PI;
  lookIntoTote.position.a3 = 0 / 180.0 * M_PI;
  lookIntoTote.position.a4 = -80 / 180.0 * M_PI;
  lookIntoTote.position.a5 = 0 / 180.0 * M_PI;
  lookIntoTote.position.a6 = +82.5 / 180.0 * M_PI;
  lookIntoTote.position.a7 = -1.7 / 180.0 * M_PI;

  atAmnesty.position.a1 = -145 / 180.0 * M_PI;
  atAmnesty.position.a2 = 40 / 180.0 * M_PI;
  atAmnesty.position.a3 = 0 / 180.0 * M_PI;
  atAmnesty.position.a4 = -42 / 180.0 * M_PI;
  atAmnesty.position.a5 = 0 / 180.0 * M_PI;
  atAmnesty.position.a6 = 97 / 180.0 * M_PI;
  atAmnesty.position.a7 = 35 / 180.0 * M_PI;

  lookIntoAmnesty.position.a1 = -146 / 180.0 * M_PI;
  lookIntoAmnesty.position.a2 = 39 / 180.0 * M_PI;
  lookIntoAmnesty.position.a3 = 0 / 180.0 * M_PI;
  lookIntoAmnesty.position.a4 = -75 / 180.0 * M_PI;
  lookIntoAmnesty.position.a5 = 7 / 180.0 * M_PI;
  lookIntoAmnesty.position.a6 = 73.5 / 180.0 * M_PI;
  lookIntoAmnesty.position.a7 = 30.5 / 180.0 * M_PI;

  atRecSpace.position.a1 = 0 / 180.0 * M_PI;
  atRecSpace.position.a2 = 34.26 / 180.0 * M_PI;
  atRecSpace.position.a3 = 0 / 180.0 * M_PI;
  atRecSpace.position.a4 = -43.66 / 180.0 * M_PI;
  atRecSpace.position.a5 = 0 / 180.0 * M_PI;
  atRecSpace.position.a6 = 101.1 / 180.0 * M_PI;
  atRecSpace.position.a7 = 0 / 180.0 * M_PI;

  atRecSpace_pose.position.x = 0.63019;
  atRecSpace_pose.position.y = 0.0;
  atRecSpace_pose.position.z = 0.68388;
  atRecSpace_pose.orientation.x = 0.0;
  atRecSpace_pose.orientation.y = 01.0;
  atRecSpace_pose.orientation.z = 0.0;
  atRecSpace_pose.orientation.w = 0.0;
  // To get this pose, either go to the above and do: rosrun tf tf_echo tnp_suction_link_ee iiwa_link_0
  // Or just add the offset from the suction cup end effector manually
  atRecSpace_pose_EE.position.x = 0.641;
  atRecSpace_pose_EE.position.y = 0.072;
  atRecSpace_pose_EE.position.z = 0.055;
  atRecSpace_pose_EE.orientation.x = 0.0;
  atRecSpace_pose_EE.orientation.y = 1.0;
  atRecSpace_pose_EE.orientation.z = 0.0;
  atRecSpace_pose_EE.orientation.w = 0.0;

  atBinA.position.a1 = 0 / 180.0 * M_PI;
  atBinA.position.a2 = 32 / 180.0 * M_PI;
  atBinA.position.a3 = 0 / 180.0 * M_PI;
  atBinA.position.a4 = -53 / 180.0 * M_PI;
  atBinA.position.a5 = 0 / 180.0 * M_PI;
  atBinA.position.a6 = 95 / 180.0 * M_PI;
  atBinA.position.a7 = 0 / 180.0 * M_PI;

  atBinA1.position.a1 = -21 / 180.0 * M_PI;
  atBinA1.position.a2 = 41 / 180.0 * M_PI;
  atBinA1.position.a3 = 0 / 180.0 * M_PI;
  atBinA1.position.a4 = -40 / 180.0 * M_PI;
  atBinA1.position.a5 = 0 / 180.0 * M_PI;
  atBinA1.position.a6 = 100 / 180.0 * M_PI;
  atBinA1.position.a7 = -21 / 180.0 * M_PI;

  atBinA2.position.a1 = 21 / 180.0 * M_PI;
  atBinA2.position.a2 = 41 / 180.0 * M_PI;
  atBinA2.position.a3 = 0 / 180.0 * M_PI;
  atBinA2.position.a4 = -40 / 180.0 * M_PI;
  atBinA2.position.a5 = 0 / 180.0 * M_PI;
  atBinA2.position.a6 = 100 / 180.0 * M_PI;
  atBinA2.position.a7 = 21 / 180.0 * M_PI;

  atBinB.position.a1 = +70 / 180.0 * M_PI;
  atBinB.position.a2 = 32 / 180.0 * M_PI;
  atBinB.position.a3 = 0 / 180.0 * M_PI;
  atBinB.position.a4 = -53 / 180.0 * M_PI;
  atBinB.position.a5 = 0 / 180.0 * M_PI;
  atBinB.position.a6 = 95 / 180.0 * M_PI;
  atBinB.position.a7 = -20 / 180.0 * M_PI;

  atBinC.position.a1 = 107 / 180.0 * M_PI;
  atBinC.position.a2 = 32 / 180.0 * M_PI;
  atBinC.position.a3 = 0 / 180.0 * M_PI;
  atBinC.position.a4 = -53 / 180.0 * M_PI;
  atBinC.position.a5 = 0 / 180.0 * M_PI;
  atBinC.position.a6 = 95 / 180.0 * M_PI;
  atBinC.position.a7 = 17 / 180.0 * M_PI;

  // Make the camera point at bin 1
  lookIntoBinA.position.a1 = 0 / 180.0 * M_PI;
  lookIntoBinA.position.a2 = 14 / 180.0 * M_PI;
  lookIntoBinA.position.a3 = 0 / 180.0 * M_PI;
  lookIntoBinA.position.a4 = -83 / 180.0 * M_PI;
  lookIntoBinA.position.a5 = 0 / 180.0 * M_PI;
  lookIntoBinA.position.a6 = 86 / 180.0 * M_PI;
  lookIntoBinA.position.a7 = -1 / 180.0 * M_PI;

  lookIntoBinA1.position.a1 = -17.71 / 180.0 * M_PI;
  lookIntoBinA1.position.a2 = 20.45 / 180.0 * M_PI;
  lookIntoBinA1.position.a3 = 0 / 180.0 * M_PI;
  lookIntoBinA1.position.a4 = -89.21 / 180.0 * M_PI;
  lookIntoBinA1.position.a5 = -0.95 / 180.0 * M_PI;
  lookIntoBinA1.position.a6 = 73.22 / 180.0 * M_PI;
  lookIntoBinA1.position.a7 = -18.42 / 180.0 * M_PI;

  lookIntoBinA2.position.a1 = 18.94 / 180.0 * M_PI;
  lookIntoBinA2.position.a2 = 20.93 / 180.0 * M_PI;
  lookIntoBinA2.position.a3 = 0 / 180.0 * M_PI;
  lookIntoBinA2.position.a4 = -88.54 / 180.0 * M_PI;
  lookIntoBinA2.position.a5 = 1.02 / 180.0 * M_PI;
  lookIntoBinA2.position.a6 = 73.38 / 180.0 * M_PI;
  lookIntoBinA2.position.a7 = 17.62 / 180.0 * M_PI;

  lookIntoBinB.position.a1 = 1.23117351532;
  lookIntoBinB.position.a2 = 0.58312100172;
  lookIntoBinB.position.a3 = 8.82039239514e-05;
  lookIntoBinB.position.a4 = -1.37524211407;
  lookIntoBinB.position.a5 = -0.0340658798814;
  lookIntoBinB.position.a6 = 1.35811281204;
  lookIntoBinB.position.a7 = -0.329303205013;

  lookIntoBinC.position.a1 = 1.97826635838;
  lookIntoBinC.position.a2 = 0.830744802952;
  lookIntoBinC.position.a3 = 7.49613463995e-05;
  lookIntoBinC.position.a4 = -1.27652215958;
  lookIntoBinC.position.a5 = 0.0834270790219;
  lookIntoBinC.position.a6 = 1.22703588009;
  lookIntoBinC.position.a7 = 0.379984885454;

  // high position (CartesianPose:z=0.650)
  lookIntoBinAhigh.position.a1 = 0.0313787;
  lookIntoBinAhigh.position.a2 = 0.350405;
  lookIntoBinAhigh.position.a3 = 0;
  lookIntoBinAhigh.position.a4 = -1.10557;
  lookIntoBinAhigh.position.a5 = 0.00498783;
  lookIntoBinAhigh.position.a6 = 1.83436;
  lookIntoBinAhigh.position.a7 = 0.0106708;
  // middle position (CartesianPose:z=0.600)
  lookIntoBinAmid.position.a1 = 0.031378;
  lookIntoBinAmid.position.a2 = 0.343795;
  lookIntoBinAmid.position.a3 = 0;
  lookIntoBinAmid.position.a4 = -1.2399;
  lookIntoBinAmid.position.a5 = 0.00499013;
  lookIntoBinAmid.position.a6 = 1.70667;
  lookIntoBinAmid.position.a7 = 0.0101210;
  // low position (CartesianPose:z=0.600)
  lookIntoBinAlow.position.a1 = 0.03137883;
  lookIntoBinAlow.position.a2 = 0.3530243;
  lookIntoBinAlow.position.a3 = 0;
  lookIntoBinAlow.position.a4 = -1.352918;
  lookIntoBinAlow.position.a5 = 0.004992915;
  lookIntoBinAlow.position.a6 = 1.584530;
  lookIntoBinAlow.position.a7 = 0.009530275;

  // Box camera positions
  lookIntoBox1.position.a1 = -81.38 / 180.0 * M_PI;
  lookIntoBox1.position.a2 = 20.77 / 180.0 * M_PI;
  lookIntoBox1.position.a3 = 0. / 180.0 * M_PI;
  lookIntoBox1.position.a4 = -85.75 / 180.0 * M_PI;
  lookIntoBox1.position.a5 = 0. / 180.0 * M_PI;
  lookIntoBox1.position.a6 = 81.44 / 180.0 * M_PI;
  lookIntoBox1.position.a7 = 5.48 / 180.0 * M_PI;

  lookIntoBox2.position.a1 = -119 / 180.0 * M_PI;
  lookIntoBox2.position.a2 = 27 / 180.0 * M_PI;
  lookIntoBox2.position.a3 = 0 / 180.0 * M_PI;
  lookIntoBox2.position.a4 = -76 / 180.0 * M_PI;
  lookIntoBox2.position.a5 = 1 / 180.0 * M_PI;
  lookIntoBox2.position.a6 = 84 / 180.0 * M_PI;
  lookIntoBox2.position.a7 = 7 / 180.0 * M_PI;

  lookIntoBox3.position.a1 = -150 / 180.0 * M_PI;
  lookIntoBox3.position.a2 = 27 / 180.0 * M_PI;
  lookIntoBox3.position.a3 = 0 / 180.0 * M_PI;
  lookIntoBox3.position.a4 = -86 / 180.0 * M_PI;
  lookIntoBox3.position.a5 = 1 / 180.0 * M_PI;
  lookIntoBox3.position.a6 = 65 / 180.0 * M_PI;
  lookIntoBox3.position.a7 = 8 / 180.0 * M_PI;

  // high position (CartesianPose:z=0.500)
  lookIntoToteLow.position.a1 = -1.614733;
  lookIntoToteLow.position.a2 = 0.4172319;
  lookIntoToteLow.position.a3 = 0;
  lookIntoToteLow.position.a4 = -1.391142;
  lookIntoToteLow.position.a5 = -0.0001376747;
  lookIntoToteLow.position.a6 = 1.495096;
  lookIntoToteLow.position.a7 = -0.07112073;

  // middle position (CartesianPose:z=0.450)
  lookIntoToteMid.position.a1 = -1.614732;
  lookIntoToteMid.position.a2 = 0.4521616;
  lookIntoToteMid.position.a3 = 0;
  lookIntoToteMid.position.a4 = -1.470598;
  lookIntoToteMid.position.a5 = -0.0001331686;
  lookIntoToteMid.position.a6 = 1.380891;
  lookIntoToteMid.position.a7 = -0.07104368;

  // low position (CartesianPose:z=0.400)
  lookIntoToteHigh.position.a1 = -1.6147317;
  lookIntoToteHigh.position.a2 = 0.49943855;
  lookIntoToteHigh.position.a3 = 0;
  lookIntoToteHigh.position.a4 = -1.5346783;
  lookIntoToteHigh.position.a5 = -0.00013364806;
  lookIntoToteHigh.position.a6 = 1.2695664;
  lookIntoToteHigh.position.a7 = -0.071034;

  // Box positions on June 8
  atBox1.position.a1 = -91 / 180.0 * M_PI;
  atBox1.position.a2 = +17 / 180.0 * M_PI;
  atBox1.position.a3 = 0 / 180.0 * M_PI;
  atBox1.position.a4 = -73 / 180.0 * M_PI;
  atBox1.position.a5 = 0 / 180.0 * M_PI;
  atBox1.position.a6 = +91 / 180.0 * M_PI;
  atBox1.position.a7 = 1 / 180.0 * M_PI;

  atBox2.position.a1 = -129 / 180.0 * M_PI;
  atBox2.position.a2 = +41 / 180.0 * M_PI;
  atBox2.position.a3 = 0 / 180.0 * M_PI;
  atBox2.position.a4 = -36 / 180.0 * M_PI;
  atBox2.position.a5 = 0 / 180.0 * M_PI;
  atBox2.position.a6 = +103 / 180.0 * M_PI;
  atBox2.position.a7 = -3 / 180.0 * M_PI;

  // This one is almost outside A1 limits.
  atBox3.position.a1 = -162 / 180.0 * M_PI;
  atBox3.position.a2 = 45 / 180.0 * M_PI;
  atBox3.position.a3 = 0 / 180.0 * M_PI;
  atBox3.position.a4 = -49.50 / 180.0 * M_PI;
  atBox3.position.a5 = 0 / 180.0 * M_PI;
  atBox3.position.a6 = 85.60 / 180.0 * M_PI;
  atBox3.position.a7 = 3 / 180.0 * M_PI;

  lookAtRecSpaceCube.position.a1 = -92.85 / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a2 = 37.15  / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a3 = 35.44  / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a4 = -74.56 / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a5 = 27.07  / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a6 = 105.28 / 180.0 * M_PI;
  lookAtRecSpaceCube.position.a7 = 15.22  / 180.0 * M_PI;


  // ==================================================== As recorded with calibrationMotion 17 and manually transformed

  atHome_pose.position.x = 0.621022;
  atHome_pose.position.y = 5.29383e-07;
  atHome_pose.position.z = 0.599078;
  atHome_pose.orientation.x = -1.69424e-05;
  atHome_pose.orientation.y = 1;
  atHome_pose.orientation.z = -5.47544e-05;
  atHome_pose.orientation.w = 4.06862e-05;
  atTote_pose.position.x = -6.6591e-05;
  atTote_pose.position.y = -0.521814;
  atTote_pose.position.z = 0.637518;
  atTote_pose.orientation.x = 0.707149;
  atTote_pose.orientation.y = 0.707064;
  atTote_pose.orientation.z = -4.08484e-06;
  atTote_pose.orientation.w = -5.15427e-05;
  lookIntoTote_pose.position.x = -0.023293;
  lookIntoTote_pose.position.y = -0.532039;
  lookIntoTote_pose.position.z = 0.497149;
  lookIntoTote_pose.orientation.x = 0.704413;
  lookIntoTote_pose.orientation.y = 0.70441;
  lookIntoTote_pose.orientation.z = 0.0642902;
  lookIntoTote_pose.orientation.w = -0.0589532;
  atAmnesty_pose.position.x = -0.547828;
  atAmnesty_pose.position.y = -0.383535;
  atAmnesty_pose.position.z = 0.585351;
  atAmnesty_pose.orientation.x = 0.999963;
  atAmnesty_pose.orientation.y = -6.10802e-05;
  atAmnesty_pose.orientation.z = -0.0070907;
  atAmnesty_pose.orientation.w = 0.00493045;
  atRecSpace_pose.position.x = 0.630136;
  atRecSpace_pose.position.y = -3.72583e-05;
  atRecSpace_pose.position.z = 0.687716;
  atRecSpace_pose.orientation.x = 4.00832e-05;
  atRecSpace_pose.orientation.y = 0.999961;
  atRecSpace_pose.orientation.z = -4.03529e-05;
  atRecSpace_pose.orientation.w = 0.00878375;
  atBinA_pose.position.x = 0.621028;
  atBinA_pose.position.y = -3.52095e-05;
  atBinA_pose.position.z = 0.599058;
  atBinA_pose.orientation.x = 2.27665e-05;
  atBinA_pose.orientation.y = 1;
  atBinA_pose.orientation.z = -3.7746e-05;
  atBinA_pose.orientation.w = 3.99665e-05;
  atBinB_pose.position.x = 0.212411;
  atBinB_pose.position.y = 0.583567;
  atBinB_pose.position.z = 0.599048;
  atBinB_pose.orientation.x = -0.707089;
  atBinB_pose.orientation.y = 0.707125;
  atBinB_pose.orientation.z = -3.41005e-05;
  atBinB_pose.orientation.w = 2.76708e-05;
  atBinC_pose.position.x = -0.181583;
  atBinC_pose.position.y = 0.593887;
  atBinC_pose.position.z = 0.59905;
  atBinC_pose.orientation.x = 0.707115;
  atBinC_pose.orientation.y = -0.707099;
  atBinC_pose.orientation.z = -2.88586e-05;
  atBinC_pose.orientation.w = -1.94879e-06;
  atBox1_pose.position.x = -0.00905108;
  atBox1_pose.position.y = -0.52004;
  atBox1_pose.position.z = 0.609685;
  atBox1_pose.orientation.x = 0.719269;
  atBox1_pose.orientation.y = 0.694678;
  atBox1_pose.orientation.z = 0.00616298;
  atBox1_pose.orientation.w = -0.00610469;
  atBox2_pose.position.x = -0.418719;
  atBox2_pose.position.y = -0.517006;
  atBox2_pose.position.z = 0.614953;
  atBox2_pose.orientation.x = 0.891022;
  atBox2_pose.orientation.y = 0.45396;
  atBox2_pose.orientation.z = 4.07123e-05;
  atBox2_pose.orientation.w = 2.64024e-05;
  atBox3_pose.position.x = -0.661441;
  atBox3_pose.position.y = -0.214882;
  atBox3_pose.position.z = 0.53974;
  atBox3_pose.orientation.x = 0.991453;
  atBox3_pose.orientation.y = 0.130467;
  atBox3_pose.orientation.z = 0.000102344;
  atBox3_pose.orientation.w = 3.1032e-05;
  lookIntoBinA_pose.position.x = 0.477433;
  lookIntoBinA_pose.position.y = -7.68974e-06;
  lookIntoBinA_pose.position.z = 0.568276;
  lookIntoBinA_pose.orientation.x = -0.00867978;
  lookIntoBinA_pose.orientation.y = 0.997525;
  lookIntoBinA_pose.orientation.z = 0.000648215;
  lookIntoBinA_pose.orientation.w = -0.0697775;
  lookIntoBinB_pose.position.x = 0.180455;
  lookIntoBinB_pose.position.y = 0.509084;
  lookIntoBinB_pose.position.z = 0.334827;
  lookIntoBinB_pose.orientation.x = -0.704192;
  lookIntoBinB_pose.orientation.y = 0.704466;
  lookIntoBinB_pose.orientation.z = -0.0526033;
  lookIntoBinB_pose.orientation.w = -0.0712301;
  lookIntoBinC_pose.position.x = -0.273894;
  lookIntoBinC_pose.position.y = 0.533851;
  lookIntoBinC_pose.position.z = 0.284488;
  lookIntoBinC_pose.orientation.x = 0.713046;
  lookIntoBinC_pose.orientation.y = -0.693482;
  lookIntoBinC_pose.orientation.z = 0.0757073;
  lookIntoBinC_pose.orientation.w = 0.070118;
  lookIntoBinAmid_pose = lookIntoBinA_pose;
  lookIntoBinBmid_pose = lookIntoBinB_pose;
  lookIntoBinCmid_pose = lookIntoBinC_pose;
  lookIntoToteMid_pose = lookIntoTote_pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_kuka_motion");

  //Create an object of class KukaMotionNode that will take care of everything
  KukaMotionNode SAPObject;

  ROS_INFO("KUKA motion node started");

  ros::spin();
  return 0;
}