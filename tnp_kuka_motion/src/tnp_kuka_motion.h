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

#ifndef TNP_KUKA_MOTION_H
#define TNP_KUKA_MOTION_H

// Services
#include "tnp_kuka_motion/calibrationMotion.h"
#include "tnp_kuka_motion/goToContainer.h"
#include "tnp_kuka_motion/goToHome.h"
#include "tnp_kuka_motion/goToTote.h"
#include "tnp_kuka_motion/goToAmnesty.h"
#include "tnp_kuka_motion/goToRecSpace.h"
#include "tnp_kuka_motion/goToBin.h"
#include "tnp_kuka_motion/goToBox.h"
#include "tnp_kuka_motion/goToLookIntoContainer.h"
#include "tnp_kuka_motion/goToLookIntoAmnesty.h"
#include "tnp_kuka_motion/goToLookIntoTote.h"
#include "tnp_kuka_motion/goToLookIntoBin.h"
#include "tnp_kuka_motion/goToLookIntoBox.h"
#include "tnp_kuka_motion/canRobotPlaceItemHere.h"
#include "tnp_kuka_motion/putItemIntoContainer.h"
#include "tnp_kuka_motion/putItemIntoTote.h"
#include "tnp_kuka_motion/putItemIntoAmnesty.h"
#include "tnp_kuka_motion/putItemIntoBin.h"
#include "tnp_kuka_motion/putItemIntoBox.h"
#include "tnp_kuka_motion/moveToCartPosePTP.h"
#include "tnp_kuka_motion/moveToJointAnglesPTP.h"
#include "tnp_kuka_motion/moveSuctionCupTo.h"
#include "tnp_kuka_motion/moveGripperTo.h"
#include "tnp_kuka_motion/sweepToteHorizontal.h"
#include "tnp_kuka_motion/sweepToteCorners.h"
#include "tnp_kuka_motion/nudgeItemsIntoContainer.h"
#include "tnp_kuka_motion/suckItem.h"
#include "tnp_kuka_motion/graspItem.h"
#include "tnp_kuka_motion/wobbleForVision.h"

// End effector
#include "tnp_end_effector/EEControl.h"
#include "tnp_end_effector/Gripper.h"
#include "tnp_end_effector/Suction.h"
#include "tnp_end_effector/SuctionRotation.h"
#include "tnp_end_effector/LinActuatorGripper.h"
#include "tnp_end_effector/LinActuatorSuction.h"
// MoveIt planner
#include "tnp_moveit_planner/moveGripperTo.h"
#include "tnp_moveit_planner/moveSuctionCupTo.h"
#include "tnp_moveit_planner/getFlangePoseAtEEPose.h"
#include "tnp_moveit_planner/isEEPoseReachable.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "tnp_kuka_motion/moveToJointAnglesPTPAction.h"

// Services we use
#include <tnp_msgs/ShowPose.h>
#include <tnp_msgs/ShowGraspPose.h>
#include <iiwa_msgs/SetPathParametersLin.h>
#include <iiwa_msgs/LED.h>

#include "iiwa_ros/iiwa_ros.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"

#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>    // Includes the TF conversions
// System libraries
#include <chrono>
#include <thread>     // To wait for time_to_destination
#include <math.h>
#include <string>
#include <sstream>    // Just for some error message
#include <vector>

// Only for the calibration motion where cartesian poses are checked for all PTP poses
#include <iostream>
#include <fstream>


class KukaMotionNode
{
public:
  //Constructor
  KukaMotionNode();

  bool initializePositions(); // Populates joint angles/positions we use. Called during startup.

  //Helpers
  bool waitUntilArrived(bool checkForOverload = true, bool checkForSuction = false, bool checkForSuctionContact = false, bool checkForGripperContact = false); // Returns true if KUKA has arrived at destination
  bool waitUntilArrived2(bool checkForOverload = true, bool checkForSuction = false, bool checkForSuctionContact = false, bool checkForGripperContact = false);
  bool setLinSpeed(double x_vel_lim = .1, double y_vel_lim = .1, double z_vel_lim = .1, double x_ang_lim = 10.0 , double y_ang_lim = 10.0 , double z_ang_lim = 10.0 );
  bool sendLinMotionUntilNearTarget(geometry_msgs::PoseStamped& target_pose);
  bool sendLinMotionUntilNearTarget(geometry_msgs::Pose& target_pose);
  bool moveToJointAnglesPTP(iiwa_msgs::JointPosition jp);
  bool moveToJointAnglesPTP(const double& j1, const double& j2, const double& j3, const double& j4, const double& j5,
                            const double& j6, const double& j7);
  bool moveToCartPosePTP(const double& x, const double& y, const double& z, const double& u, const double& v,
                         const double& w); // uvw = roll,pitch,yaw
  bool moveToCartPosePTP(geometry_msgs::Pose pose, bool wait = true);
  bool stop(std::string mode="lin"); // Stops the robot at the current position

  // Transformation-related helpers
  bool isEEPoseReachable(geometry_msgs::Pose EE_pose, bool use_gripper_EE);
  geometry_msgs::Pose getFlangePoseAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE, bool& pose_is_reachable);
  bool getFlangePoseAndA7AngleAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE, double& A7_angle);
  double getA7AngleAtEEPose(geometry_msgs::Pose EE_pose, bool use_gripper_EE);
  void transformPositionFromSuctionCupToFlange(geometry_msgs::Pose& inpose, geometry_msgs::Pose& outpose);
  void transformPositionFromGripperTipToFlange(geometry_msgs::Pose& inpose, geometry_msgs::Pose& outpose);
  geometry_msgs::Pose getCurrentSuctionCupPose();
  bool isPointSafelyInsideContainer(geometry_msgs::Point p_world, std::string container_name, double safety_margin);
  geometry_msgs::Point movePointIntoContainerBounds(geometry_msgs::Point p_world, std::string container_name, double safety_margin);
  void containerPoseToWorld(std::string container_name, geometry_msgs::Pose& pose_in_container, geometry_msgs::Pose& pose_in_world);
  std::vector<std::string> getCloseBorders(geometry_msgs::Point p_world, std::string container_name, double detection_margin);
  double getSafeGraspRotation(geometry_msgs::Point p_world, double theta, std::string container_name);
  double getSafeSuckRotation(geometry_msgs::Point p_world, double theta, std::string container_name);
  geometry_msgs::Pose getSuctionCupPoseToSuckItemInContainer(geometry_msgs::Point p, std::string container_name);
  geometry_msgs::Pose checkIfPoseInContainer(geometry_msgs::Pose& item_pose, int container_id, bool useSuction); // Checks if a pose is inside one of the containers, returns a "corrected" pose if not, and displays a warning.
  geometry_msgs::Point getPointHighAboveContainer(std::string container_name, const double height = 0.2);
  bool checkIfSuctionTargetOK(geometry_msgs::Pose target_pose, std::string container_name); 
  bool pickUpItem(geometry_msgs::Pose target_pose, bool gripper_EE_is_used, int force = 40, int width = 10, bool set_up_EE = true);   // Takes the item with the selected tool

  // Helpers that call other nodes' servicse
  bool moveEETo(geometry_msgs::Pose target_pose, bool use_gripper_EE, bool use_lin_motion = true, bool wait=true, bool use_moveit_planning = true);
  bool moveGripperTo(geometry_msgs::Pose target_pose, bool use_lin_motion = true, bool wait=true, bool use_moveit_planning = true);
  bool moveSuctionCupTo(geometry_msgs::Pose target_pose, bool use_lin_motion = true, bool wait=true, bool use_moveit_planning = true);
  void publishPoseMarker(geometry_msgs::Pose pose, std::string name);
  void publishGraspPoseMarker(geometry_msgs::Quaternion pad_orientation, geometry_msgs::Point pad_point_1, geometry_msgs::Point pad_point_2);
  void setLED(std::string color);

  bool setEEStatus(bool gripper_closed, bool suction_on, bool gripper_extended, bool suction_extended,
                  int gripper_control_param, int suction_force = 40);
  
  /**
   * Start/set suction with specific force and rotation (last DOF).
   * @param force this is an integer from 0 (minimum force) to 40 (maximum force).
   * @return true if succeeds
   */
  bool StartSuction(const int& force = 40);

  /**
   * Set the rotation of the suction cup (last DOF).
   * @param rotation double from 0 [rad] (straight along the tube) to PI/2 [rad] (one side).
   * @return true if succeeds
   */
  bool SetSuctionRotation(const double& rotation);

  bool StopSuction();
  bool AdvanceSuction();
  bool RetractSuction();

  /**
   * Close the gripper with force control.
   * @param force this is an integer from 0 (minimum force) to 100 (maximum force).
   * @note you can increase the force but there is a bug making not possible to reduce the force
   * @note to open the gripper you need to SetGripperWidth
   * @return true if succeeds
   */
  bool CloseGripperRelativeForceControl(const int& force);

  /**
   * Set the gripper with position control.
   * @param width [mm] this is an integer from 0[mm] (fully closed) to 110 [mm] (fully open).
   * @note use this to set a specific with in case the space to approach the object is too narrow
   * @note this method has a very limited compliance so it can damage objects
   * @return true if succeeds
   */
  bool SetGripperOpeningWidth(const int& width);

  bool AdvanceGripper();
  bool RetractGripper();

  // Service callback declarations
  bool calibrationMotionCallback(tnp_kuka_motion::calibrationMotion::Request &req, tnp_kuka_motion::calibrationMotion::Response &res);
  bool goToContainerCallback(tnp_kuka_motion::goToContainer::Request &req, tnp_kuka_motion::goToContainer::Response &res);
  bool goToHomeCallback(tnp_kuka_motion::goToHome::Request &req, tnp_kuka_motion::goToHome::Response &res);
  bool goToToteCallback(tnp_kuka_motion::goToTote::Request &req, tnp_kuka_motion::goToTote::Response &res);
  bool goToAmnestyCallback(tnp_kuka_motion::goToAmnesty::Request &req, tnp_kuka_motion::goToAmnesty::Response &res);
  bool goToRecSpaceCallback(tnp_kuka_motion::goToRecSpace::Request &req, tnp_kuka_motion::goToRecSpace::Response &res);
  bool goToBinCallback(tnp_kuka_motion::goToBin::Request &req, tnp_kuka_motion::goToBin::Response &res);
  bool goToBoxCallback(tnp_kuka_motion::goToBox::Request &req, tnp_kuka_motion::goToBox::Response &res);
  bool goToLookIntoContainerCallback(tnp_kuka_motion::goToLookIntoContainer::Request &req,
                                tnp_kuka_motion::goToLookIntoContainer::Response &res);
  bool goToLookIntoToteCallback(tnp_kuka_motion::goToLookIntoTote::Request &req,
                                tnp_kuka_motion::goToLookIntoTote::Response &res);
  bool goToLookIntoAmnestyCallback(tnp_kuka_motion::goToLookIntoAmnesty::Request &req,
                                   tnp_kuka_motion::goToLookIntoAmnesty::Response &res);
  bool goToLookIntoBinCallback(tnp_kuka_motion::goToLookIntoBin::Request &req,
                             tnp_kuka_motion::goToLookIntoBin::Response &res);
  bool goToLookIntoBoxCallback(tnp_kuka_motion::goToLookIntoBox::Request &req,
                           tnp_kuka_motion::goToLookIntoBox::Response &res);
  bool canRobotPlaceItemHereCallback(tnp_kuka_motion::canRobotPlaceItemHere::Request &req,
                              tnp_kuka_motion::canRobotPlaceItemHere::Response &res);
  bool putItemIntoContainerCallback(tnp_kuka_motion::putItemIntoContainer::Request &req,
                              tnp_kuka_motion::putItemIntoContainer::Response &res);
  bool putItemIntoToteCallback(tnp_kuka_motion::putItemIntoTote::Request &req,
                              tnp_kuka_motion::putItemIntoTote::Response &res);
  bool putItemIntoAmnestyCallback(tnp_kuka_motion::putItemIntoAmnesty::Request &req,
                              tnp_kuka_motion::putItemIntoAmnesty::Response &res);
  bool putItemIntoBinCallback(tnp_kuka_motion::putItemIntoBin::Request &req,
                              tnp_kuka_motion::putItemIntoBin::Response &res);
  bool putItemIntoBoxCallback(tnp_kuka_motion::putItemIntoBox::Request &req,
                              tnp_kuka_motion::putItemIntoBox::Response &res);
  
  bool moveToJointAnglesPTPCallback(tnp_kuka_motion::moveToJointAnglesPTP::Request &req,
                                    tnp_kuka_motion::moveToJointAnglesPTP::Response &res);
  bool moveToCartPosePTPCallback(tnp_kuka_motion::moveToCartPosePTP::Request &req,
                                 tnp_kuka_motion::moveToCartPosePTP::Response &res);
  bool moveSuctionCupToCallback(tnp_kuka_motion::moveSuctionCupTo::Request &req,
                                 tnp_kuka_motion::moveSuctionCupTo::Response &res);
  bool moveGripperToCallback(tnp_kuka_motion::moveGripperTo::Request &req,
                                 tnp_kuka_motion::moveGripperTo::Response &res);

  bool sweepToteHorizontalCallback(tnp_kuka_motion::sweepToteHorizontal::Request &req, tnp_kuka_motion::sweepToteHorizontal::Response &res);
  bool sweepToteCornersCallback(tnp_kuka_motion::sweepToteCorners::Request &req, tnp_kuka_motion::sweepToteCorners::Response &res);
  bool nudgeItemsIntoContainerCallback(tnp_kuka_motion::nudgeItemsIntoContainer::Request &req, tnp_kuka_motion::nudgeItemsIntoContainer::Response &res);
  bool suckItemCallback(tnp_kuka_motion::suckItem::Request &req, tnp_kuka_motion::suckItem::Response &res);
  bool graspItemCallback(tnp_kuka_motion::graspItem::Request &req, tnp_kuka_motion::graspItem::Response &res);
  bool wobbleForVisionCallback(tnp_kuka_motion::wobbleForVision::Request &req,
                               tnp_kuka_motion::wobbleForVision::Response &res);

  // Subscriber callback declarations
  void itemIsSuctionedCallback(const std_msgs::Int16::ConstPtr& input);
  void itemIsGraspedCallback(const std_msgs::Int16::ConstPtr& input);
  void cupIsTouchingSomethingCallback(const std_msgs::Int16::ConstPtr& input);
  void gripperIsTouchingSomethingCallback(const std_msgs::Int16::ConstPtr& input);

  bool goToContainer(std::string container_name);
  bool goToContainerWithFixedA7(std::string container_name);
  bool goToLookIntoContainer(std::string container_name, int height);
  bool canRobotPlaceItemHere(geometry_msgs::Pose target_pose, std::string container_name, bool use_gripper_EE, int calculation_mode = 0);
  bool putItemIntoContainer(std::string container_name, bool gripper_EE_is_used, geometry_msgs::Point target_point_in_world, double rotation_angle = 0);

  // Actions
  void execute(const tnp_kuka_motion::moveToJointAnglesPTPGoalConstPtr& goal);

private:
  ros::NodeHandle n_;
  ros::Publisher pubJointPos_;
  ros::Publisher pubCartPose_;
  ros::Subscriber subItemIsSuctioned_;
  ros::Subscriber subItemIsGrasped_;
  ros::Subscriber subCupIsTouchingSomething_;
  ros::Subscriber subGripperIsTouchingSomething_;

  // Service declarations
  ros::ServiceServer calibrationMotionService;
  ros::ServiceServer goToContainerService;
  ros::ServiceServer goToHomeService;
  ros::ServiceServer goToToteService;
  ros::ServiceServer goToAmnestyService;
  ros::ServiceServer goToRecSpaceService;
  ros::ServiceServer goToBinService;
  ros::ServiceServer goToBoxService;
  ros::ServiceServer goToLookIntoContainerService;
  ros::ServiceServer goToLookIntoToteService;
  ros::ServiceServer goToLookIntoAmnestyService;
  ros::ServiceServer goToLookIntoBinService;
  ros::ServiceServer goToLookIntoBoxService;
  ros::ServiceServer canRobotPlaceItemHereService;
  ros::ServiceServer putItemIntoContainerService;
  ros::ServiceServer putItemIntoToteService;
  ros::ServiceServer putItemIntoAmnestyService;
  ros::ServiceServer putItemIntoBinService;
  ros::ServiceServer putItemIntoBoxService;
  ros::ServiceServer moveToJointAnglesPTPService;
  ros::ServiceServer moveToCartPosePTPService;
  ros::ServiceServer moveSuctionCupToService;
  ros::ServiceServer moveGripperToService;
  ros::ServiceServer sweepToteHorizontalService;
  ros::ServiceServer sweepToteCornersService;
  ros::ServiceServer nudgeItemsIntoContainerService;
  ros::ServiceServer suckItemService;
  ros::ServiceServer graspItemService;
  ros::ServiceServer wobbleForVisionService;

  // Service clients
  // End effector
  ros::ServiceClient gripperClient;
  ros::ServiceClient suctionClient;
  ros::ServiceClient suctionRotationClient;
  ros::ServiceClient LinActuatorSuctionClient;
  ros::ServiceClient LinActuatorGripperClient;
  ros::ServiceClient EEControlClient;

  ros::ServiceClient moveGripperToClient;
  ros::ServiceClient moveSuctionCupToClient;
  ros::ServiceClient getFlangePoseAtEEPoseClient;
  ros::ServiceClient isEEPoseReachableClient;

  ros::ServiceClient showPoseClient;
  ros::ServiceClient showGraspPoseClient;
  ros::ServiceClient LEDClient;
  ros::ServiceClient setPathParametersLinClient;

  // Joint angles and positions we use
  iiwa_msgs::JointPosition atHome, atTote, lookIntoTote, atAmnesty, lookIntoAmnesty, atRecSpace, atBinA, atBinA1, atBinA2, atBinB, atBinC,
      atBox1, atBox2, atBox3, lookIntoBinA, lookIntoBinA1, lookIntoBinA2, lookIntoBinB, lookIntoBinC, lookIntoBox1, lookIntoBox2, lookIntoBox3;
  iiwa_msgs::JointPosition lookIntoBinAhigh, lookIntoBinAmid, lookIntoBinAlow, lookIntoBinBhigh, lookIntoBinBmid, lookIntoBinBlow,
  lookIntoBinChigh, lookIntoBinCmid, lookIntoBinClow, lookIntoToteHigh, lookIntoToteMid, lookIntoToteLow,
  lookIntoAmnestyHigh, lookIntoAmnestyMid, lookIntoAmnestyLow,
  lookAtRecSpaceCube;
  geometry_msgs::Pose atHome_pose, atTote_pose, lookIntoTote_pose, atAmnesty_pose, lookIntoAmnesty_pose, atRecSpace_pose, atBinA_pose, atBinB_pose, atBinC_pose,
      atBox1_pose, atBox2_pose, atBox3_pose, lookIntoBinA_pose, lookIntoBinB_pose, lookIntoBinC_pose, lookIntoBox1_pose, lookIntoBox2_pose, lookIntoBox3_pose;
  geometry_msgs::Pose lookIntoBinAhigh_pose, lookIntoBinAmid_pose, lookIntoBinAlow_pose, lookIntoBinBhigh_pose, lookIntoBinBmid_pose, lookIntoBinBlow_pose,
  lookIntoBinChigh_pose, lookIntoBinCmid_pose, lookIntoBinClow_pose, lookIntoToteHigh_pose, lookIntoToteMid_pose, lookIntoToteLow_pose,
  lookIntoAmnestyHigh_pose, lookIntoAmnestyMid_pose, lookIntoAmnestyLow_pose,
  lookAtRecSpaceCube_pose, atRecSpace_pose_EE;

  bool kuka_is_dead = false;
  bool itemIsSuctioned = false; // Local copy of the value we subscribe to. There's probably a better way to do this.
  bool itemIsGrasped = false; // Local copy of the value we subscribe to. There's probably a better way to do this.
  bool cupIsTouchingSomething = false; // Local copy of the value we subscribe to. There's probably a better way to do this
  bool gripperIsTouchingSomething = false; // Local copy of the value we subscribe to. There's probably a better way to do this.
  double suction_safety_margin_to_borders_ = 0.06;
  double grasping_safety_margin_to_borders_ = 0.07;

  geometry_msgs::Twist last_sent_vel_limits;

  // Action declarations
  actionlib::SimpleActionServer<tnp_kuka_motion::moveToJointAnglesPTPAction> moveToJointAnglesPTPActionServer_;

  iiwa_ros::iiwaRos my_iiwa_ros_object; // for time_to_destination

  tf::TransformListener tflistener;
  bool arc_task_type_is_stow;
  bool arc_task_type_is_pick;
};
//End of class SubscribeAndPublish

#endif

