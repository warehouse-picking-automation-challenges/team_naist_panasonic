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

#include "tnp_moveit_planner.h"

TNP_Moveit_Planning::TNP_Moveit_Planning() : suction_group("suction") , gripper_group("gripper"), camera_group("camera"), flange_group("flange"), robot_model_loader("robot_description")

{
  ROS_INFO_STREAM("Constructor of TNP_Moveit_Planning class that contains movegroup has started.");

  // services you want to advertise
  moveCameraToService = n_.advertiseService("tnp_moveit_planner/moveCameraTo",
                                                 &TNP_Moveit_Planning::moveCameraToCallback, this);
  moveGripperToService = n_.advertiseService("tnp_moveit_planner/moveGripperTo",
                                                 &TNP_Moveit_Planning::moveGripperToCallback, this);
  moveSuctionCupToService = n_.advertiseService("tnp_moveit_planner/moveSuctionCupTo",
                                                 &TNP_Moveit_Planning::moveSuctionCupToCallback, this);
  moveToCartesianPoseService = n_.advertiseService("tnp_moveit_planner/moveToCartesianPose",
                                                 &TNP_Moveit_Planning::moveToCartesianPoseCallback, this);
  setMotionParamsService = n_.advertiseService("tnp_moveit_planner/setMotionParams",
                                                 &TNP_Moveit_Planning::setMotionParamsCallback, this);
  getFlangePoseAtEEPoseService = n_.advertiseService("tnp_moveit_planner/getFlangePoseAtEEPose",
                                                 &TNP_Moveit_Planning::getFlangePoseAtEEPoseCallback, this);
  isEEPoseReachableService = n_.advertiseService("tnp_moveit_planner/isEEPoseReachable",
                                                 &TNP_Moveit_Planning::isEEPoseReachableCallback, this);

  // Configure movegroup planners
  suction_group.setPlanningTime(0.5);
  suction_group.setPlannerId("RRTConnectkConfigDefault");
  suction_group.setEndEffectorLink("tnptool_suction_link_ee");
  gripper_group.setPlanningTime(0.5);
  gripper_group.setPlannerId("RRTConnectkConfigDefault");
  gripper_group.setEndEffectorLink("tnptool_gripper_link_ee");
  camera_group.setPlanningTime(0.5);
  camera_group.setPlannerId("RRTConnectkConfigDefault");
  camera_group.setEndEffectorLink("tnp_ee_camera_frame");
  flange_group.setPlanningTime(0.5);
  flange_group.setPlannerId("RRTConnectkConfigDefault");
  flange_group.setEndEffectorLink("iiwa_link_ee");

  // Initialize the helper class
  my_iiwa.init();
}

bool TNP_Moveit_Planning::moveToPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &group)
{
  bool use_LIN_motion = n_.getParam("/iiwa/tnp_moveit_planner/use_LIN_motion", use_LIN_motion);
  ROS_INFO_STREAM("use_LIN_motion is set to " << use_LIN_motion);

  if (use_LIN_motion)
  {
    moveToPoseLIN(target_pose, group);
  }
  else //Random motion
  {
    geometry_msgs::PoseStamped command_cartesian_position;
    bool success_plan = false, motion_done = false, new_pose = false;
    while (ros::ok()) {
      if (my_iiwa.getRobotIsConnected()) {
        
        command_cartesian_position = group.getCurrentPose();  
        command_cartesian_position.pose = target_pose;
    
        group.setStartStateToCurrentState();
        group.setPoseTarget(command_cartesian_position);
        success_plan = group.plan(myplan);
        if (success_plan) 
        {
          motion_done = group.execute(myplan);
        } 
        else if (!success_plan) 
        {
          ROS_ERROR_STREAM("No path to the target point has been found!");
          return false;
        }
        if (motion_done) 
        {
          break;
        }
      }
      else {
        ROS_WARN_STREAM("Robot is not connected...");
        ros::Duration(2.0).sleep();
      }
    }
    ROS_INFO_STREAM("Debug 1 ");
    return true;
  }
}

bool TNP_Moveit_Planning::moveToPoseLIN(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface &group)
{
  bool success_plan = false, motion_done = false;
  double velocity_scaling_factor, jump_threshold, eef_step;
  n_.getParam("/iiwa/tnp_moveit_planner/velocity_scaling_factor", velocity_scaling_factor);
  n_.getParam("/iiwa/tnp_moveit_planner/jump_threshold", jump_threshold);
  n_.getParam("/iiwa/tnp_moveit_planner/eef_step", eef_step);

  ROS_INFO_STREAM("params: " << velocity_scaling_factor << ", " << jump_threshold << ", " << eef_step);

  geometry_msgs::PoseStamped ps_now_debug;
  my_iiwa.getCartesianPose(ps_now_debug);
  ROS_INFO_STREAM("LIN motion requested, with robot currently at " << ps_now_debug.pose.position);

  group.setMaxVelocityScalingFactor(velocity_scaling_factor);
  ROS_INFO_STREAM("Attempting linear motion; sleeping .5 seconds.");
  ros::Duration(0.5).sleep(); // 3 seconds
  while (ros::ok()) {
    if (my_iiwa.getRobotIsConnected()) {
      std::vector<geometry_msgs::Pose> waypoints;
  
      waypoints.push_back(target_pose);
      
      moveit_msgs::RobotTrajectory trajectory;
      double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      myplan.trajectory_ = trajectory;   // TODO: There's no collision avoidance here. Can this be checked here?
      
      if (fraction) {
        ROS_INFO("Cartesian path has been planned (%.2f%% achieved)", fraction * 100.0);
        motion_done = group.execute(myplan);
        group.move();
      }
      if (fraction < 0.001) // If the planning failed
      {
        ROS_ERROR_STREAM("Planning the cartesian path has failed. Returning false.");
        return false;
      }
      if (motion_done) {
        geometry_msgs::PoseStamped ps_now;
        my_iiwa.getCartesianPose(ps_now);
        double residual =  abs(target_pose.position.x - ps_now.pose.position.x) + 
                          abs(target_pose.position.y - ps_now.pose.position.y) + 
                          abs(target_pose.position.z - ps_now.pose.position.z);
        if (residual < .05)      // 5 cm
        {
          return true;
        }
        else 
        {
          ROS_ERROR_STREAM("After linear motion was reported as done, KUKA is too far away from target pose (" << residual << "). Returning false.");
          return false;
        }
      }
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(2.0).sleep(); // 3 seconds
    }
  }
}

bool TNP_Moveit_Planning::moveCameraToCallback(tnp_moveit_planner::moveCameraTo::Request  &req,
         tnp_moveit_planner::moveCameraTo::Response &res)
{
  ROS_INFO_STREAM("Called moveCameraTo service with target pose: " << req.target_pose);
  return moveToPose(req.target_pose, camera_group);
}

bool TNP_Moveit_Planning::moveGripperToCallback(tnp_moveit_planner::moveGripperTo::Request  &req,
         tnp_moveit_planner::moveGripperTo::Response &res)
{
  ROS_INFO_STREAM("Called moveGripperTo service with target pose: " << req.target_pose);
  return moveToPose(req.target_pose, gripper_group);
}

bool TNP_Moveit_Planning::moveSuctionCupToCallback(tnp_moveit_planner::moveSuctionCupTo::Request  &req,
         tnp_moveit_planner::moveSuctionCupTo::Response &res)
{
  ROS_INFO_STREAM("Called moveSuctionCupTo service with target pose: " << req.target_pose);
  return moveToPose(req.target_pose, suction_group);
}

bool TNP_Moveit_Planning::moveToCartesianPoseCallback(tnp_moveit_planner::moveToCartesianPose::Request  &req,
         tnp_moveit_planner::moveToCartesianPose::Response &res)
{
  ROS_INFO_STREAM("Entered moveToCartesianPose service.");
  return moveToPose(req.target_pose, camera_group);
}

bool TNP_Moveit_Planning::setMotionParamsCallback(tnp_moveit_planner::setMotionParams::Request  &req,
         tnp_moveit_planner::setMotionParams::Response &res)
{
  n_.setParam("/iiwa/tnp_moveit_planner/velocity_scaling_factor", req.velocity_scaling_factor);
  n_.setParam("/iiwa/tnp_moveit_planner/jump_threshold", req.jump_threshold);
  n_.setParam("/iiwa/tnp_moveit_planner/eef_step", req.eef_step);
  ROS_INFO_STREAM("Set tnp_moveit_planner parameters to: " << req.use_LIN_motion << ", " << req.velocity_scaling_factor << ", " << req.jump_threshold << ", " << req.eef_step);
  return true;
}

bool TNP_Moveit_Planning::getFlangePoseAtEEPoseCallback(tnp_moveit_planner::getFlangePoseAtEEPose::Request  &req,
         tnp_moveit_planner::getFlangePoseAtEEPose::Response &res)
{
  ROS_INFO_STREAM("moveit_planner calculating flange pose/A7 angle for " << (req.use_gripper_EE ? "gripper" : "suction") << " pose: " << req.EE_target_pose.position.x << ", " << req.EE_target_pose.position.y << ", " << req.EE_target_pose.position.z);  
  res.pose_is_reachable = getFlangePoseAtEEPose(req.EE_target_pose, res.flange_target_pose, res.A7_angle, req.use_gripper_EE);
  return res.pose_is_reachable;
}

bool TNP_Moveit_Planning::isEEPoseReachableCallback(tnp_moveit_planner::isEEPoseReachable::Request  &req,
         tnp_moveit_planner::isEEPoseReachable::Response &res)
{
  ROS_INFO_STREAM("moveit_planner checking if ee position is in reach for " << (req.use_gripper_EE ? "gripper" : "suction") << ": " << req.EE_target_pose.position.x << ", " << req.EE_target_pose.position.y << ", " << req.EE_target_pose.position.z);  
  geometry_msgs::Pose dummy;
  double dummy_A7_angle;
  res.pose_is_reachable = getFlangePoseAtEEPose(req.EE_target_pose, dummy, dummy_A7_angle, req.use_gripper_EE);
  return res.pose_is_reachable;
}

bool TNP_Moveit_Planning::getFlangePoseAtEEPose(geometry_msgs::Pose EE_target_pose, geometry_msgs::Pose& out_flange_pose, double& A7_angle, bool use_gripper_EE)
{
  // First, we calculate the joint angles at the target pose via the move group
  
  std::string EE_group_name;
  moveit::planning_interface::MoveGroupInterface *EE_group;
  if (use_gripper_EE)
  {
    EE_group_name = "gripper";
    EE_group = &gripper_group;
  }
  else
  {
    EE_group_name = "suction";
    EE_group = &suction_group;
  }
  geometry_msgs::PoseStamped command_cartesian_position = gripper_group.getCurrentPose(target_ee_link);  
  command_cartesian_position.pose = EE_target_pose;

  EE_group->setStartStateToCurrentState();
  EE_group->setPoseTarget(command_cartesian_position, target_ee_link);

  bool success_plan = EE_group->plan(myplan);
  if (!success_plan) 
  {
    ROS_ERROR_STREAM("No path to the target pose has been found!");
    return false;
  }

  // Get the goal joint state
  std::vector<double> goal_joint_angles = myplan.trajectory_.joint_trajectory.points.back().positions;
  ROS_INFO_STREAM("moveit planner calculated as goal joint state: " << ( goal_joint_angles[0] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[1] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[2] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[3] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[4] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[5] /M_PI)*180.0 << ", "
     << ( goal_joint_angles[6] /M_PI)*180.0);
  A7_angle = goal_joint_angles[6];

  // Initialize all the stuff from the kinematic tutorial
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(EE_group_name);
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  // Set the joint values of the target joint pose here!
  kinematic_state->setJointGroupPositions(joint_model_group, goal_joint_angles);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Newly set state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_ee");   // This actually has to be iiwa_link_ee or something

  tf::Pose tfp;
  tf::poseEigenToTF(end_effector_state, tfp);
  tf::poseTFToMsg(tfp, out_flange_pose);
  ROS_INFO_STREAM("Position of iiwa_link_ee in target state: " << out_flange_pose.position.x << ", " << out_flange_pose.position.y << ", " << out_flange_pose.position.z);  

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_moveit_planner");
  ROS_INFO("tnp_moveit_planner node is starting up");
  // Without the AsyncSpinner, the iiwa_ros object does not seem to connect
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Create an object of class TNP_Moveit_Planning that will take care of everything
  TNP_Moveit_Planning SAPObject;

  ROS_INFO("tnp_moveit_planner node has started");

  ros::spin();
  return 0;
}
