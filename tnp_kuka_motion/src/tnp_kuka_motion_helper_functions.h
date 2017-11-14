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

#include "visualization_msgs/Marker.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "iiwa_ros/iiwa_ros.h"

#include <tf/transform_datatypes.h>
#include <termios.h>  // non-blocking getchar

#include "ros/ros.h"
#include <ros/console.h>

#include <math.h>
#include <algorithm>  //For min
#include <string>


void rotatePoseByRPY(const double roll, const double pitch, const double yaw, geometry_msgs::Pose& inpose)
{
  tf::Quaternion q;
  tf::Quaternion qrotate = tf::createQuaternionFromRPY(roll, pitch, yaw);

  tf::quaternionMsgToTF(inpose.orientation, q);

  q = q * qrotate;

  tf::quaternionTFToMsg(q, inpose.orientation);
}

bool isKUKANearTarget(iiwa_msgs::JointPosition& target_jp, iiwa_ros::iiwaRos& iiwa_ros_object)
{
  iiwa_msgs::JointPosition jp_now;
  iiwa_ros_object.getJointPosition(jp_now);
  double residual =  abs(jp_now.position.a1 - target_jp.position.a1) +
                    abs(jp_now.position.a2 - target_jp.position.a2) +
                    abs(jp_now.position.a3 - target_jp.position.a3) +
                    abs(jp_now.position.a4 - target_jp.position.a4) +
                    abs(jp_now.position.a5 - target_jp.position.a5) +
                    abs(jp_now.position.a6 - target_jp.position.a6) +
                    abs(jp_now.position.a7 - target_jp.position.a7);
  if (residual < (15.0/180.0)*M_PI)     // 15 degrees
  {
    return true;
  }
  else
  {
    ROS_INFO_STREAM("KUKA is too far away from target joint angles (" << residual << "). Returning false.");
    return false;
  }
}

bool isKUKANearTarget(geometry_msgs::Pose& target_pose, iiwa_ros::iiwaRos& iiwa_ros_object)
{
  geometry_msgs::PoseStamped ps_now;
  iiwa_ros_object.getCartesianPose(ps_now);
  double residual =  abs(target_pose.position.x - ps_now.pose.position.x) +
                    abs(target_pose.position.y - ps_now.pose.position.y) +
                    abs(target_pose.position.z - ps_now.pose.position.z);
  if (residual < .05)      // 5 cm
  {
    return true;
  }
  else
  {
    ROS_INFO_STREAM("KUKA is too far away from target pose (" << residual << "). Returning false.");
    return false;
  }
}

// Returns the angle between two quaternions
double quaternionDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) 
{ 
  tf::Quaternion q1tf, q2tf;
  tf::quaternionMsgToTF(q1, q1tf);
  tf::quaternionMsgToTF(q2, q2tf);
  return 2*q1tf.angle(q2tf); 
}

double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  tf::Point tp1, tp2;
  tf::pointMsgToTF(p1, tp1);
  tf::pointMsgToTF(p2, tp2);
  return tfDistance(tp1, tp2);
}

// Only used for the stow task
int guessTargetContainer(geometry_msgs::Pose& target_pose)
{
  ///// Guess target container by the coordinates of its pose
  // 0 = tote
  // 1 = bin A
  // 2 = bin B
  // 3 = bin C
  // 4 = amnesty
  ros::NodeHandle n_;
  int target_container = 1;
  if ((target_pose.position.y < -.45) ) {target_container = 0;}    // Tote
  if (target_pose.position.x >= .4) {target_container = 1;}         // Bin A
  if (target_pose.position.y > 0.0)
  {
    if ((target_pose.position.x < 0.4) && (target_pose.position.x >= -0.1)) {target_container = 2;}    // Bin B
    if (target_pose.position.x < -0.1) {target_container = 3;}    // Bin C
  }
  return target_container;
}

// These two functions convert between container name and ID.
std::string getContainerName(int container_id)
{
  std::string container_name;
  if (container_id == 0) // Tote
  {
   container_name = "tote";
  }
  else if (container_id == 1) // Bin A
  {
   container_name = "bin_A";
  }
  else if (container_id == 2) // Bin B
  {
   container_name = "bin_B";
  }
  else if (container_id == 3) // Bin C
  {
   container_name = "bin_C";
  }
  else if (container_id == 4) // Amnesty
  {
   container_name = "amnesty";
  }
  else if (container_id == 5) // Box 1
  {
   container_name = "box_1";
  }
  else if (container_id == 6) // Box 2
  {
   container_name = "box_2";
  }
  else if (container_id == 7) // Box 3
  {
   container_name = "box_3";
  }
  else if (container_id == 8) // Box 2
  {
   container_name = "bin_A_1";
  }
  else if (container_id == 9) // Box 3
  {
   container_name = "bin_A_2";
  }
  return container_name;
}

int getContainerID(std::string container_name)
{
  int container_id = 0;
  if (container_name.compare("tote") == 0) // Tote
  {
   container_id = 0;
  }
  else if (container_name.compare("bin_A") == 0)
  {
   container_id = 1;
  }
  else if (container_name.compare("bin_B") == 0)
  {
   container_id = 2;
  }
  else if (container_name.compare("bin_C") == 0)
  {
   container_id = 3;
  }
  else if (container_name.compare("amnesty") == 0)
  {
   container_id = 4;
  }
  else if (container_name.compare("box_1") == 0)
  {
   container_id = 5;
  }
  else if (container_name.compare("box_2") == 0)
  {
   container_id = 6;
  }
  else if (container_name.compare("box_3") == 0)
  {
   container_id = 7;
  }
  else if (container_name.compare("bin_A_1") == 0)
  {
   container_id = 8;
  }
  else if (container_name.compare("bin_A_2") == 0)
  {
   container_id = 9;
  }
  return container_id;

}

// (c) Salvo Virga, sankyu~~
// Transforms a stamped pose from its current reference frame (as given in its header) to referenceFrame
geometry_msgs::PoseStamped transform_pose_now(geometry_msgs::PoseStamped& pose, const std::string& referenceFrame, const tf::TransformListener& listener) 
{   
  // Check if the frames are different
  if (pose.header.frame_id != referenceFrame ) {

    bool success = false;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped result_pose;

    while (!success) {
      try {
        ros::Time t = ros::Time::now();
        pose.header.stamp = t;
        listener.waitForTransform(pose.header.frame_id, referenceFrame, t, ros::Duration(3.0));
        listener.lookupTransform(pose.header.frame_id, referenceFrame, t, transform);
        listener.transformPose(referenceFrame, pose, result_pose);
        success = true;
        return result_pose;
      } catch (tf::ExtrapolationException e) {
        // ROS_ERROR(e.what());
        ROS_ERROR("Something went wrong in transform_pose_now.");
      }
      sleep(0.1);
    }
  }
  return pose;
}


// container_name has to be "tote", "amnesty", "box_1", bin_A", "bin_B"... like the TF frames
geometry_msgs::Point containerPointToWorld(std::string container_name, geometry_msgs::Point& point_in_container, const tf::TransformListener& tflistener)
{
  // ROS_INFO_STREAM("Transforming item pose of container to world.");
  geometry_msgs::PoseStamped container_pstamped, world_pstamped;
  container_pstamped.pose.position = point_in_container;
  container_pstamped.pose.orientation.w = 1.0;
  container_pstamped.header.frame_id = "/" + container_name;
  world_pstamped = transform_pose_now(container_pstamped, "/iiwa_link_0", tflistener);
  // ROS_INFO_STREAM("Transformed point of container to world. Container point:" << point_in_container << "; and in world: " << world_pstamped.pose.position);
  return world_pstamped.pose.position;
}

// container_name has to be "tote", "amnesty", "box_1", bin_A", "bin_B"... like the TF frames
geometry_msgs::Point worldPointToContainer(std::string container_name, geometry_msgs::Point& point_in_world, const tf::TransformListener& tflistener)
{
  geometry_msgs::PoseStamped ps_world, ps_in_container;
  ps_world.pose.position = point_in_world;
  ps_world.pose.orientation.w = 1.0;
  ps_world.header.frame_id = "/iiwa_link_0";
  ps_in_container = transform_pose_now(ps_world, "/" + container_name, tflistener);
  return ps_in_container.pose.position;
}

// Check if an orientation is permissible (within tolerance of target_rotation) and if not, flip it by 180 degrees
double flipGraspRotationIfNecessary(double in_rotation, double target_rotation, double tolerance)
{
  // Thank you internet!!! https://github.com/petercorke/toolbox-common-matlab/blob/master/angdiff.m
  double angdiff = in_rotation - target_rotation;
  angdiff = fmod(angdiff+M_PI, 2.0*M_PI) - M_PI;

  if (abs(angdiff) > tolerance)
  {
  // Flip rotation
    if (in_rotation <= 0.0)
    {
      return (in_rotation + M_PI);
    }
    if (in_rotation > 0.0)
    {
      return (in_rotation - M_PI);
    }
  }
  return in_rotation;
}

// Returns the correct orientation of the end effector to go into bin A
// The orientation is chosen so the tool is furthest away from the robot, and thus increases the range.
// ---
// "If you have rotations A*B you can think of it as applying A as a global rotation to B. Or as applying B as a local rotation to A." -( http://answers.unity3d.com/questions/810579/quaternion-multiplication-order.html )
geometry_msgs::Quaternion getTiltedEEOrientation(double target_x, double target_y, bool useSuction){
  double alpha = 0.0;   
  
  tf::Quaternion q_out, q_turn;
  tf::Quaternion q_tilt = tf::createQuaternionFromRPY(0.0, (-15.0/180.0)*M_PI, 0.0);
  tf::Quaternion q_suck(.707, .707, .0, .0);
  tf::Quaternion q_grip(-.707, .707, .0, .0);
  if (useSuction) {q_out = q_suck;}
  else {q_out = q_grip;}

  ROS_INFO_STREAM("Horizontal EE distance from origin: " << sqrt(target_y*target_y+target_x*target_x));
  if (sqrt(target_y*target_y+target_x*target_x) > .75)    // This distance is roughly when reachability issues start, so we incline the gripper a bit
    // at 66 cm or less (limit untested) the robot cannot go down enough.
  {
    ROS_INFO("Tilting the end effector :~)");
    q_out = q_tilt*q_out;
  }

  // Turn the EE around global z
  alpha = atan2(target_y, target_x);
  ROS_WARN_STREAM("Turning the end effector around z by angle: " << (alpha/M_PI)*180.0);
  q_turn = tf::createQuaternionFromRPY(0.0, 0.0, alpha);
  q_out = q_turn*q_out;

  geometry_msgs::Quaternion q_tilt_msg, q_turn_msg, q_out_msg1;
  tf::quaternionTFToMsg(q_tilt, q_tilt_msg);
  tf::quaternionTFToMsg(q_turn, q_turn_msg);
  tf::quaternionTFToMsg(q_out, q_out_msg1);
  ROS_INFO_STREAM("q_tilt_msg: " << q_tilt_msg);
  ROS_INFO_STREAM("q_turn_msg: " << q_turn_msg);
  ROS_INFO_STREAM("q_out_msg: " << q_out_msg1);

  geometry_msgs::Quaternion q_out_msg;
  tf::quaternionTFToMsg(q_out, q_out_msg);
  return q_out_msg;
}

geometry_msgs::Quaternion getRotatedNeutralOrientationForContainer(std::string container_name, double rotation_angle_rad)
{
  tf::Quaternion q_bin_A = tf::createQuaternionFromRPY(0.0, (180.0/180.0)*M_PI, 0.0);
  tf::Quaternion q_rotate_left = tf::createQuaternionFromRPY(0.0, 0.0, (90.0/180.0)*M_PI);
  tf::Quaternion q_rotate_right = tf::createQuaternionFromRPY(0.0, 0.0, (-90.0/180.0)*M_PI);
  tf::Quaternion q_rotate_extra = tf::createQuaternionFromRPY(0.0, 0.0, rotation_angle_rad);
  tf::Quaternion q_container;

  geometry_msgs::Quaternion q_out;

  if (container_name.compare("tote") == 0)
  {
    q_container = q_rotate_extra * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("bin_A") == 0)
  {
    q_container = q_rotate_extra * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("bin_B") == 0)
  {
    q_container = q_rotate_extra * q_rotate_left * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("bin_C") == 0)
  {
    q_container = q_rotate_extra * q_rotate_left * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("amnesty") == 0)
  {
    q_container = q_rotate_extra * q_rotate_right * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("box_1") == 0)
  {
    q_container = q_rotate_extra * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("box_2") == 0)
  {
    q_container = q_rotate_extra * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }
  else if (container_name.compare("box_3") == 0)
  {
    q_container = q_rotate_extra * q_rotate_right * q_rotate_right * q_bin_A;
    tf::quaternionTFToMsg(q_container, q_out);
  }

  return q_out;
}

// Returns the quaternion for which A7 is closest to 0 for each container
geometry_msgs::Quaternion getNeutralRotationForContainer(std::string container_name)
{
  return getRotatedNeutralOrientationForContainer(container_name, 0);
}

// Non-blocking getchar
int getch()
{
  ROS_INFO("Press any key to continue...");
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON); // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

// Used to restrict rotation values to a certain interval.
double restrictValueToInterval(double input_value, double allowed_min, double allowed_max)
{
  if (input_value > allowed_max)
  {
    input_value = allowed_max;
  }
  else if (input_value < allowed_min)
  {
    input_value = allowed_min;
  }
  return input_value;
}

// Returns how far the value is from the interval
double distOfValueToInterval(double input_value, double allowed_min, double allowed_max)
{
  if (input_value > allowed_max)
  {
    return abs(input_value - allowed_max);
  }
  else if (input_value < allowed_min)
  {
    return abs(input_value - allowed_min);
  }
  else
  {
    return input_value;  
  }
}

// This either flips or rotates an input rotation value to the desired interval
// It is used during the grasping calculation to obtain a safe grasping position
// It flips and/or adjusts the input rotation to allow the least deviation from the desired grasp rotation
double getSafeRotationValueInCorner(double input_rotation_value, double ideal_permissible_rotation_value)
{
  double deg45 = (45.0/180.0)*M_PI;
  return restrictValueToInterval(input_rotation_value, ideal_permissible_rotation_value - deg45, ideal_permissible_rotation_value + deg45);
}

geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w)
{
  geometry_msgs::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::Pose makePose()
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(0.0, 0.0, 0.0);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p)
{
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z, geometry_msgs::Quaternion q)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = q;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p, double xq, double yq, double zq, double w)
{
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z, double xq, double yq, double zq, double w)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}

bool twistEqual(geometry_msgs::Twist t1, geometry_msgs::Twist t2)
{
  if ((t1.linear.x == t2.linear.x) && 
      (t1.linear.y == t2.linear.y) && 
      (t1.linear.z == t2.linear.z) && 
      (t1.angular.x == t2.angular.x) && 
      (t1.angular.y == t2.angular.y) && 
      (t1.angular.z == t2.angular.z))
  {
    return true;
  }
  else
  {
    return false;
  }
}