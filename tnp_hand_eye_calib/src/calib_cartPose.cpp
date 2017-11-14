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

#include <ros/ros.h>

// Our own services
#include <tnp_hand_eye_calib/calibrateCamera.h>
#include <tnp_hand_eye_calib/checkCalibration.h>

// Services to use the robot
#include "tnp_kuka_motion/moveToCartPosePTP.h"
#include "tnp_moveit_planner/moveSuctionCupTo.h"

// visp functions
#include "visp_hand2eye_calibration/compute_effector_camera.h" 
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h" 

// General
#include <tf/transform_listener.h>    // Includes the TF conversions
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Int8.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// Helper function
float GetRandom(float min,float max)
{
    float ans = min + rand()*(max*100-min*100)/RAND_MAX/100.0;

  return ans;
}

// (c) Salvo Virga, sankyu~~  (copied from tnp_kuka_motions)
geometry_msgs::PoseStamped transform_pose_now(geometry_msgs::PoseStamped& pose, const std::string& referenceFrame, const tf::TransformListener& listener) {   
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
        ROS_ERROR(e.what());
      }
      sleep(0.1);
    }            
  }
  return pose;
}

// End of helper function definitions
// =======================

class KukaCalib
{
public:
    KukaCalib();

    void calibCartPose();
    void calibCartPose_random();
    void publishPoses();

    // Subscriber callbacks
    void trackStatusCallback(const std_msgs::Int8 &status);
    void trackObjectCallback(const geometry_msgs::PoseStamped &pos);
    void effectorPosCallback(const geometry_msgs::PoseStamped &pos);

    // Service callback declarations
    bool calibrateCameraCallback(tnp_hand_eye_calib::calibrateCamera::Request &req, tnp_hand_eye_calib::calibrateCamera::Response &res);
    bool checkCalibrationCallback(tnp_hand_eye_calib::checkCalibration::Request &req, tnp_hand_eye_calib::checkCalibration::Response &res);

    // Publishers for visp hand2eye calibration
    void pose_camera_object_pub_(geometry_msgs::Transform pose_camera_object_);
    void pose_world_effector_pub_(geometry_msgs::Transform pose_world_effector_);
    void pose_world_object_pub_(geometry_msgs::PoseStamped ps_world_object_);

    // Helper functions to pass data to the service
    bool moveToCartPosePTP(double x, double y, double z, double u, double v, double w);
    bool moveToCartPosePTP(geometry_msgs::Pose target_pose);
    bool moveSuctionCupTo(geometry_msgs::Pose target_pose);

    std_msgs::Int8 getTrackStatus(void);
    geometry_msgs::PoseStamped getObjectPose(void);
    geometry_msgs::PoseStamped getEffectorPose(void);

    void computeTransformUsingQuickService(void);
    void computeTransformFromTopicStream(void);

private:
    ros::NodeHandle n_;

    ros::Subscriber trackStatus_sub_;
    ros::Subscriber trackObject_sub_;
    ros::Subscriber effectorPos_sub_;

    ros::Publisher camera_object_publisher_;
    ros::Publisher world_effector_publisher_;
    ros::Publisher world_object_publisher_;

    std_msgs::Int8 track_status;
    geometry_msgs::PoseStamped objectPos;
    geometry_msgs::PoseStamped effectorPos;

    // Service declarations
    ros::ServiceServer calibrateCameraService;
    ros::ServiceServer checkCalibrationService;

    // Services we use
    ros::ServiceClient compute_effector_camera_service_;
    ros::ServiceClient compute_effector_camera_quick_service_;
    // End effector
    ros::ServiceClient moveToCartPosePTPClient;
    ros::ServiceClient moveSuctionCupToClient;

    visp_hand2eye_calibration::compute_effector_camera emc_comm;
    visp_hand2eye_calibration::compute_effector_camera_quick emc_quick_comm;

    tf::TransformListener tf_listener;
};

KukaCalib::KukaCalib()
{
    calibrateCameraService = n_.advertiseService("tnp_hand_eye_calib/calibrateCamera", &KukaCalib::calibrateCameraCallback, this);
    checkCalibrationService = n_.advertiseService("tnp_hand_eye_calib/checkCalibration", &KukaCalib::checkCalibrationCallback, this);

    trackStatus_sub_ = n_.subscribe("/visp_auto_tracker/status",1, &KukaCalib::trackStatusCallback,this);
    trackObject_sub_ = n_.subscribe("/visp_auto_tracker/object_position",1, &KukaCalib::trackObjectCallback,this);
    effectorPos_sub_ = n_.subscribe("/iiwa/state/CartesianPose",1, &KukaCalib::effectorPosCallback,this);

    // Topics we publish
    camera_object_publisher_ = n_.advertise<geometry_msgs::Transform> ("/camera_object", 1000);
    world_effector_publisher_ = n_.advertise<geometry_msgs::Transform> ("/world_effector", 1000);

    world_object_publisher_ = n_.advertise<geometry_msgs::PoseStamped> ("/world_object", 1000);

    // Service we subscribe to
    compute_effector_camera_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera> ("compute_effector_camera");
    compute_effector_camera_quick_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> ("compute_effector_camera_quick");
    moveToCartPosePTPClient = n_.serviceClient<tnp_kuka_motion::moveToCartPosePTP>("/tnp_kuka_motion/moveToCartPosePTP");
    moveSuctionCupToClient = n_.serviceClient<tnp_moveit_planner::moveSuctionCupTo>("/iiwa/tnp_moveit_planner/moveSuctionCupTo");

    ROS_INFO("Kuka Calib node initialized.");
}

void KukaCalib::trackStatusCallback(const std_msgs::Int8 &status){
    track_status.data = status.data;
}

void KukaCalib::trackObjectCallback(const geometry_msgs::PoseStamped &pos){
    objectPos = pos;
}

void KukaCalib::effectorPosCallback(const geometry_msgs::PoseStamped &pos){ 
    effectorPos = pos;
}

bool KukaCalib::calibrateCameraCallback(tnp_hand_eye_calib::calibrateCamera::Request &req, tnp_hand_eye_calib::calibrateCamera::Response &res) 
{
  if (req.movement_type == 0)
  {
    calibCartPose();
  }
  else if (req.movement_type == 1)
  {
    calibCartPose_random();
  }
  else 
  {
    ROS_INFO("No movement type was specified. Aborting.");
    return 0;
  }
  return 1;
}

bool KukaCalib::checkCalibrationCallback(tnp_hand_eye_calib::checkCalibration::Request &req, tnp_hand_eye_calib::checkCalibration::Response &res) 
{
  std::string camframe = "tnp_ee_camera_frame";
  if (req.type_of_check == 0)
  {
    ROS_INFO("Moving suction cup to last seen marker."); 
    geometry_msgs::PoseStamped objectPos, object_world_pose;
    objectPos = getObjectPose();
    objectPos.header.frame_id = "/" + camframe;
    object_world_pose = transform_pose_now(objectPos, "/iiwa_link_0", tf_listener);
    pose_world_object_pub_(object_world_pose);
    geometry_msgs::Quaternion q;
    q.y = 1.0;          
    object_world_pose.pose.orientation = q;
    
    moveSuctionCupTo(object_world_pose.pose);
  }
  else 
  {
    ROS_INFO("No check type was specified. Aborting.");
    return 0;
  }
  return 1;
}

void KukaCalib::pose_camera_object_pub_(geometry_msgs::Transform pose_camera_object_){
    camera_object_publisher_.publish(pose_camera_object_);
    emc_quick_comm.request.camera_object.transforms.push_back(pose_camera_object_);
}

void KukaCalib::pose_world_effector_pub_(geometry_msgs::Transform pose_world_effector_){
  world_effector_publisher_.publish(pose_world_effector_);
  emc_quick_comm.request.world_effector.transforms.push_back(pose_world_effector_);
}

void KukaCalib::pose_world_object_pub_(geometry_msgs::PoseStamped ps_world_object_)
{
  world_object_publisher_.publish(ps_world_object_);
}

bool KukaCalib::moveToCartPosePTP(double x, double y, double z, double u, double v, double w)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = tf::createQuaternionMsgFromRollPitchYaw(u, v, w);

  return moveToCartPosePTP(p);
}

bool KukaCalib::moveToCartPosePTP(geometry_msgs::Pose target_pose)
{
  tnp_kuka_motion::moveToCartPosePTP srv;
  srv.request.target_pose = target_pose;
  if (moveToCartPosePTPClient.call(srv))
  {
    ROS_INFO("moveToCartPosePTP is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_kuka_motion/moveToCartPosePTP");
    return false;
  }
  ros::spinOnce();
  ROS_INFO("done");
  return true;
}

bool KukaCalib::moveSuctionCupTo(geometry_msgs::Pose target_pose)
{
  tnp_moveit_planner::moveSuctionCupTo srv;
  srv.request.target_pose = target_pose;

  bool response = false;
  if (moveSuctionCupToClient.call(srv))
  {
    ROS_INFO("iiwa/tnp_moveit_planner/moveSuctionCupTo service is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service iiwa/tnp_moveit_planner/moveSuctionCupTo");
    return false;
  }
  ros::spinOnce();
  
  ROS_INFO("waiting for .5 seconds after calling the moveSuctionCupTo service.");  
  ros::Duration(.5).sleep();
  return true;
}

std_msgs::Int8 KukaCalib::getTrackStatus(){
    return track_status;
}

geometry_msgs::PoseStamped KukaCalib::getObjectPose(){
    return objectPos;
}

geometry_msgs::PoseStamped KukaCalib::getEffectorPose(){
    return effectorPos;
}

void KukaCalib::computeTransformUsingQuickService()
{
  ROS_INFO("2) QUICK SERVICE:");
  if (compute_effector_camera_quick_service_.call(emc_quick_comm))
  {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
    // This object is a geometry_msgs::Transform, so, via http://answers.ros.org/question/50113/transform-quaternion/
    tf::Quaternion q(emc_quick_comm.response.effector_camera.rotation.x,
                    emc_quick_comm.response.effector_camera.rotation.y,
                    emc_quick_comm.response.effector_camera.rotation.z,
                    emc_quick_comm.response.effector_camera.rotation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

void KukaCalib::computeTransformFromTopicStream()
{
  
  ROS_INFO("3) TOPIC STREAM:");
  if (compute_effector_camera_service_.call(emc_comm))
  {
    ROS_INFO_STREAM("hand_camera: " << std::endl << emc_comm.response.effector_camera);
    // This object is a geometry_msgs::Transform, so, via http://answers.ros.org/question/50113/transform-quaternion/
    tf::Quaternion q(emc_quick_comm.response.effector_camera.rotation.x,
                    emc_quick_comm.response.effector_camera.rotation.y,
                    emc_quick_comm.response.effector_camera.rotation.z,
                    emc_quick_comm.response.effector_camera.rotation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

void KukaCalib::calibCartPose(){
  std_msgs::Int8 trackStatus;
  trackStatus.data = 0;
  ros::Rate loop_rate(1);
  while(trackStatus.data == 0) {
      trackStatus = getTrackStatus();
      loop_rate.sleep();
      ros::spinOnce();
  }

  double centerx, centery, centerz, centeru, centerv, centerw;
  centerx = 0.450;
  centery = 0;
  centerz = 0;
  centeru = +180   /180*M_PI;
  centerv = 0 /180*M_PI;
  centerw = +180   /180*M_PI;

  int xpointNum = 3; //x-direction size of point for calibration
  int ypointNum = 8; //y-direction size of point for calibration
  int zpointNum = 3; //z-direction size of point for calibration

  int pointNum = 50; //size of point for calibration
  int TIMEOUT = 5; //If it is not found after 5 seconds, the next operation is performed
  srand((unsigned int)time(NULL));

  geometry_msgs::Pose Point[pointNum], goal; //Set move area

  //Set move area
  float maxX = +0.600;
  float minX = +0.450;
  float maxY = +0.300;
  float minY = -0.300;
  float maxZ = +0.600;
  float minZ = +0.500;
  
  geometry_msgs::PoseStamped objectPos;
  geometry_msgs::PoseStamped effectorPos;

  double temp_u, temp_v, temp_w;
  
  while(ros::ok()){
      int znum=1;
    while(znum < zpointNum+1){ //z=minZ
      int xnum=1;
      while(xnum < xpointNum+1){
        int ynum=1;
        while(ynum < ypointNum+1){ 
          int num=(xnum-1)*(ypointNum)+ynum-1+(zpointNum-1)*xpointNum*ypointNum;

          Point[num].position.x = minX+0.10*(xnum-1);
          Point[num].position.y = minY+0.10*(ynum-1);
          Point[num].position.z = minZ+0.10*(znum-1);

          double alpha = atan2(fabs(Point[num].position.y-centery),fabs(Point[num].position.z-centerz));
          double beta = atan2(fabs(Point[num].position.x-centerx),fabs(Point[num].position.z-centerz));

          if(Point[num].position.y > 0){
              temp_u = centeru + alpha;
          }else{
              temp_u = centeru - alpha;
          }
          if(Point[num].position.x > 0){
              temp_v = centerv + beta;
          }else{
              temp_v = centerv - beta;
          }
          temp_w = centerw;

          Point[num].orientation = tf::createQuaternionMsgFromRollPitchYaw(temp_u, temp_v, temp_w);

          goal = Point[num];
          ROS_INFO("Moving Kuka-chan to next pose");
          moveToCartPosePTP(goal);
          ynum++;

          ros::spinOnce();
          trackStatus = getTrackStatus();

          // When the marker is tracked, the trackStatus becomes 3
          if(trackStatus.data == 3){
            ROS_INFO("Pose Nr. %d sent!", num);
            objectPos = getObjectPose();
            geometry_msgs::Transform pose_camera_object;
            pose_camera_object.translation.x = objectPos.pose.position.x;
            pose_camera_object.translation.y = objectPos.pose.position.y;
            pose_camera_object.translation.z = objectPos.pose.position.z;
            pose_camera_object.rotation = objectPos.pose.orientation;

            pose_camera_object_pub_(pose_camera_object);

            effectorPos = getEffectorPose();
            geometry_msgs::Transform pose_world_effector;
            pose_world_effector.translation.x = effectorPos.pose.position.x;
            pose_world_effector.translation.y = effectorPos.pose.position.y;
            pose_world_effector.translation.z = effectorPos.pose.position.z;
            pose_world_effector.rotation = effectorPos.pose.orientation;

            pose_world_effector_pub_(pose_world_effector);
            loop_rate.sleep();
          }
            ros::Duration(3).sleep();
            
          }
          xnum++;
      } 
      znum++;   
    }
  }
              
  computeTransformUsingQuickService();
  
  std::wcout << L"end" << L"\n";
}


void KukaCalib::calibCartPose_random()
{
  std_msgs::Int8 trackStatus;
  trackStatus.data = 0;
  ros::Rate loop_rate(1);
  ROS_INFO("Waiting for marker to appear");
  while(trackStatus.data == 0) {
      trackStatus = getTrackStatus();
      loop_rate.sleep();
      ros::spinOnce();
  }

  double centerx, centery, centerz, centeru, centerv, centerw;
  centerx = 0.450;
  centery = 0;
  centerz = 0;
  centeru = +180   /180*M_PI;
  centerv = 0 /180*M_PI;
  centerw = +180   /180*M_PI;

  int xpointNum = 3; //x-direction size of point for calibration
  int ypointNum = 8; //y-direction size of point for calibration
  int zpointNum = 3; //z-direction size of point for calibration

  int pointNum = 100; //size of point for calibration
  int TIMEOUT = 5; //If it is not found after 5 seconds, the next operation is performed
  srand((unsigned int)time(NULL));

  geometry_msgs::Pose Point[pointNum], goal; //Set move area

  //Set move area
  float maxX = +0.600;
  float minX = +0.450;
  float maxY = +0.300;
  float minY = -0.300;
  float maxZ = +0.600;
  float minZ = +0.450;
  
  geometry_msgs::PoseStamped objectPos;
  geometry_msgs::PoseStamped effectorPos;

  double temp_u, temp_v, temp_w;

  ROS_INFO("Starting movements");
  int i = 0;
  while(i<pointNum){

        //set next goal
        goal.position.x = GetRandom(minX,maxX);
        goal.position.y = GetRandom(minY,maxY);
        goal.position.z = GetRandom(minZ,maxZ);
        double alpha = atan2(fabs(goal.position.y-centery),fabs(goal.position.z-centerz));
        double beta = atan2(fabs(goal.position.x-centerx),fabs(goal.position.z-centerz));
        
        if(goal.position.y > 0){
            temp_u = centeru + alpha;
        }else{
            temp_u = centeru - alpha;
        }
        if(goal.position.x > centerx){
            temp_v = centerv - beta;
        }else{
            temp_v = centerv + beta;
        }
        temp_w = centerw;
        
        goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(temp_u, temp_v, temp_w);

        ROS_INFO("Moving Kuka-chan to next pose");
        moveToCartPosePTP(goal);

        ros::spinOnce();
        trackStatus = getTrackStatus();
        int count = 0;
        bool timeout_exit = false;
        while(trackStatus.data != 3){
            ROS_INFO("Where is QR code?");
            trackStatus = getTrackStatus();
            loop_rate.sleep();
            ros::spinOnce();
            count++;
            if(count >= TIMEOUT){
                timeout_exit = true;
                break;
            }
        }

        if(timeout_exit){
            ROS_INFO("QR code NOT found!!");
            continue; 
        }else{
            ROS_INFO("QR code found!!");
        }
        

        if(trackStatus.data == 3){
          ROS_INFO("No.%d pose send!!", i+1);
            objectPos = getObjectPose();
            geometry_msgs::Transform pose_camera_object;
            pose_camera_object.translation.x = objectPos.pose.position.x;
            pose_camera_object.translation.y = objectPos.pose.position.y;
            pose_camera_object.translation.z = objectPos.pose.position.z;
            pose_camera_object.rotation = objectPos.pose.orientation;

            pose_camera_object_pub_(pose_camera_object);

            effectorPos = getEffectorPose();
            geometry_msgs::Transform pose_world_effector;
            pose_world_effector.translation.x = effectorPos.pose.position.x;
            pose_world_effector.translation.y = effectorPos.pose.position.y;
            pose_world_effector.translation.z = effectorPos.pose.position.z;
            pose_world_effector.rotation = effectorPos.pose.orientation;

            pose_world_effector_pub_(pose_world_effector);
          ros::spinOnce();
        }else{
            ROS_INFO("Invalid data - can't send pose");
            ros::spinOnce();
        }

        i++;
  }
 
  computeTransformUsingQuickService();
  
  std::wcout << L"end" << L"\n";
}

// Constantly publishes the seen markers
void KukaCalib::publishPoses() 
{
  ROS_INFO("Looking for a marker to publish.");
  std_msgs::Int8 trackStatus;
  trackStatus.data = 0;
  ros::Rate loop_rate(10);
  while(trackStatus.data == 0) {
      trackStatus = getTrackStatus();
      loop_rate.sleep();
      ros::spinOnce();
  }
  ROS_INFO("Starting to publish the pose of the marker in camera and world coordinate systems.");

  std::string camframe = "tnp_ee_camera_frame";

  geometry_msgs::PoseStamped objectPos, effectorPos, object_world_pose;
  geometry_msgs::Pose pose_camera_object, pose_world_effector;
  
  while(ros::ok())
  {
    ros::spinOnce();
    trackStatus = getTrackStatus();

    if(trackStatus.data == 3) // = Tracking the model/QR code. http://wiki.ros.org/visp_auto_tracker
    {
      ROS_DEBUG("Publishing marker pose.");
      objectPos = getObjectPose();
      geometry_msgs::Transform pose_camera_object;
      pose_camera_object.translation.x = objectPos.pose.position.x;
      pose_camera_object.translation.y = objectPos.pose.position.y;
      pose_camera_object.translation.z = objectPos.pose.position.z;
      pose_camera_object.rotation = objectPos.pose.orientation;
      camera_object_publisher_.publish(pose_camera_object);

      effectorPos = getEffectorPose();
      geometry_msgs::Transform pose_world_effector;
      pose_world_effector.translation.x = effectorPos.pose.position.x;
      pose_world_effector.translation.y = effectorPos.pose.position.y;
      pose_world_effector.translation.z = effectorPos.pose.position.z;
      pose_world_effector.rotation = effectorPos.pose.orientation;
      world_effector_publisher_.publish(pose_world_effector);

      objectPos.header.frame_id = "/" + camframe;
      object_world_pose = transform_pose_now(objectPos, "/iiwa_link_0", tf_listener);
      world_object_publisher_.publish(object_world_pose);
    }
    else 
    {
      ROS_DEBUG("No marker seen.");
    }
    loop_rate.sleep();
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Kuka_calib"); 
  KukaCalib kuka_calib;

  bool checkCamCalib = false;
  ros::NodeHandle nh;
  nh.param<bool>("check_camera_calibration", checkCamCalib, false);
  if (checkCamCalib)
  {
    ros::AsyncSpinner(1);
    kuka_calib.publishPoses();
  }

  ros::spin();
  return 0;
}
