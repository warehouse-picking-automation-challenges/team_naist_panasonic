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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "tnp_kuka_motion/moveToCartPosePTPAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "visp_hand2eye_calibration/compute_effector_camera.h" 
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h" 
#include "std_msgs/Int8.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>

class KukaCalib
{
public:
    KukaCalib();

    void trackStatusCallback(const std_msgs::Int8 &status);
    void trackObjectCallback(const geometry_msgs::PoseStamped &pos);
    void effectorPosCallback(const geometry_msgs::PoseStamped &pos);

    void pose_camera_object_pub_(geometry_msgs::Transform pose_camera_object_);
    void pose_world_effector_pub_(geometry_msgs::Transform pose_world_effector_);

    std_msgs::Int8 getTrackStatus(void);
    geometry_msgs::PoseStamped getObjectPos(void);
    geometry_msgs::PoseStamped geteffectorPos(void);

    void computeUsingQuickService(void);
    void computeFromTopicStream(void);

private:
    ros::Subscriber trackStatus_sub_;
    ros::Subscriber trackObject_sub_;
    ros::Subscriber effectorPos_sub_;

    ros::Publisher camera_object_publisher_;
    ros::Publisher world_effector_publisher_;

    std_msgs::Int8 track_status;
    geometry_msgs::PoseStamped objectPos;
    geometry_msgs::PoseStamped effectorPos;

    ros::ServiceClient compute_effector_camera_service_;
    ros::ServiceClient compute_effector_camera_quick_service_;

    visp_hand2eye_calibration::compute_effector_camera emc_comm;
    visp_hand2eye_calibration::compute_effector_camera_quick emc_quick_comm;

};

KukaCalib::KukaCalib(){
    ros::NodeHandle n_;
    trackStatus_sub_ = n_.subscribe("/visp_auto_tracker/status",1, &KukaCalib::trackStatusCallback,this);
    trackObject_sub_ = n_.subscribe("/visp_auto_tracker/object_position",1, &KukaCalib::trackObjectCallback,this);
    effectorPos_sub_ = n_.subscribe("/iiwa/state/CartesianPose",1, &KukaCalib::effectorPosCallback,this);

    camera_object_publisher_ = n_.advertise<geometry_msgs::Transform> ("/camera_object", 1000);
    world_effector_publisher_ = n_.advertise<geometry_msgs::Transform> ("/world_effector", 1000);

    compute_effector_camera_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera> ("compute_effector_camera");
    compute_effector_camera_quick_service_
      = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> ("compute_effector_camera_quick");
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

void KukaCalib::pose_camera_object_pub_(geometry_msgs::Transform pose_camera_object_){
    ros::Rate loop_rate(100);
    while(camera_object_publisher_.getNumSubscribers() == 0) {
        loop_rate.sleep();
    }
    camera_object_publisher_.publish(pose_camera_object_);
    emc_quick_comm.request.camera_object.transforms.push_back(pose_camera_object_);
}

void KukaCalib::pose_world_effector_pub_(geometry_msgs::Transform pose_world_effector_){
    ros::Rate loop_rate(100);
    while(world_effector_publisher_.getNumSubscribers() == 0) {
        loop_rate.sleep();
    }
    world_effector_publisher_.publish(pose_world_effector_);
    emc_quick_comm.request.world_effector.transforms.push_back(pose_world_effector_);
}


std_msgs::Int8 KukaCalib::getTrackStatus(){
    return track_status;
}

geometry_msgs::PoseStamped KukaCalib::getObjectPos(){
    return objectPos;
}

geometry_msgs::PoseStamped KukaCalib::geteffectorPos(){
    return effectorPos;
}

void KukaCalib::computeUsingQuickService()
{
  ROS_INFO("2) QUICK SERVICE:");
  if (compute_effector_camera_quick_service_.call(emc_quick_comm))
  {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

void KukaCalib::computeFromTopicStream()
{
  
  ROS_INFO("3) TOPIC STREAM:");
  if (compute_effector_camera_service_.call(emc_comm))
  {
    ROS_INFO_STREAM("hand_camera: " << std::endl << emc_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

}

float GetRandom(float min,float max)
{
    float ans = min + rand()*(max*100-min*100)/RAND_MAX/100.0;

  return ans;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Kuka_calib"); 
  KukaCalib kuka_calib;
  
  actionlib::SimpleActionClient<tnp_kuka_motion::moveToCartPosePTPAction> CartPosePTPAction("tnp_kuka_motion/moveToCartPosePTPAction", true); 
   ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  CartPosePTPAction.waitForServer(); //will wait for infinite time
  
  ROS_INFO("Action servers started, sending goal.");
  // send a goalJA to the action 

  std_msgs::Int8 trackStatus;
  trackStatus.data = 0;
  ros::Rate loop_rate(1);
  while(trackStatus.data == 0) {
      trackStatus = kuka_calib.getTrackStatus();
      loop_rate.sleep();
      ros::spinOnce();
  }

  ROS_INFO("Initialized");

  tnp_kuka_motion::moveToCartPosePTPGoal center; //Please set QR code position
  center.x = 0.600;
  center.y = 0;
  center.z = -0.30;
  center.u = +180   /180*M_PI;
  center.v = 0 /180*M_PI;
  center.w = +180   /180*M_PI;
  
  int pointNum = 30; //size of point for calibration
  int TIMEOUT = 1; //If it is not found after 5 seconds, the next operation is performed
  srand((unsigned int)time(NULL));

  //Set move area
  float maxX = +0.600;
  float minX = +0.400;
  float maxY = +0.300;
  float minY = -0.300;
  float maxZ = +0.600;
  float minZ = +0.500;
  
  geometry_msgs::PoseStamped objectPos;
  geometry_msgs::PoseStamped effectorPos;
  tnp_kuka_motion::moveToCartPosePTPGoal goal;

  int i = 0;
  while(i<pointNum){

        //set nect goal
        goal.x = GetRandom(minX,maxX);
        goal.y = GetRandom(minY,maxY);
        goal.z = GetRandom(minZ,maxZ);
        double alpha = atan2(fabs(goal.y-center.y),fabs(goal.z-center.z));
        double beta = atan2(fabs(goal.x-center.x),fabs(goal.z-center.z));
        if(goal.y > 0){
            goal.u = center.u + alpha;
        }else{
            goal.u = center.u - alpha;
        }

        if(goal.x > 0){
            goal.v = center.v + beta;
        }else{
            goal.v= center.v - beta;
        }
        goal.w = center.w;

        CartPosePTPAction.sendGoal(goal); //send to KUKA
        ROS_INFO("Now, Kuka-chan is traveling with CartPosePTPAction");

        //wait for the action to return
        bool finished_before_timeout = CartPosePTPAction.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = CartPosePTPAction.getState();
            ROS_INFO("CartPosePTPActionAction finished: %s",state.toString().c_str());
        }
        else{
            ROS_INFO("CartPosePTPActionAction did not finish before the time out.");
            continue;
        }

      ros::spinOnce();
        trackStatus = kuka_calib.getTrackStatus();
        int count = 0;
        bool e = false;
        while(trackStatus.data != 3){
            ROS_INFO("Where is QR code?");
            trackStatus = kuka_calib.getTrackStatus();
            loop_rate.sleep();
            ros::spinOnce();
            count++;
            if(count >= TIMEOUT){
                e = true;
                break;
            }
        }

        if(e){
            ROS_INFO("QR code NOT found!!");
            continue; 
        }else{
            ROS_INFO("QR code found!!");
        }
        

        if(trackStatus.data == 3){
          ROS_INFO("No.%d pose send!!", i+1);
            objectPos = kuka_calib.getObjectPos();
            geometry_msgs::Transform pose_c_o;
            pose_c_o.translation.x = objectPos.pose.position.x;
            pose_c_o.translation.y = objectPos.pose.position.y;
            pose_c_o.translation.z = objectPos.pose.position.z;
            pose_c_o.rotation = objectPos.pose.orientation;

            kuka_calib.pose_camera_object_pub_(pose_c_o);

            effectorPos = kuka_calib.geteffectorPos();
            geometry_msgs::Transform pose_world_effector;
            pose_world_effector.translation.x = effectorPos.pose.position.x;
            pose_world_effector.translation.y = effectorPos.pose.position.y;
            pose_world_effector.translation.z = effectorPos.pose.position.z;
            pose_world_effector.rotation = effectorPos.pose.orientation;

            kuka_calib.pose_world_effector_pub_(pose_world_effector);
          ros::spinOnce();
        }else{
            ROS_INFO("Invalid data - can't send pose");
            ros::spinOnce();
        }

        i++;
  }
 
  kuka_calib.computeUsingQuickService();
  
  std::wcout << L"end" << L"\n";

}
