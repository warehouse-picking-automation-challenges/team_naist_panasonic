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
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"

#include <tf/transform_listener.h>    // Includes the TF conversions

#include "iiwa_ros.h"           // To call time_to_destination
#include <chrono>
#include <thread>               // To wait for time_to_destination

// Services
#include "tnp_kuka_motion/goToHome.h"
#include "tnp_kuka_motion/goToTote.h"
#include "tnp_kuka_motion/goToAmnesty.h"
#include "tnp_kuka_motion/goToRecSpace.h"
#include "tnp_kuka_motion/goToBinNum.h"
#include "tnp_kuka_motion/goToBoxNum.h"
#include "tnp_kuka_motion/moveToGraspPose.h"
#include "tnp_kuka_motion/moveToJointAnglesPTP.h"
#include "tnp_kuka_motion/moveToCartPosePTP.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "tnp_kuka_motion/moveToCartPosePTPAction.h"


class SubscribeAndPublish
{
public:
  //Constructor
  SubscribeAndPublish();

  bool initializePositions();   // Populates joint angles/positions we use. Called during startup.

  //Helpers
  bool waitUntilArrived();      // Returns true if KUKA has arrived at destination
  bool moveToJointAnglesPTP(iiwa_msgs::JointPosition jp);
  bool moveToJointAnglesPTP(const double& j1, const double& j2, 
    const double& j3, const double& j4, const double& j5, const double& j6, const double& j7);
  bool moveToCartPosePTP(const double& x, const double& y, const double& z, 
  const double& u, const double& v, const double& w); // uvw = roll,pitch,yaw
  bool moveToCartPosePTP(geometry_msgs::Pose pose);
  bool stop();                  // Stops the robot at the current position

  // Callback declarations
  bool goToHomeCallback(tnp_kuka_motion::goToHome::Request &req,
                        tnp_kuka_motion::goToHome::Response &res);
  bool goToToteCallback(tnp_kuka_motion::goToTote::Request &req,
                        tnp_kuka_motion::goToTote::Response &res);
  bool goToAmnestyCallback(tnp_kuka_motion::goToAmnesty::Request &req,
                           tnp_kuka_motion::goToAmnesty::Response &res);
  bool goToRecSpaceCallback(tnp_kuka_motion::goToRecSpace::Request &req,
                            tnp_kuka_motion::goToRecSpace::Response &res);
  bool goToBinNumCallback(tnp_kuka_motion::goToBinNum::Request &req,
                          tnp_kuka_motion::goToBinNum::Response &res);
  bool goToBoxNumCallback(tnp_kuka_motion::goToBoxNum::Request &req,
                          tnp_kuka_motion::goToBoxNum::Response &res);
  bool moveToGraspPoseCallback(tnp_kuka_motion::moveToGraspPose::Request &req,
                               tnp_kuka_motion::moveToGraspPose::Response &res);

  bool moveToJointAnglesPTPCallback(tnp_kuka_motion::moveToJointAnglesPTP::Request &req,
                                    tnp_kuka_motion::moveToJointAnglesPTP::Response &res);
  bool moveToCartPosePTPCallback(tnp_kuka_motion::moveToCartPosePTP::Request &req,
                                           tnp_kuka_motion::moveToCartPosePTP::Response &res);

  // Actions
  void execute(const tnp_kuka_motion::moveToCartPosePTPGoalConstPtr& goal);

private:
  ros::NodeHandle n_;
  ros::Publisher pubJointPos_;
  ros::Publisher pubCartPose_;

  // Service declarations
  ros::ServiceServer goToHomeService;
  ros::ServiceServer goToToteService;
  ros::ServiceServer goToAmnestyService;
  ros::ServiceServer goToRecSpaceService;
  ros::ServiceServer goToBinNumService;
  ros::ServiceServer goToBoxNumService;
  ros::ServiceServer moveToGraspPoseService;
  ros::ServiceServer moveToJointAnglesPTPService;
  ros::ServiceServer moveToCartPosePTPService;

  // Joint angles and positions we use
  iiwa_msgs::JointPosition atHome, atTote, atAmnesty, atRecSpace, atBins, atBox1, atBox2, atBox3;

  // Action declarations
  actionlib::SimpleActionServer<tnp_kuka_motion::moveToCartPosePTPAction> 
    moveToCartPosePTPActionServer_;

  iiwa_ros::iiwaRos my_iiwa_ros_object;      // for time_to_destination
  
  int a;
 
};//End of class SubscribeAndPublish


SubscribeAndPublish::SubscribeAndPublish() : moveToCartPosePTPActionServer_(n_, 
    "tnp_kuka_motion/moveToCartPosePTPAction", boost::bind(&SubscribeAndPublish::execute, this, _1),false)
{ 
  initializePositions();
  //Topic you want to publish
  pubJointPos_ = n_.advertise<iiwa_msgs::JointPosition>("iiwa/command/JointPosition", 1000);
  pubCartPose_ = n_.advertise<geometry_msgs::PoseStamped>("iiwa/command/CartesianPose", 1000);

  // services you want to advertise
  goToHomeService = n_.advertiseService("tnp_kuka_motion/goToHome", &SubscribeAndPublish::goToHomeCallback,
                                        this);
  goToToteService = n_.advertiseService("tnp_kuka_motion/goToTote", &SubscribeAndPublish::goToToteCallback,
                                        this);
  goToAmnestyService = n_.advertiseService("tnp_kuka_motion/goToAmnesty", &SubscribeAndPublish::goToAmnestyCallback,
                                        this);
  goToRecSpaceService = n_.advertiseService("tnp_kuka_motion/goToRecSpace", &SubscribeAndPublish::goToRecSpaceCallback,
                                        this);
  goToBinNumService = n_.advertiseService("tnp_kuka_motion/goToBinNum", &SubscribeAndPublish::goToBinNumCallback,
                                        this);
  goToBoxNumService = n_.advertiseService("tnp_kuka_motion/goToBoxNum", &SubscribeAndPublish::goToBoxNumCallback,
                                        this);
  moveToGraspPoseService = n_.advertiseService("tnp_kuka_motion/moveToGraspPose", 
                                                    &SubscribeAndPublish::moveToGraspPoseCallback,
                                                    this);
  moveToJointAnglesPTPService = n_.advertiseService("tnp_kuka_motion/moveToJointAnglesPTP", 
                                                    &SubscribeAndPublish::moveToJointAnglesPTPCallback,
                                                    this);
  moveToCartPosePTPService = n_.advertiseService("tnp_kuka_motion/moveToCartPosePTP", 
                                                    &SubscribeAndPublish::moveToCartPosePTPCallback,
                                                    this);

  // Actions we serve
  moveToCartPosePTPActionServer_.start();

  // Initialize the helper class for time_to_destination and torque data
  my_iiwa_ros_object.init();
}


bool SubscribeAndPublish::waitUntilArrived()
{
    ROS_INFO("Waiting for robot to reach its destination");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));         // Otherwise the robot may not have started the movement yet.

    double time = 5.;
    double force = 0.0;
    double forcelimit = 30.0;   // Newtons. This has to be connected to the outside of this function somehow.
    forcelimit = forcelimit*forcelimit;   // Squared once here so we don't have to do sqrt in the loop
    geometry_msgs::WrenchStamped ws;
    ros::Rate loop_rate(100);
    while (time > 0.){
        time = my_iiwa_ros_object.getTimeToDestinationService().getTimeToDestination();
        if (time > 0.) {
            //ROS_INFO_STREAM(time << " seconds till the robot reaches its destination");
        }
        else  if (time == -999){
            ROS_INFO_STREAM("Something went wrong!");
            stop();
            return false;
        }
        else {
            ROS_INFO_STREAM(time << " seconds since the robot reached its destination");
        }

        // Check for overload
        ros::spinOnce();      // This is required for iiwa_ros to get updated force data
        my_iiwa_ros_object.getCartesianWrench(ws);
        force = ws.wrench.force.x*ws.wrench.force.x + ws.wrench.force.y*ws.wrench.force.y + ws.wrench.force.z*ws.wrench.force.z;
        if (force > forcelimit) {
          ROS_INFO_STREAM("Robot encountered too much force!!! Force seen (squared): " << force << " N^2");
          stop();
          return false;
        }

        loop_rate.sleep();
    }
    return true;
}

bool SubscribeAndPublish::moveToJointAnglesPTP(iiwa_msgs::JointPosition jp)
{
  //wait until the connection is completed
  ROS_INFO("Waiting until the connection is completed");
  ros::Rate loop_rate(100);
  while(pubJointPos_.getNumSubscribers() == 0) {
    loop_rate.sleep();
  }

  // once connection is ok, send the command
  ROS_INFO("Publishing joint angle command");
  if(ros::ok()){
    pubJointPos_.publish(jp);
    ros::spinOnce();
  }
  else {
    ROS_INFO("ros not ok");
    return false;
  }
  waitUntilArrived();
  ROS_INFO("done");

  return true;
}

bool SubscribeAndPublish::moveToJointAnglesPTP(const double& j1, const double& j2, 
    const double& j3, const double& j4, const double& j5, const double& j6, const double& j7)
{
  iiwa_msgs::JointPosition my_joint_position;
  my_joint_position.position.a1 = j1;
  my_joint_position.position.a2 = j2;
  my_joint_position.position.a3 = j3;
  my_joint_position.position.a4 = j4;
  my_joint_position.position.a5 = j5;
  my_joint_position.position.a6 = j6;
  my_joint_position.position.a7 = j7;
  bool out = moveToJointAnglesPTP(my_joint_position);
  return true;
}


// Publishes a cartesian pose for the robot to move to.
bool SubscribeAndPublish::moveToCartPosePTP(const double& x, const double& y, const double& z, 
  const double& u, const double& v, const double& w)
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

bool SubscribeAndPublish::moveToCartPosePTP(geometry_msgs::Pose pose)
{
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time();
  ps.pose = pose;

  //wait until the connection is completed
  ROS_INFO("wait until the connection is completed");
  ros::Rate loop_rate(100);
  while(pubCartPose_.getNumSubscribers() == 0) {
    loop_rate.sleep();
  }

  // once connection is ok, send the command
  ROS_INFO("publishing single command");
  if(ros::ok()){
    pubCartPose_.publish(ps);
    ros::spinOnce();
  }
  else {
    ROS_INFO("ros not ok");
    return false;
  }
  waitUntilArrived();
  ROS_INFO("done");

  return true;
}

bool SubscribeAndPublish::stop()
{
  ROS_INFO("Trying to stop the robot.");
  iiwa_msgs::JointPosition jp;
  my_iiwa_ros_object.getJointPosition(jp);
  if(ros::ok()){
    pubJointPos_.publish(jp);
    ros::spinOnce();
  }
  else {
    ROS_INFO("ros not ok");
    return false;
  }
  ROS_INFO("Robot should have stopped.");
  return true;
}


// Service definitions

bool SubscribeAndPublish::moveToJointAnglesPTPCallback(tnp_kuka_motion::moveToJointAnglesPTP::Request &req,
                                           tnp_kuka_motion::moveToJointAnglesPTP::Response &res)
{
  ROS_INFO("moveToJointAnglesPTPCallback was called");

  bool response = moveToJointAnglesPTP(req.a1, req.a2, req.a3, req.a4, req.a5, req.a6, req.a7);
  res.motionComplete = response;
  return response;
}

bool SubscribeAndPublish::moveToCartPosePTPCallback(tnp_kuka_motion::moveToCartPosePTP::Request &req,
                                           tnp_kuka_motion::moveToCartPosePTP::Response &res)
{
  ROS_INFO("moveToJointAnglesPTPCallback was called");

  bool response = moveToCartPosePTP(req.target_pose);
  res.motionComplete = response;
  return response;
}


bool SubscribeAndPublish::goToHomeCallback(tnp_kuka_motion::goToHome::Request &req,
                                           tnp_kuka_motion::goToHome::Response &res)
{
  ROS_INFO("goToHomeCallback was called");
  bool response = moveToJointAnglesPTP(atHome);
  return response;
}


bool SubscribeAndPublish::goToToteCallback(tnp_kuka_motion::goToTote::Request &req,
                      tnp_kuka_motion::goToTote::Response &res)
{
  ROS_INFO("goToToteCallback was called");
  bool response = moveToJointAnglesPTP(atTote);
  return response;
}

bool SubscribeAndPublish::goToAmnestyCallback(tnp_kuka_motion::goToAmnesty::Request &req,
                              tnp_kuka_motion::goToAmnesty::Response &res)
{
  ROS_INFO("goToAmnestyCallback was called");
  return true;
}

bool SubscribeAndPublish::goToRecSpaceCallback(tnp_kuka_motion::goToRecSpace::Request &req,
                              tnp_kuka_motion::goToRecSpace::Response &res)
{
  ROS_INFO("goToRecSpaceCallback was called");
  bool response = moveToJointAnglesPTP(atRecSpace);
  return response;
}

bool SubscribeAndPublish::goToBinNumCallback(tnp_kuka_motion::goToBinNum::Request &req,
                              tnp_kuka_motion::goToBinNum::Response &res)
{
  ROS_INFO("goToBinNumCallback was called");
  bool response = moveToJointAnglesPTP(atBins);
  return response;
}

bool SubscribeAndPublish::goToBoxNumCallback(tnp_kuka_motion::goToBoxNum::Request &req,
                              tnp_kuka_motion::goToBoxNum::Response &res)
{
  ROS_INFO("goToBoxNumCallback was called");
  ROS_INFO("Going to box number %d", req.box_id);
  bool response;
  if (req.box_id == 1){
    response = moveToJointAnglesPTP(atBox1);
  }
  else if (req.box_id == 2){
    response = moveToJointAnglesPTP(atBox2);
  }
  else if (req.box_id == 3){
    response = moveToJointAnglesPTP(atBox3);
  }
  
  return response;
}

bool SubscribeAndPublish::moveToGraspPoseCallback(tnp_kuka_motion::moveToGraspPose::Request &req,
                              tnp_kuka_motion::moveToGraspPose::Response &res)
{
  ROS_INFO("moveToGraspPoseCallback was called");
  return true;
}

void SubscribeAndPublish::execute(const tnp_kuka_motion::moveToCartPosePTPGoalConstPtr& goal)
{
  ROS_INFO("moveToCartPosePTPAction was called");
  moveToCartPosePTP(goal->x, goal->y, goal->z, goal->u, goal->v, goal->w);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("moveToCartPosePTPAction is set as succeeded");
  moveToCartPosePTPActionServer_.setSucceeded();
}

// This is only called once to define the big important positions
bool SubscribeAndPublish::initializePositions(){
  atHome.position.a1 = 0   /180.0*M_PI;
  atHome.position.a2 = -43 /180.0*M_PI;
  atHome.position.a3 = 0   /180.0*M_PI;
  atHome.position.a4 = -103/180.0*M_PI;
  atHome.position.a5 = 0   /180.0*M_PI;
  atHome.position.a6 = +30 /180.0*M_PI;
  atHome.position.a7 = 0   /180.0*M_PI;

  atTote.position.a1 = -90 /180.0*M_PI;
  atTote.position.a2 = +23 /180.0*M_PI;
  atTote.position.a3 = 0   /180.0*M_PI;
  atTote.position.a4 = -42 /180.0*M_PI;
  atTote.position.a5 = 0   /180.0*M_PI;
  atTote.position.a6 = +115/180.0*M_PI;
  atTote.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: 0, 659, 582, 90, 0, 180 (according to KUKA)

  // NOT USED YET
  atAmnesty.position.a1 = 0    /180.0*M_PI;   
  atAmnesty.position.a2 = 0    /180.0*M_PI;
  atAmnesty.position.a3 = 0    /180.0*M_PI;
  atAmnesty.position.a4 = 0    /180.0*M_PI;
  atAmnesty.position.a5 = 0    /180.0*M_PI;
  atAmnesty.position.a6 = 0    /180.0*M_PI;
  atAmnesty.position.a7 = 0    /180.0*M_PI;
  // Cartesian pose:

  // IDENTICAL TO atBins AT THE MOMENT
  atRecSpace.position.a1 = 0   /180.0*M_PI;
  atRecSpace.position.a2 = 12  /180.0*M_PI;
  atRecSpace.position.a3 = 0   /180.0*M_PI;
  atRecSpace.position.a4 = -64 /180.0*M_PI;
  atRecSpace.position.a5 = 0   /180.0*M_PI;
  atRecSpace.position.a6 = 103 /180.0*M_PI;
  atRecSpace.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: 529, 0, 763, 180, 0, 180 (according to KUKA)

  atBins.position.a1 = 0   /180.0*M_PI;
  atBins.position.a2 = 32  /180.0*M_PI;
  atBins.position.a3 = 0   /180.0*M_PI;
  atBins.position.a4 = -53 /180.0*M_PI;
  atBins.position.a5 = 0   /180.0*M_PI;
  atBins.position.a6 = 95  /180.0*M_PI;
  atBins.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: 623, 0, 596, -179, 0, 180 (according to KUKA)

  atBox1.position.a1 = 50  /180.0*M_PI;
  atBox1.position.a2 = 30  /180.0*M_PI;
  atBox1.position.a3 = 0   /180.0*M_PI;
  atBox1.position.a4 = -41 /180.0*M_PI;
  atBox1.position.a5 = 0   /180.0*M_PI;
  atBox1.position.a6 = +108/180.0*M_PI;
  atBox1.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: 376, 453, 699, -130, 0, 180  (according to KUKA)

  atBox2.position.a1 = +93 /180.0*M_PI;
  atBox2.position.a2 = 10  /180.0*M_PI;
  atBox2.position.a3 = 0   /180.0*M_PI;
  atBox2.position.a4 = -68 /180.0*M_PI;
  atBox2.position.a5 = 0   /180.0*M_PI;
  atBox2.position.a6 = 100 /180.0*M_PI;
  atBox2.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: -28, 468, 699, -87.5, 0, 180 (according to KUKA)

  atBox3.position.a1 = 139 /180.0*M_PI;
  atBox3.position.a2 = 37  /180.0*M_PI;
  atBox3.position.a3 = 0   /180.0*M_PI;
  atBox3.position.a4 = -31 /180.0*M_PI;
  atBox3.position.a5 = 0   /180.0*M_PI;
  atBox3.position.a6 = 111 /180.0*M_PI;
  atBox3.position.a7 = 0   /180.0*M_PI;
  // Cartesian pose: -470, 410, 693, -41, 0, 180 (according to KUKA)
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_kuka_cartPose_motion");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ROS_INFO("KUKA motion node started");

  ros::spin();

  return 0;
}
