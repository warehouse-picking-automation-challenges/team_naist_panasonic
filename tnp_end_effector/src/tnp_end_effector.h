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

#ifndef TNP_END_EFFECTOR_H
#define TNP_END_EFFECTOR_H

#include "tnp_end_effector/Blower.h"
#include "tnp_end_effector/LinActuatorGripper.h"
#include "tnp_end_effector/LinActuatorSuction.h"
#include "tnp_end_effector/Gripper.h"
#include "tnp_end_effector/Suction.h"
#include "tnp_end_effector/SuctionRotation.h"
#include "tnp_end_effector/Drawer.h"
#include "tnp_end_effector/EEControl.h"
#include "tnp_end_effector/ItemSuctioned.h"

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <sstream>


/**
 * EndEffectorNode, controls the end effector processes
 */
class EndEffectorNode
{
public:
  //Constructor
  EndEffectorNode();
  bool setupROSConfiguration();

  //Helpers
  bool SendCommandToArduino(const std::string& arduinoCommand);

  // topics callbacks
  void ArduinoReadCallback(const std_msgs::StringConstPtr& input);

  // Service declarations
  bool SuctionCallback(tnp_end_effector::Suction::Request &req, tnp_end_effector::Suction::Response &res);
  bool SuctionRotationCallback(tnp_end_effector::SuctionRotation::Request &req, tnp_end_effector::SuctionRotation::Response &res);
  bool LinActuatorSuctionCallback(tnp_end_effector::LinActuatorSuction::Request &req,
                                  tnp_end_effector::LinActuatorSuction::Response &res);
  bool GripperCallback(tnp_end_effector::Gripper::Request &req, tnp_end_effector::Gripper::Response &res);
  bool LinActuatorGripperCallback(tnp_end_effector::LinActuatorGripper::Request &req,
                                  tnp_end_effector::LinActuatorGripper::Response &res);
  bool BlowerCallback(tnp_end_effector::Blower::Request &req, tnp_end_effector::Blower::Response &res);
  bool DrawerCallback(tnp_end_effector::Drawer::Request &req, tnp_end_effector::Drawer::Response &res);
  bool EEControlCallback(tnp_end_effector::EEControl::Request &req, tnp_end_effector::EEControl::Response &res);
  bool SuctionStatusCallback(tnp_end_effector::ItemSuctioned::Request &req, tnp_end_effector::ItemSuctioned::Response &res);

private:
  ros::NodeHandle n_;
  ros::Publisher pub_arduino_write_;
  ros::Publisher pub_item_is_suctioned_;
  ros::Publisher pub_item_is_gripped_;
  ros::Publisher pub_suction_is_contact_;
  ros::Publisher pub_linear_actuator_gripper_;
  ros::Publisher pub_linear_actuator_suction_;
  ros::Publisher pub_gripper_open_close_ ;
  ros::Publisher pub_gripper_is_contact_ ;
  ros::Publisher pub_drawer_status_;
  ros::Publisher pub_suction_on_off_ ;
  ros::Publisher pub_blower_;

  ros::Subscriber sub_arduino_read_;

  ros::ServiceServer SuctionService;
  ros::ServiceServer LinActuatorSuctionService;
  ros::ServiceServer GripperService;
  ros::ServiceServer LinActuatorGripperService;
  ros::ServiceServer BlowerService;
  ros::ServiceServer DrawerService;
  ros::ServiceServer EEControlService;
  ros::ServiceServer SuctionStatusService;

  std::string old_eestatus = "xxxxxxxxxxxxxxxxxxxx";
  std::string eecommand = "set 1000010 0110 40";
  bool suction_status = false;
};
//End of class SubscribeAndPublish

#endif
