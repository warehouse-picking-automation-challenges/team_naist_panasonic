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

#ifndef TNP_SHUTTERS_H
#define TNP_SHUTTERS_H

#include "tnp_end_effector/OpenShutterCloseTheRest.h"

#include "ros/ros.h"

/**
 * ShuttersNode, controls the mechanical shutters
 */
class ShuttersNode
{
public:
  //Constructor
  ShuttersNode();
  bool setupROSConfiguration();

  //Helpers
  bool SendCommandToArduino(const std::string& arduinoCommand);

  // topics callbacks
  void ArduinoReadCallback(const std_msgs::StringConstPtr& input);

  // Service declarations
  bool OpenShutterCloseTheRestCallback(tnp_end_effector::OpenShutterCloseTheRest::Request &req, tnp_end_effector::OpenShutterCloseTheRest::Response &res);
private:
  // node handle
  ros::NodeHandle n_;

  //publishers
  ros::Publisher pub_arduino_write_;

  //subscribers
  ros::Subscriber sub_arduino_read_;

  // services provided
  ros::ServiceServer open_shutter_close_the_rest_service_;
};
//End of class SubscribeAndPublish

#endif
