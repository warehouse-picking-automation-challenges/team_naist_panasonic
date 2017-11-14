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

#include "tnp_shutters.h"

ShuttersNode::ShuttersNode()
{
  //Topic you want to publish
  pub_arduino_write_ = n_.advertise<std_msgs::String>("tnp_arduino_shutters/write", 100);

  //Topic you want to subscribe
  sub_arduino_read_ = n_.subscribe("tnp_arduino_shutters/read", 1, &ShuttersNode::ArduinoReadCallback, this);

  // services you want to advertise
  open_shutter_close_the_rest_service_ = n_.advertiseService("tnp_shutters/open_shutter_close_the_rest", &ShuttersNode::OpenShutterCloseTheRestCallback, this);
}

//Helpers

/* The readme.md of the tnp_end_effector has the commands details*/
bool ShuttersNode::SendCommandToArduino(const std::string& arduinoCommand)
{
  ROS_DEBUG("Sending command to Arduino (shutters). Wait until the connection is completed...");
  ros::Rate loop_rate(100);
  while (pub_arduino_write_.getNumSubscribers() == 0)
  {
    loop_rate.sleep();
    ros::spinOnce(); //testing if this make the wait until connection not to get stuck
  }

  std_msgs::String msg;
  msg.data = arduinoCommand;

  // once connection is ok, send the command
  if (ros::ok())
  {
    pub_arduino_write_.publish(msg);
    ros::spinOnce();
  }
  else
  {
    ROS_INFO("ros not ok. Tried sending: %s", msg.data.c_str());
    return false;
  }

  ROS_DEBUG("Sent: %s", msg.data.c_str());
  return true;
}

// Topics callbacks
void ShuttersNode::ArduinoReadCallback(const std_msgs::StringConstPtr& input)
{
  //ROS_ERROR("ArduinoReadCallback NOT IMPLEMENTED YET");
}

// Service definitions

bool ShuttersNode::OpenShutterCloseTheRestCallback(tnp_end_effector::OpenShutterCloseTheRest::Request &req, tnp_end_effector::OpenShutterCloseTheRest::Response &res)
{
  ROS_DEBUG("OpenShutterCloseTheRestCallback was called");

  // 5 shutters, 5 states. 1 open, the rest closed
  // bit 0 is E (end effector)
  // bit 1 is L (left)
  // bit 2 is R (right)
  // bit 3 is C (center)
  // bit 4 is B (base)
  // bit 5 is T
  // bit 6 is U

  //0: open, 1:close
  if(req.camera_id.data.compare("E") == 0 || req.camera_id.data.compare("e") == 0)
  {
    SendCommandToArduino("set 0 111");
  }
  else if(req.camera_id.data.compare("L") == 0 || req.camera_id.data.compare("l") == 0)
  {
    SendCommandToArduino("set 0 119");
  }
  else if(req.camera_id.data.compare("R") == 0 || req.camera_id.data.compare("r") == 0)
  {
    SendCommandToArduino("set 0 126");
  }
  else if(req.camera_id.data.compare("C") == 0 || req.camera_id.data.compare("c") == 0)
  {
    SendCommandToArduino("set 0 123");
  }
  else if(req.camera_id.data.compare("B") == 0 || req.camera_id.data.compare("b") == 0)
  {
    SendCommandToArduino("set 0 125");
  }
  else if(req.camera_id.data.compare("T") == 0 || req.camera_id.data.compare("t") == 0)
  {
    SendCommandToArduino("set 0 95");
  }
  else if(req.camera_id.data.compare("U") == 0 || req.camera_id.data.compare("u") == 0)
  {
    SendCommandToArduino("set 0 63");
  }
  else
  {
    ROS_ERROR_STREAM("Unexpected camera id: " << req.camera_id.data);
  }

  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_shutters");

  ShuttersNode SAPObject;

  ROS_INFO("Shutters node started");

  ros::spin();

  return 0;
}
