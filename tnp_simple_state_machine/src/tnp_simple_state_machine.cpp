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
#include "rosgraph_msgs/Log.h"
#include "std_msgs/String.h"
#include <ctype.h>
#include <stdio.h>

// Message structure:
// ROS_WARN("[TNP_STATE L2] STATUS /tnp_simple_statemachine/Service_C Comment");

class SubscribeAndPublish {
public:
  SubscribeAndPublish()
  {
    chatter_pub = n.advertise<std_msgs::String>("tnp_simple_state_machine/status_message", 1000);
    sub = n.subscribe("/rosout_agg", 1000, &SubscribeAndPublish::chatterCallback, this);
  }
  void chatterCallback(const rosgraph_msgs::Log::ConstPtr &msg)
  {
    std_msgs::String msg_publish;
    std::string message = msg->msg.c_str();
    if ((int(msg->level) == 4) && (msg->msg.substr(0, 10) == "[TNP_STATE")) {
      //Check spaces in the string
      int space_location[5] = {};
      int position_space = 0;
      for (int i = 0; i < message.length(); i++) {
        if ((message.substr(i, 1)) == " ") {
          space_location[position_space] = i;
          position_space++;
          if (position_space > 5) {
            break;
          }
        }
      }

      int msg_level = atoi(message.substr(space_location[0] + 2, (space_location[1] - space_location[0] - 2)).c_str());
      int msg_indent = (msg_level)*10;

      std::string status = message.substr((space_location[1] + 1), (space_location[2] - space_location[1]));
      std::string who_called = message.substr((space_location[2] + 1), (space_location[3] - space_location[2]));
      std::string message_comment = message.substr((space_location[3] + 1), (message.length() - space_location[3]));

      printf(" %*s", msg_indent, " ");
      std::cout << who_called << std::endl;
      printf(" %*s", msg_indent, " ");
      std::cout << "Status: " << status << std::endl;
      printf(" %*s", msg_indent, " ");
      std::cout << "Comment: " << message_comment << std::endl;
      std::cout << "=====================================" << std::endl;

      std::stringstream ss;
      ss << status << " " << who_called << " "
         << message_comment;

      msg_publish.data = ss.str();
      chatter_pub.publish(msg_publish);
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher chatter_pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_simple_state_machine");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
