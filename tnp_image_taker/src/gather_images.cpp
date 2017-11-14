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
#include "std_msgs/Int8.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "gather_images"); 
    
    actionlib::SimpleActionClient<tnp_kuka_motion::moveToCartPosePTPAction> CartPosePTPAction("tnp_kuka_motion/moveToCartPosePTPAction", true); 
     ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    CartPosePTPAction.waitForServer(); //will wait for infinite time
    
    ROS_INFO("Action servers started, sending goal.");
    // send a goalCartesianPose to the action 
  
    tnp_kuka_motion::moveToCartPosePTPGoal goal;
    srand((unsigned int)time(NULL));
    //set camera position
    float X = -0.0233;
    float Y = -0.532;
    float Z_first = +0.510;
    float Z_high = +0.500;
    float Z_middle = +0.450;
    float Z_low = +0.400;
    double roll = 90.75 / 180.0 * M_PI;
    double pitch = -10.01 / 180.0 * M_PI;
    double yaw = 179.44 / 180.0 * M_PI;

    goal.x = X;
	goal.y = Y;
	goal.z = Z_first;
	goal.u = roll;
	goal.v = pitch;
	goal.w = yaw;
	CartPosePTPAction.sendGoal(goal);
	ros::Duration(3).sleep();

  	while(ros::ok()){
  	//high_position;
		goal.x = X;
		goal.y = Y;
		goal.z = Z_high;
	    goal.u = roll;
	    goal.v = pitch;
	    goal.w = yaw;
	    CartPosePTPAction.sendGoal(goal);
	    ros::Duration(1).sleep();
	//middle_position;
		goal.x = X;
		goal.y = Y;
		goal.z = Z_middle;
	    goal.u = roll;
	    goal.v = pitch;
	    goal.w = yaw;
	    CartPosePTPAction.sendGoal(goal);
	    ros::Duration(1).sleep();
	//low_position;
		goal.x = X;
		goal.y = Y;
		goal.z = Z_low;
	    goal.u = roll;
	    goal.v = pitch;
	    goal.w = yaw;
	    CartPosePTPAction.sendGoal(goal);
	    ros::Duration(1).sleep(); 
	}
}
