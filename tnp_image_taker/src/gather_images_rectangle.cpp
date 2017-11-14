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
  // send a goalJA to the action 

  tnp_kuka_motion::moveToCartPosePTPGoal center; //Please set object's position
  center.x = 0.550;
  center.y = 0;
  center.z = -0.30;
  center.u = +180   /180*M_PI;
  center.v = 0 /180*M_PI;
  center.w = +180   /180*M_PI;
  
  int xpointNum = 3; //x-direction size of point for gathering
  int ypointNum = 4; //y-direction size of point for gathering
  int zpointNum = 2; //z-direction size of point for gathering
  int pointNum = xpointNum*ypointNum*zpointNum;

  tnp_kuka_motion::moveToCartPosePTPGoal Point[pointNum]; //Please set move area
  tnp_kuka_motion::moveToCartPosePTPGoal goal;
  srand((unsigned int)time(NULL));
  float maxX = +0.550;
  float minX = +0.350;
  float maxY = +0.350;
  float minY = -0.350;
  float maxZ = +0.700;
  float minZ = +0.600;

  	while(ros::ok()){
  	  int znum=1;
	  while(znum < zpointNum+1){ //z=minZ
	  	int xnum=1;
	   	while(xnum < xpointNum+1){
	   		int ynum=1;
	   		while(ynum < ypointNum+1){ 
		   		int num=(xnum-1)*(ypointNum)+ynum-1+(zpointNum-1)*xpointNum*ypointNum;
		   		Point[num].x = minX+0.10*(xnum-1);
		  		Point[num].y = minY+0.233*(ynum-1);
		  		Point[num].z = minZ+0.10*(znum-1);
		  		double alpha = atan2(fabs(Point[num].y-center.y),fabs(Point[num].z-center.z));
		      	double beta = atan2(fabs(Point[num].x-center.x),fabs(Point[num].z-center.z));
		      	if(Point[num].y > 0){
		          	Point[num].u = center.u + alpha;
		      	}else{
		          	Point[num].u = center.u - alpha;
		      	}
		      	if(Point[num].x > 0){
		          	Point[num].v = center.v + beta;
		      	}else{
		          	Point[num].v= center.v - beta;
		      	}
		      	Point[num].w = center.w;
		      	goal = Point[num];
		        std::wcout << goal.u << " " << L"\n";
		        CartPosePTPAction.sendGoal(goal); 
		        bool finished_before_timeout = CartPosePTPAction.waitForResult(ros::Duration(10.0));
		        if(finished_before_timeout){
		        	ynum++;	
		        }
		        ros::Duration(0.1).sleep();
	        }
	        xnum++;
	   	} 
	   	znum++;		
	  }
	}

}
