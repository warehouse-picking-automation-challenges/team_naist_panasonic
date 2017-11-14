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
#include "tnp_kuka_motion/moveToJointAnglesPTPAction.h"

// This was written to test using the action server in tnp_kuka_motion.

int main (int argc, char **argv)
{
  ros::init(argc, argv, "moveToJointAnglesPTPAction_client"); 

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<tnp_kuka_motion::moveToJointAnglesPTPAction> ac("tnp_kuka_motion/moveToJointAnglesPTPAction", true); 

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action 

  tnp_kuka_motion::moveToJointAnglesPTPGoal atHome, atTote, lookIntoTote, atAmnesty, lookIntoAmnesty, atRecSpace, atBins, atBox1, atBox2, atBox3;
  atHome.a1 = 0 / 180.0 * M_PI;
  atHome.a2 = 32 / 180.0 * M_PI;
  atHome.a3 = 0 / 180.0 * M_PI;
  atHome.a4 = -53 / 180.0 * M_PI;
  atHome.a5 = 0 / 180.0 * M_PI;
  atHome.a6 = 95 / 180.0 * M_PI;
  atHome.a7 = 0 / 180.0 * M_PI;

  atTote.a1 = -90 / 180.0 * M_PI;
  atTote.a2 = +17 / 180.0 * M_PI;
  atTote.a3 = 0 / 180.0 * M_PI;
  atTote.a4 = -69 / 180.0 * M_PI;
  atTote.a5 = 0 / 180.0 * M_PI;
  atTote.a6 = +94 / 180.0 * M_PI;
  atTote.a7 = 0 / 180.0 * M_PI;

  lookIntoTote.a1 = -92.5 / 180.0 * M_PI;
  lookIntoTote.a2 = +24 / 180.0 * M_PI;
  lookIntoTote.a3 = 0 / 180.0 * M_PI;
  lookIntoTote.a4 = -80 / 180.0 * M_PI;
  lookIntoTote.a5 = 0 / 180.0 * M_PI;
  lookIntoTote.a6 = +86 / 180.0 * M_PI;
  lookIntoTote.a7 = -2.5 / 180.0 * M_PI;

  atAmnesty.a1 = -145 / 180.0 * M_PI;
  atAmnesty.a2 = 40 / 180.0 * M_PI;
  atAmnesty.a3 = 0 / 180.0 * M_PI;
  atAmnesty.a4 = -42 / 180.0 * M_PI;
  atAmnesty.a5 = 0 / 180.0 * M_PI;
  atAmnesty.a6 = 97 / 180.0 * M_PI;
  atAmnesty.a7 = -55 / 180.0 * M_PI;

  lookIntoAmnesty.a1 = -143 / 180.0 * M_PI;
  lookIntoAmnesty.a2 = 41 / 180.0 * M_PI;
  lookIntoAmnesty.a3 = 0 / 180.0 * M_PI;
  lookIntoAmnesty.a4 = -62 / 180.0 * M_PI;
  lookIntoAmnesty.a5 = 8 / 180.0 * M_PI;
  lookIntoAmnesty.a6 = 87 / 180.0 * M_PI;
  lookIntoAmnesty.a7 = 35 / 180.0 * M_PI;

  atRecSpace.a1 = 0 / 180.0 * M_PI;
  atRecSpace.a2 = 12 / 180.0 * M_PI;
  atRecSpace.a3 = 0 / 180.0 * M_PI;
  atRecSpace.a4 = -64 / 180.0 * M_PI;
  atRecSpace.a5 = 0 / 180.0 * M_PI;
  atRecSpace.a6 = 103 / 180.0 * M_PI;
  atRecSpace.a7 = 0 / 180.0 * M_PI;
  // Cartesian pose: 529, 0, 763, 180, 0, 180 (according to KUKA)


  tnp_kuka_motion::moveToJointAnglesPTPGoal goal;
  int destinationNumber = -1;
  while (destinationNumber != 999){
        std::cout << "kuka-chan destinations" << std::endl;
        std::cout << "0: at Home" << std::endl;
        std::cout << "1: at Tote" << std::endl;
        std::cout << "2: (DO NOT USE)at Amnesty" << std::endl;
        std::cout << "3: at RecSpace" << std::endl;
        std::cout << "4: at Bins" << std::endl;
        std::cout << "5: at Box1" << std::endl;
        std::cout << "6: at Box2" << std::endl;
        std::cout << "7: at Box3" << std::endl;
        std::cout << "999: Program end" << std::endl;
        std::cout << "Please select destination number = " << std::endl;
        std::cin >> destinationNumber;
        switch(destinationNumber){
            case 0:
                goal = atHome;
                break;
            case 1:
                goal = atTote;
                break;
            case 2:
                goal = lookIntoTote;
                break;
            case 3:
                goal = atRecSpace;
                break;
            case 4:
                //goal = atBins;
                break;
            case 5:
                //goal = atBox1;
                break;
            case 6:
                //goal = atBox2;
                break;
            case 7:
                //goal = atBox3;
                break;
            case 999:
                ROS_INFO("Good bye!!");
                return 0;
            default:
                std::cout << "Invalid Number!!" << std::endl;
                continue;
        }
        ac.sendGoal(goal);
        ROS_INFO("Now, Kuka-chan is traveling");

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else  
            ROS_INFO("Action did not finish before the time out.");

  }
  

  //exit
  return 0;
}