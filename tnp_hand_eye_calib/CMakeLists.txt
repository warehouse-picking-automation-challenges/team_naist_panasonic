#
# Version:  2017.07.31
# Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
#           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
#           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
#           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
#           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
#           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
#####################
# Copyright 2017 Team NAIST-Panasonic 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at 
#     http://www.apache.org/licenses/LICENSE-2.0 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#####################

cmake_minimum_required(VERSION 2.8.3)
project(tnp_hand_eye_calib)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  iiwa_ros
  message_generation
  std_msgs
  geometry_msgs
  iiwa_msgs
  tf
  genmsg			## These three are for the actionlib
  actionlib_msgs 
  actionlib
  tnp_kuka_motion
  tnp_moveit_planner
)

add_service_files(
  FILES
  calibrateCamera.srv
  checkCalibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  iiwa_msgs
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs iiwa_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calib_cartPose src/calib_cartPose.cpp)
target_link_libraries(calib_cartPose ${catkin_LIBRARIES})
add_dependencies(calib_cartPose 
	${tnp_hand_eye_calib_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS} )
