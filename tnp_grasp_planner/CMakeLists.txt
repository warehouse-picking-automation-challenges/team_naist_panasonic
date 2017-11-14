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

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

cmake_minimum_required(VERSION 2.8.3)
project(tnp_grasp_planner)

add_definitions(-std=c++11)
use_cxx11() # since the upper one seems to be uneffective sometimes

set(FCL_INCLUDE_DIRS "/usr/include/")
set(FCL_LIBRARIES "/usr/lib/x86_64-linux-gnu/libfcl.so")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  realsense_camera
  message_generation
  octomap_ros
  phoxi_camera
  tnp_kuka_motion
)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED)

add_service_files(
  FILES
  getSuctionCandidates_bb.srv
  getSuctionCandidates_container.srv
  getGripCandidates_bb.srv
  getGripCandidates_container.srv
  getContainerOccupancy.srv
	savePCDFile.srv
  getPoseToPlaceItem.srv
  updateOccupancyMap.srv
  getSuctionCandidates_container_fromOctomap.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LINK_DIRS}
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

add_executable(tnp_grasp_planner src/tnp_grasp_planner.cpp)

add_dependencies(tnp_grasp_planner ${tnp_grasp_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tnp_grasp_planner
  ${catkin_LIBRARIES}
	${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${FCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
