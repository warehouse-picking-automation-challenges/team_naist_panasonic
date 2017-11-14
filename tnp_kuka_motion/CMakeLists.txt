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
project(tnp_kuka_motion)

add_definitions(-std=c++11)
use_cxx11() # since the upper one seems to be uneffective sometimes

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
  tnp_end_effector
  tnp_moveit_planner
  tnp_msgs
)

add_service_files(
  DIRECTORY srv
)

add_action_files(
  DIRECTORY action    ## This line is from the tutorial but was not in the original CMakeLists example
  FILES
  moveToJointAnglesPTP.action
  moveToCartPosePTP.action
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
  src
  ${catkin_INCLUDE_DIRS} 
)
link_directories(${catkin_LINK_DIRS})

add_library(tnp_kuka_controller-lib src/tnp_kuka_motion.cpp)
add_dependencies(
  tnp_kuka_controller-lib
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(tnp_kuka_controller-lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/tnp_kuka_motion.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )

MESSAGE(STATUS "~~~~ ${catkin_LIBRARIES}")

add_executable(tnp_kuka_motion_client src/tnp_kuka_motion_client.cpp)
target_link_libraries(tnp_kuka_motion_client ${catkin_LIBRARIES})
add_dependencies(
  tnp_kuka_motion_client 
  tnp_kuka_motion_generate_messages_cpp 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )

add_executable(tnp_kuka_cartPose_motion src/tnp_kuka_cartPose_motion.cpp)
target_link_libraries(tnp_kuka_cartPose_motion ${catkin_LIBRARIES})
add_dependencies(tnp_kuka_cartPose_motion  
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)

  set(TESTPROJ gtest_kuka_motion)
  find_package(rostest REQUIRED)
  find_package(GTest)
  
  include_directories( ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} src src/test )

  add_rostest_gtest( ${TESTPROJ}_unit test/kuka_motion_utest.test src/test/test_utest.cpp)
  target_link_libraries(${TESTPROJ}_unit ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_kuka_controller-lib) 
  add_rostest_gtest( ${TESTPROJ}_node test/kuka_motion_node.test src/test/test_node.cpp)
  target_link_libraries(${TESTPROJ}_node ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_kuka_controller-lib)
  add_rostest_gtest( ${TESTPROJ}_API test/kuka_motion_api.test src/test/test_API.cpp )
  target_link_libraries(${TESTPROJ}_API ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_kuka_controller-lib)

endif()
