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
project(tnp_task_manager)

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

add_definitions(-std=c++11)
use_cxx11() # since the upper one seems to be uneffective sometimes

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  rostest
  message_generation
  image_geometry
  sensor_msgs

  tnp_kuka_motion
  tnp_end_effector
  tnp_weight_events
  tnp_recognition_space
  tnp_feat_vision
  tnp_deep_vision
  tnp_grasp_planner
  tf
  tnp_led_control
  tnp_msgs
  tnp_moveit_planner
  tnp_svm

  genmsg      ## These three are for the actionlib
  actionlib_msgs
  actionlib
)

add_service_files(
  FILES
  setupBeforeRound.srv
  initialSetup.srv
  stowTask.srv
  pickTask.srv
  testMotions.srv
  transform2poses.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  tnp_task_manager
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
)

file( GLOB TASK_MANGR_SRCS src/*.h src/*.cpp )
MESSAGE(STATUS "~~~~ All files in tnp_task_manager module ${TASK_MANGR_SRCS}")

include_directories(
  ${catkin_INCLUDE_DIRS} include src src/test
)
link_directories(${catkin_LINK_DIRS})

add_library(tnp_task_manager-lib  ${TASK_MANGR_SRCS})
target_link_libraries(
  tnp_task_manager-lib
  ${catkin_LIBRARIES}
)
add_dependencies(
  tnp_task_manager-lib
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(${PROJECT_NAME} ${TASK_MANGR_SRCS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

if (CATKIN_ENABLE_TESTING)

  set(TESTPROJ gtest_task_manager)
  find_package(rostest REQUIRED)
  find_package(GTest)

  include_directories( ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} src src/test )

  add_rostest_gtest( ${TESTPROJ}_unit test/task_manager_utest.test src/test/test_utest.cpp)
  target_link_libraries(${TESTPROJ}_unit ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_task_manager-lib )
  add_rostest_gtest( ${TESTPROJ}_node test/task_manager_node.test src/test/test_node.cpp)
  target_link_libraries(${TESTPROJ}_node ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_task_manager-lib)
  add_rostest_gtest( ${TESTPROJ}_API test/task_manager_api.test src/test/test_API.cpp )
  target_link_libraries(${TESTPROJ}_API ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_task_manager-lib )

endif()
