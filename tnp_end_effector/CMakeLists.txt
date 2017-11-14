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
project(tnp_end_effector)

use_cxx11() # since the upper one seems to be uneffective sometimes

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  genmsg
)

add_service_files(
  FILES
  Suction.srv
  SuctionRotation.srv
  LinActuatorSuction.srv
  LinActuatorGripper.srv
  Blower.srv
  Gripper.srv
  Drawer.srv
  OpenShutterCloseTheRest.srv
  EEControl.srv
  ItemSuctioned.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} src
)
link_directories(${catkin_LINK_DIRS})

add_library(tnp_end_effector-lib  src/tnp_end_effector.cpp)
add_dependencies(
  tnp_end_effector-lib
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(tnp_end_effector-lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_ee src/tnp_end_effector.cpp)
add_dependencies(
  ${PROJECT_NAME}_ee
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_ee ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_shutters src/tnp_shutters.cpp)
add_dependencies(
  ${PROJECT_NAME}_shutters
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_shutters ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

  set(TESTPROJ gtest_end_effector)
  find_package(rostest REQUIRED)
  find_package(GTest)

  include_directories( ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} src src/test )

  add_rostest_gtest( ${TESTPROJ}_unit test/end_effector_utest.test src/test/test_utest.cpp)
  target_link_libraries(${TESTPROJ}_unit ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_end_effector-lib )
  add_rostest_gtest( ${TESTPROJ}_node test/end_effector_node.test src/test/test_node.cpp)
  target_link_libraries(${TESTPROJ}_node ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_end_effector-lib)
  add_rostest_gtest( ${TESTPROJ}_API test/end_effector_api.test src/test/test_API.cpp )
  target_link_libraries(${TESTPROJ}_API ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread tnp_end_effector-lib )

endif()# macro definition finding the right compiler flag for c++11
