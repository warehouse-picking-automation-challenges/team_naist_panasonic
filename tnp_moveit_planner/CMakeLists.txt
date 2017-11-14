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
project(tnp_moveit_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  tf
  tf_conversions
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

add_service_files(
  DIRECTORY srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(command_robot_moveit src/command_robot_moveit.cpp)
add_dependencies(command_robot_moveit 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(command_robot_moveit ${catkin_LIBRARIES})

add_executable(one_move src/one_move.cpp)
add_dependencies(one_move
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(one_move ${catkin_LIBRARIES})

add_executable(tnp_moveit_planner src/tnp_moveit_planner.cpp)
add_dependencies(tnp_moveit_planner
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS}	
)
target_link_libraries(tnp_moveit_planner ${catkin_LIBRARIES})

add_executable(tnp_cartesian_move src/tnp_cartesian_move.cpp)
add_dependencies(tnp_cartesian_move 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(tnp_cartesian_move ${catkin_LIBRARIES})
