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
project(tnp_feat_vision)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  realsense_camera
  cv_bridge
  image_transport
  image_geometry
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_service_files(
  FILES
  set_items_info.srv
  set_items_pics.srv
  remove_background.srv
  cloud_matching.srv
  identify_item.srv
  set_rs_background.srv
  get_bounding_box.srv  
  matching_by_size.srv
  matching_by_histogram.srv
  matching_by_descriptor.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs  
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_image_viewer src/tnp_image_viewer.cpp)
add_executable(${PROJECT_NAME}_plane_matching src/tnp_plane_matching.cpp)
add_executable(${PROJECT_NAME}_cloud_matching src/tnp_cloud_matching.cpp src/RGBDFrame.cpp src/BBoxExtractor.cpp src/EndEffectorQRDetector.cpp src/BackgroundSubtraction.cpp src/UtilCvPclRs.cpp src/helpers.cpp)
add_executable(${PROJECT_NAME}_record_rgbd_frames src/tnp_record_RgbdFrames.cpp src/RGBDFrame.cpp src/UtilCvPclRs.cpp src/helpers.cpp)
add_executable(${PROJECT_NAME}_color_histo src/tnp_color_histogram.cpp src/helpers.cpp src/UtilCvPclRs.cpp src/RGBDFrame.cpp src/EndEffectorQRDetector.cpp)

# don't erase these add_dependecies, otherwise you need to build twice 
add_dependencies(${PROJECT_NAME}_plane_matching ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_cloud_matching ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_record_rgbd_frames ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_color_histo ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_image_viewer ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_plane_matching ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cloud_matching ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_record_rgbd_frames ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_color_histo ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})