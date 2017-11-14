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

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(RecognitionSpaceCalibration)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization surface)
find_package( OpenCV REQUIRED )
find_package(Ceres QUIET)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

include_directories(${PCL_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} /opt/ros/kinetic/include)
link_directories(${PCL_LIBRARY_DIRS} /opt/ros/kinetic/lib)
add_definitions(${PCL_DEFINITIONS})

add_executable (RecognitionSpaceCalibration main.cpp Calibration.cpp CeresModels.cpp RGBDFrame.cpp UtilCvPclRs.cpp GLRenderer.cpp)
target_link_libraries (RecognitionSpaceCalibration ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} glfw /opt/ros/kinetic/lib/librealsense.so ceres epoxy)
