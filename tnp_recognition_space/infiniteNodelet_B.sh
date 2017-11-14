#!/bin/bash
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

# starting two cameras recovering from crashes
source /opt/ros/kinetic/setup.bash
source /root/catkin_ws/devel/setup.bash
cd /root/catkin_ws/src/tnp

while [ true ]; do

	rostopic pub -1 /tnp_recognition_space/state std_msgs/String "data: 'Recognition nodelet B: Starting...'"
	roslaunch tnp_recognition_space tnp_rec_space_B_2x_SR300.launch
	rostopic pub -1 /tnp_recognition_space/state std_msgs/String "data: 'Recognition nodelet B: Died.'"
	./tnp_recognition_space/usbutils/clearUSBbus.sh   
	sleep 10

done

