#!/usr/bin/env python
# coding: utf-8

#############################################
# LED illlumination control
# Convert ROS command messages to UDP messages
# May 3, 2017  M. Yamamoto 
#############################################
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

from __future__ import print_function
import socket
import time
from contextlib import closing
import rospy
from std_msgs.msg import String

host = '160.69.69.33'
port = 40001

str_data = String()

def main(host, port, mes):
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  with closing(sock):
      sock.sendto(mes.encode('utf-8'), (host, port))
  return

def check_sum(ss):
    sum = 0
    for i in range(len(ss)):
        sum += ord(ss[i])
    hh = hex(sum).upper()
    return(ss+hh[-2:]+chr(13)+chr(10)) 

# ROS message received
def callback(data):
	str_data = data.data
	main(host, port, check_sum('@'+data.data))
	rospy.loginfo(rospy.get_caller_id()+"I heard %s", data.data)

def get_led_intensity_callback():
	response = get_led_intensityResponse()	
	response.current_intensity.data = str_data
	return response

rospy.init_node('LEDlistener', anonymous=True)
rospy.Subscriber('LEDchatter', String, callback)
rospy.spin()








