#!/usr/bin/env python

#############################################
# LED set LED intensity
# 1 June, 2017  Seigo Okada
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

import rospy
from std_msgs.msg import String , Int16 , Bool
import time
from tnp_led_control.srv import *

class LED:
	def __init__(self):		
		self.current_led_state_ = [-1] * 8

	# we're expecting values from 1 to 8 in the request (1 is the end effector the others are marked starting with an L)
	def set_led_intensity_callback(self, req):
		response = set_led_intensityResponse()
		
		# sanity check
		if req.target_led_num.data < 1 or req.target_led_num.data > 8:
			rospy.logerr("Invalid led_num: %d. Use led_num values between [1-8] (1 is end effector)", req.target_led_num.data)
			return False;
		if req.target_intensity.data < 0 or req.target_intensity.data > 255:
			rospy.logerr("Invalid target_intensity: %d. Use value range of [0-255]", req.target_intensity.data)
			return False;		
		
		# encode led state and publish it
		led_msg = '0' + str(req.target_led_num.data - 1) + 'F' + str(req.target_intensity.data).zfill(3)  # format example: 00F
		self.led_service_publisher_.publish(led_msg)
		self.current_led_state_[req.target_led_num.data - 1] = req.target_intensity.data;
		rospy.loginfo("Setting led %d to intensity %d", req.target_led_num.data, req.target_intensity.data)
		response.succeeded = True
		time.sleep(0.05) # NOTE: Competition optimization

		return response

	def get_led_intensity_callback(self, req):
		response = get_led_intensityResponse()
		
		# sanity check 
		if req.target_led_num.data < 1 or req.target_led_num.data > 8:
			rospy.logerr("Invalid led_num: %d. Use led_num values between [1-8] (1 is end effector)", req.target_led_num.data)
			response.current_intensity.data = -2 # unknown LED num
		else:
			response.current_intensity.data = self.current_led_state_[req.target_led_num.data - 1]
		
			if response.current_intensity.data == -1:
				rospy.logwarn("led_num %d hasn't been set in this session.", req.target_led_num.data)
				
		return response

	def subscribe_callback(self, current_intensity_sub):
		rospy.loginfo("New subscriber")

	def talker(self):
		rospy.init_node('LED_controller')
		self.led_service_publisher_ = rospy.Publisher('LEDchatter', String, queue_size=10)
		str_sub = rospy.Subscriber('/LEDchatter', String, self.subscribe_callback , queue_size=10)
		set_service = rospy.Service('tnp_led_control/set_led_intensity', set_led_intensity, self.set_led_intensity_callback)		
		get_service = rospy.Service('tnp_led_control/get_led_intensity', get_led_intensity, self.get_led_intensity_callback)
		on_str = "01L1"  # Channel 00, switch ON 
		self.led_service_publisher_.publish(on_str)
			
		rospy.loginfo("LED control node ready")

		while not rospy.is_shutdown():		
			rospy.spin()

if __name__ == '__main__':
    try:
		LED= LED()
		LED.talker()
    except rospy.ROSInterruptException: 
    	pass
