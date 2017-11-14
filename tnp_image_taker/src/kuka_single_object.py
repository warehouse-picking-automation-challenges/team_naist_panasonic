#!/usr/bin/env python
## Program to control KUKA-chan with CartesianPose
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

import tf
import rospy
from std_msgs.msg import Header
from iiwa_msgs.msg import CartesianEulerPose, CartesianQuantity, ControlMode
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from iiwa_msgs.srv import ConfigureSmartServo

def talker():
	rospy.init_node('talker', anonymous=True)
	pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	C = PoseStamped()
	C.header = Header()
	C.header.stamp = rospy.Time.now()
	C.pose.position = Point()

	direction = []
	direction = ['0','45','90','135','180','225','270','315']
	duration = 5.0
	r=0.10

	position_x = 0.518724337869
	position_y = 0.0170282023198
	position_z = 0.600
	position_u = -178.74 /180*math.pi
	position_v = -8.52 /180*math.pi
	position_w = 179.80 /180*math.pi

	center_x = 0.518724337869 
	center_y = 0.0170282023198
	center_z = 0.0

	rospy.sleep(duration)

	while not rospy.is_shutdown():

		C.pose.position.x = 0.518724337869
		C.pose.position.y = 0.0170282023198
		C.pose.position.z = 0.600
		C.pose.orientation.x =-0.0107676379336
		C.pose.orientation.y =0.997174799442
		C.pose.orientation.z =0.000923524488372
		C.pose.orientation.w =-0.0743348046154
		print('center_position')
		pub.publish(C)
		rospy.sleep(duration)
		for i in range(8):
			position_x = center_x + r*math.cos(i*math.pi/4)
			position_y = center_y + r*math.sin(i*math.pi/4)
			position_z = position_z            
			alpha = math.atan2(math.fabs(position_y-center_y),math.fabs(position_z-center_z));
			beta = math.atan2(math.fabs(position_x-center_x),math.fabs(position_z-center_z));
			if position_y > 0:
			    euler_u = position_u + alpha
			else:
			    euler_u = position_u - alpha
			if position_x-center_x> 0:
			    euler_v = position_v - beta
			else:
			    euler_v= position_v + beta
			euler_w = position_w
			C.pose.position.x = position_x
			C.pose.position.y = position_y
			C.pose.position.z = position_z   
			C.pose.orientation.x = tf.transformations.quaternion_from_euler(euler_u,euler_v,euler_w)[0]
			C.pose.orientation.y = tf.transformations.quaternion_from_euler(euler_u,euler_v,euler_w)[1]
			C.pose.orientation.z = tf.transformations.quaternion_from_euler(euler_u,euler_v,euler_w)[2]
			C.pose.orientation.w = tf.transformations.quaternion_from_euler(euler_u,euler_v,euler_w)[3]
			pub.publish(C)
			rospy.sleep(duration)
			print(direction[i])

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

