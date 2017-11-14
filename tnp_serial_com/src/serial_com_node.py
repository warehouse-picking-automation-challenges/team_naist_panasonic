#!/usr/bin/env python
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
import serial
import sys
from std_msgs.msg import String

pub = rospy.Publisher('read', String, queue_size=100)
st = String()

args = sys.argv
ser = serial.Serial(args[1], args[2])#9600 or 115200

ser.rtscts=False
ser.dsrdtr=False
ser.rts=False
ser.dtr=False


def callback(data): 
	ser.write("%s\n"%(data.data))

def serialcom():
	rospy.init_node('serial_com')
	S1 = rospy.Subscriber("write", String, callback)
	while not rospy.is_shutdown():
		try: 
			line = ser.readline()
			st.data = line.rstrip()
			print st.data
			pub.publish(st)
		except:
			print("Error while reading from serial or something")


if __name__ == '__main__':
	try:
		serialcom()
	except rospy.ROSInterruptException:
  		pass
