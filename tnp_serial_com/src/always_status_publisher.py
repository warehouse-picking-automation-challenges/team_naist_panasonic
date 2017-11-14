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
from std_msgs.msg import String

pub = rospy.Publisher('/ee_status', String, queue_size=100)
st = String()
st.data = "No data..."

def update_status(data):
    st.data = data.data
    print st

def publisher():
    rospy.init_node('status_publisher')
    r = rospy.Rate(10)
    S1 = rospy.Subscriber("/tnp_arduino/read", String, update_status)
    while not rospy.is_shutdown():
        pub.publish(st)
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
  		pass
