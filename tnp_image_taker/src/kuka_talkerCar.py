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

import rospy
from std_msgs.msg import Header
from iiwa_msgs.msg import CartesianEulerPose, CartesianQuantity, ControlMode
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from iiwa_msgs.srv import ConfigureSmartServo

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    print('pub1')
    rate = rospy.Rate(10) # 10hz
    C = PoseStamped()
    C.header = Header()
    C.header.stamp = rospy.Time.now()
    C.pose.position = Point()

    while not rospy.is_shutdown():
        print('high')
        duration = 10.0
        C.pose.position.x = -0.0234378140834
        C.pose.position.y = -0.533953078892
        C.pose.position.z = 0.499789786528
        C.pose.orientation.x =0.695140765407
        C.pose.orientation.y =0.714312195778
        C.pose.orientation.z =0.0603375173907
        C.pose.orientation.w =-0.0538219537931
        pub.publish(C)
        rospy.sleep(duration)
        print('middle')
        C.pose.position.x = -0.0234378140834
        C.pose.position.y = -0.533953078892
        C.pose.position.z = 0.450
        C.pose.orientation.x =0.695140765407
        C.pose.orientation.y =0.714312195778
        C.pose.orientation.z =0.0603375173907
        C.pose.orientation.w =-0.0538219537931
        pub.publish(C)
        rospy.sleep(duration)
        
        print('low')
        C.pose.position.x = -0.0234378140834
        C.pose.position.y = -0.533953078892
        C.pose.position.z = 0.400
        C.pose.orientation.x =0.695140765407
        C.pose.orientation.y =0.714312195778
        C.pose.orientation.z =0.0603375173907
        C.pose.orientation.w =-0.0538219537931
        pub.publish(C)
        rospy.sleep(duration)
        rospy.sleep(duration)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass