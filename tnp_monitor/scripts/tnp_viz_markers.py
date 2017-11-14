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
import math
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from tnp_msgs.srv import *
from collections import deque

from multiprocessing import Lock
import threading

# For reference: http://wiki.ros.org/PyStyleGuide
class VizMarkersNode(object):

    def __init__(self, publish_rate=10):
        # Initialize a circular buffer that markers remain in until they expire
        self.markers_ = deque()
        self.my_mutex = threading.Lock()
        self.lifetime = 120.0       # Default node lifetime
        self.create_marker_dictionary()

    def update_markers(self, the_argument_ros_seems_to_need):
        self.my_mutex.acquire()

        num_of_markers_expired = 0
        for marker in self.markers_:
            node_age = rospy.Time.now() - marker.header.stamp
            if node_age.to_sec() > self.lifetime:
                num_of_markers_expired += 1
            else:
                marker.color.a = self.set_alpha(node_age.to_sec())
                self.markers_pub_.publish(marker)

        for i in range(num_of_markers_expired):
            self.markers_.popleft()

        self.my_mutex.release()
        return True

    def set_alpha(self, age):
        alpha = (self.lifetime - age) / self.lifetime
        return alpha

    def create_and_publish_marker(self, name, pose, ns=''):
        self.my_mutex.acquire()
        # Create marker
        marker = Marker()
        try:
            marker.type = self.marker_primitives[name].type
            marker.scale.x = self.marker_primitives[name].scale.x
            marker.scale.y  = self.marker_primitives[name].scale.y
            marker.scale.z = self.marker_primitives[name].scale.z
            marker.color.r = self.marker_primitives[name].color.r
            marker.color.g = self.marker_primitives[name].color.g
            marker.color.b = self.marker_primitives[name].color.b
            marker.color.a = self.marker_primitives[name].color.a
        except:     # Use default if not found as template
            rospy.warn('default marker') # TODO this doesn't disappear properly
            marker.type = 2 # sphere
            marker.scale.x = .1
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "iiwa_link_0"
        marker.ns = ns
        if not self.markers_:
            marker.id = 1
        else:
            marker.id = self.markers_[-1].id + 1            # TODO: Make this counter restart from 0 if it gets too big
        marker.action = 0   # Add/modify
        marker.pose = pose
        marker.lifetime = rospy.Duration(self.lifetime)  # This is not very meaningful as it gets reset to the same value when the marker is updated
        # append the new marker and publish it
        self.markers_.append(marker)
        self.markers_pub_.publish(marker)
        self.my_mutex.release()

    def show_marker_callback(self, req):
        self.my_mutex.acquire()
        #self.markers_.append(req.marker) # FIXME this append may be a problem
        self.markers_pub_.publish(req.marker)
        self.my_mutex.release()
        response = ShowMarkerResponse()
        return response

    def show_pose_callback(self, req):
        self.create_and_publish_marker(req.name, req.pose, req.ns)
        response = ShowPoseResponse()
        return response

    def show_grasp_pose_callback(self, req):
        pad_pose_1 = Pose()
        pad_pose_2 = Pose()

        pad_pose_1.position = req.pad_point_1
        pad_pose_1.orientation = req.pad_orientation
        pad_pose_2.position = req.pad_point_2
        pad_pose_2.orientation = req.pad_orientation
        
        # Shift position upwards to accommodate for height of the marker
        pad_pose_1.position.z += .05
        pad_pose_2.position.z += .05

        self.create_and_publish_marker("grasp_pad_point", pad_pose_1, req.ns)
        self.create_and_publish_marker("grasp_pad_point", pad_pose_2, req.ns)
        response = ShowPoseResponse()
        return response

    def create_marker_dictionary(self):
        # This contains the default settings for different kinds of markers
        self.marker_primitives = dict()

        # For "target_item_pose", use these settings:
        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .05
        marker.scale.y = .05
        marker.scale.z = .02
        # Cyan
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_primitives["target_item_pose"] = marker

        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .05
        # Pink
        marker.color.r = 1.0
        marker.color.g = 20.0/255.0
        marker.color.b = 107.0/255.0
        self.marker_primitives["refused_item_pose"] = marker

        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .05
        marker.scale.y = .05
        marker.scale.z = .05
        # Blue
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_primitives["target_robot_pose"] = marker

        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .08
        marker.scale.y = .08
        marker.scale.z = .02
        # Orange
        marker.color.r = 1.0
        marker.color.g = 160.0/255.0
        marker.color.b = 77.0/255.0
        self.marker_primitives["target_robot_pose_flange"] = marker

        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .03
        marker.scale.y = .03
        marker.scale.z = .1
        # Blue
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_primitives["target_robot_pose_EE"] = marker

        marker = Marker()
        marker.type = 2 # sphere
        marker.scale.x = .07
        marker.scale.y = .07
        marker.scale.z = .04
        # Cyan
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_primitives["target_item_pose_task_manager"] = marker

        marker = Marker()
        marker.type = 1 # cube
        marker.scale.x = .01
        marker.scale.y = .02
        marker.scale.z = .1
        # ABS color
        marker.color.r = 245.0/255.0
        marker.color.g = 215.0/255.0
        marker.color.b = 147.0/255.0
        self.marker_primitives["grasp_pad_point"] = marker


    def start_ros(self):
        # publishers
        self.markers_pub_ = rospy.Publisher('/tnp_monitor/visualization_marker', Marker, queue_size=10)

        # services we advertise
        rospy.Service('tnp_monitor/showMarker', ShowMarker, self.show_marker_callback)
        rospy.Service('tnp_monitor/showPose', ShowPose, self.show_pose_callback)
        rospy.Service('tnp_monitor/showGraspPose', ShowGraspPose, self.show_grasp_pose_callback)

        rospy.loginfo("visualization_marker ready.")

        rospy.Timer(rospy.Duration(.5), self.update_markers)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tnp_weight_events')
    try:
        node = VizMarkersNode()
        node.start_ros()
    except rospy.ROSInterruptException:
        pass