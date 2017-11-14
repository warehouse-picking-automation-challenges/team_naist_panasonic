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

import glob
import sys
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String ,Float64 ,Bool
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from tnp_svm.srv import *


class TestSVMRecognition:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_is_ok = False
        
    def rgb_callback(self, Image_rgb):
        self.Image_rgb = Image_rgb

    def depth_callback(self, Depth_data):
        try:
            depth_data_cv2 = self.bridge.imgmsg_to_cv2(Depth_data,"16UC1")
            depth_array_tmp = np.array(depth_data_cv2, dtype=np.int16)
            # print(np.max(depth_array_tmp))
            if np.max(depth_array_tmp) == 0:
                rospy.sleep(0.05)
                self.depth_is_ok = False
            else:
                self.Depth_data = Depth_data
                self.depth_is_ok = True   
        except CvBridgeError,e:
            print e        
        
    def cam_info_callback(self, cam_info_data):
        self.cam_info = cam_info_data  

    def test_recognize_by_svm_callback(self, req):
        print 'test_svm_callback called'
        response = TestRecognizeBySVMResponse()
        print 'wait for service'
        rospy.wait_for_service('tnp_svm/identify_item')
        try:
            print("identify call")
            recognize_by_svm = rospy.ServiceProxy('tnp_svm/identify_item', IdentifyItem)          
            cam_ids = [String("L")] # L C R B E(?)
            rgb_info_data = [self.cam_info]
            depth_info_data = [self.cam_info] #FIXME this should be from the depth cam info
            rgb_data = [self.Image_rgb,self.Image_rgb,self.Image_rgb,self.Image_rgb]
            depth_data = [self.Depth_data,self.Depth_data,self.Depth_data,self.Depth_data]
            resp = recognize_by_svm(cam_ids, rgb_info_data, depth_info_data, rgb_data, depth_data)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return response
   
    def start(self):
        service = rospy.Service('tnp_svm/test_recognize_by_svm', TestRecognizeBySVM, self.test_recognize_by_svm_callback)        
        rgb_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        rgb_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        cam_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.cam_info_callback)

if __name__== "__main__":
    try:
        rospy.init_node('test_tnp_svm')
        testSVMRecognition = TestSVMRecognition()
        testSVMRecognition.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass