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
from sklearn import svm
from sklearn import cross_validation
from sklearn.svm import LinearSVC
from sklearn.metrics import classification_report, accuracy_score
from sklearn.externals import joblib
from BagOfFeatures import BagOfFeatures
from tnp_svm.srv import *

codebook_size = 15
group_name = "group_all"
bof = BagOfFeatures(codebookSize=codebook_size, groupName=group_name)
bof.load()
#load_model
save_model = "./model_" + group_name + "_KAZE"
fit_model = joblib.load(save_model + "/fit_model_" + group_name + ".bin")
method = "KAZE"

def extractDescriptors(images, method):
    detector = cv2.KAZE_create()
    keypoints = map(detector.detect, images)
    descriptors = map(lambda a, b: detector.compute(a, b)[1], images, keypoints)
    return (descriptors)

class Identification:
    def __init__(self):
        self.bridge = CvBridge()
        self.Trigger_imshow = False
        self.Trigger_rgb = False
        self.Trigger_depth = False
        
    def rgb_callback(self,Image_rgb):
        try:
            rgb_image_cv2 = self.bridge.imgmsg_to_cv2(Image_rgb,"bgr8")
            self.rgb_array = np.array(rgb_image_cv2, dtype=np.uint8)
            self.Trigger_rgb = True
        except CvBridgeError,e:
            print e

    def depth_callback(self,Depth_data):
        try:
            depth_data_cv2 = self.bridge.imgmsg_to_cv2(Depth_data,"16UC1")
            depth_array_tmp = np.array(depth_data_cv2, dtype=np.int16)
            if depth_array_tmp[240][320] == 0:
                rospy.sleep(0.05)
            else:
                self.depth_array = depth_array_tmp
                self.Trigger_depth = True   
                # print(self.depth_array[240][320])
        except CvBridgeError,e:
            print e

    def recognize_by_svm_callback(self,req):
        print 'callback recognize_items_callback called'
        # print 'target_item: ', req.target_item_ids.data
        items_ids, confidences = self.identify()
        response = RecognizeBySvmResponse()
        for elem in items_ids:
            response.items_ids.append(String(elem))
        response.confidences = [Float64(confidences)]
        rospy.loginfo("Items_ids: %s",items_ids)
        rospy.loginfo("Confidences: %s",confidences)
        return response

    def subtract_background(self):
        depth_filtered = cv2.blur(self.depth_array,(10,10))
        print(np.max(depth_filtered))
        print(np.min(depth_filtered))
        print(depth_filtered[240][320])
        threshold_low = depth_filtered[240][320] -500
        threshold_high = depth_filtered[240][320] +500
        rgb = self.rgb_array
        rgb_subtracted = self.rgb_array
        for i in range (0,3):
            for j in range (0,640):
                for k in range (0,480):
                    if threshold_low < depth_filtered[k][j] < threshold_high:
                        rgb_subtracted[k][j][i] = rgb[k][j][i]
                    else:
                        rgb_subtracted[k][j][i] = 0
        return rgb_subtracted

    def identify(self):
        if self.Trigger_rgb and self.Trigger_depth == True:
            self.rgb_subtracted = self.subtract_background()
            self.Trigger_imshow = True
            features = {}
            features["test"] = extractDescriptors([self.rgb_subtracted], method=method)
            #make_histogram
            hist = {}
            hist["test"] = map(lambda a: bof.makeHistogram(np.matrix(a)), features["test"])
            #predict_label
            result_labels = fit_model.predict(hist["test"][0])    
            print("Result=%s" % (result_labels))               
            items_ids=result_labels
            #Predict margin in SVM
            confidences_matrix=fit_model.predict_proba(hist["test"][0])
            confidences=np.max(confidences_matrix[0])
            print("Confidence=%s" % (confidences))
        else:
            print("wait")
            rospy.sleep(1.0)
        return items_ids , confidences

    def start(self):
        service = rospy.Service('tnp_svm/recognize_by_svm', RecognizeBySvm, self.recognize_by_svm_callback)        
        rgb_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.rgb_callback)
        depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if self.Trigger_imshow == True:
                cv2.imshow("img",self.rgb_subtracted)
                cv2.waitKey(1)
                rate.sleep()

if __name__== "__main__":
    try:
        rospy.init_node('identification')
        Identification = Identification()
        Identification.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass