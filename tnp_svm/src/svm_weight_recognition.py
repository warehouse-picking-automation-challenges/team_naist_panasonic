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
group_name = "SVM_40items"
input_dir = "./input"
bof = BagOfFeatures(codebookSize=codebook_size, groupName=group_name)
bof.load()
#load_model
fit_model = joblib.load("./model/fit_model_" + group_name + ".bin")

def loadImages(path):
    imagePathes = glob.glob(os.path.join(path, "*.png"))
    images = map(cv2.imread, imagePathes)
    return (images)

def extractDescriptors(images, method):
    detector = cv2.xfeatures2d.SIFT_create()
    keypoints = map(detector.detect, images)
    descriptors = map(lambda a, b: detector.compute(a, b)[1], images, keypoints)
    return (descriptors)

class Identification:
    def __init__(self):
        self.bridge = CvBridge()
        self.Trigger_rgb = False
        self.Trigger_depth = False
        
    def rgb_callback(self,Image_rgb):
        try:
            self.rgb_image_cv2 = self.bridge.imgmsg_to_cv2(Image_rgb,"bgr8")
            self.rgb_array = np.array(self.rgb_image_cv2, dtype=np.uint8)
            self.Trigger_rgb = True
        except CvBridgeError,e:
            print e

    def depth_callback(self,Depth_data):
        try:
            self.depth_data_cv2 = self.bridge.imgmsg_to_cv2(Depth_data,"16UC1")
            self.depth_array = np.array(self.depth_data_cv2, dtype=np.int16)
            self.Trigger_depth = True   
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

    def subtract_background(self,depth,rgb):
        depth_filtered = cv2.blur(depth,(10,10))
        rgb_subtracted = rgb
        if depth_filtered[240][320] == 0:
            print("wait for depth sensor")
        else:
            #set camera solution
            for i in range (0,3):
                for j in range (0,640):
                    for k in range (0,320):
                        if 3000 < depth_filtered[k][j] < 6000:
                            # print(depth_filtered[240][320])
                            rgb_subtracted = rgb
                        else:
                            rgb_subtracted[k][j][i] = 0
        return rgb_subtracted

    def weight_callback(self,weight):
        if 0<weight<0.125:
            self.group1_Trigger = True
        elif 0.125<weight<0.20:
            self.group2_Trigger = True
        elif 0.20<weight<0.25:
            self.group3_Trigger = True
        elif 0.25<weight<0.31:
            self.group4_Trigger = True
        elif 0.31<weight<0.40:
            self.group5_Trigger = True
        elif 0.40<weight<0.55:
            self.group6_Trigger = True
        elif 0.55<weight<1.00:
            self.group7_Trigger = True
        
    def identify(self):
        if self.Trigger_rgb and self.Trigger_depth == True:
            rgb_subtracted = self.subtract_background(self.depth_array,self.rgb_array)
            features = {}
            features["test"] = extractDescriptors([rgb_subtracted], method="SIFT")
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
            cv2.imshow("img",rgb_subtracted)
            cv2.waitKey(1)
        else:
            print("wait")
            rospy.sleep(1.0)
        return items_ids , confidences

    def start(self):
        #service,topic
        service = rospy.Service('tnp_svm/recognize_by_svm', RecognizeBySvm, self.recognize_by_svm_callback)        
        rgb_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.rgb_callback)
        weight_sub = rospy.Subscriber("tnp_weight_events/weight_difference", Float64, self.weight_callback)
        depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)

if __name__== "__main__":
    try:
        rospy.init_node('identification')
        Identification = Identification()
        Identification.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass