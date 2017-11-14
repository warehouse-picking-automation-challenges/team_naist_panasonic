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

import sys
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String ,Float64 ,Bool
import numpy as np
from sklearn import svm
from sklearn.svm import LinearSVC
from sklearn.externals import joblib
from tnp_svm.srv import *
import os

def get_right_path(path):
    if path.startswith("./") or path.startswith("../"):
        process_folder_path = os.path.split(os.path.realpath(__file__))[0]
        path = os.path.realpath(os.path.join(process_folder_path,path))
        return path
    else:
        return path

fit_model = joblib.load(get_right_path("./second_svm_parameter.pkl"))


def remap_xyz(OldValue,OldMax=0.45,OldMin=0,NewMax=0.9,NewMin=0.1):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue

def remap_volume(OldValue,OldMax=0.01,OldMin=0,NewMax=0.9,NewMin=0.1):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue

def remap_weight(OldValue,OldMax=1200,OldMin=0,NewMax=0.9,NewMin=0.1):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue


class Identification:
    def __init__(self):
        self.bridge = CvBridge()
        self.Trigger_imshow = False
        self.Trigger_rgb = False
        self.Trigger_depth = False
        self.items_in_use_list = []

    def recognize_by_svm_callback(self, req):
        print 'recognize_items_callback called'

        self.vector = req #Get info from msg

        #TODO std_msgs/String[] cam_ids, sensor_msgs/CameraInfo cam_info
        items_ids, confidences = self.identify()
        rospy.loginfo("Items_ids: %s",items_ids)
        rospy.loginfo("Confidences: %s",confidences)
        response = IdentifyItemResponse()
        response.items_ids = items_ids
        for elem in confidences:
            response.confidences.append(Float64(elem))
        return response

    def set_items_info_callback(self, req):
        count = 0
        for item in req.items_ids:
            self.items_in_use_list.append(item.data) # TODO double check this
            count = count + 1
        response = SetItemsInfoResponse()
        response.total_items = count

        return response

    def identify(self):
        features = {}
        vector = self.vector

        bbox_xyz = [self.vector.x, self.vector.y, self.vector.z]
        volume = bbox_xyz[0]*bbox_xyz[1]*bbox_xyz[2]
        weight = self.vector.weight

        xyz_normalized = []
        for value in bbox_xyz:
            xyz_normalized.append(remap_xyz(value,0.45,0,0.9,0.1))
        xyz_normalized = sorted(xyz_normalized)
        volume_normalized = remap_volume(volume)
        weight_normalized = remap_weight(weight)

        vector_normalized = xyz_normalized
        vector_normalized.append(volume_normalized)

        vector_normalized.append(weight_normalized)

        ###predict_label
        result_label = fit_model.predict(vector_normalized)

        ###Predict probability
        confidences_matrix = fit_model.predict_proba(vector_normalized)

        # print(confidences_matrix)
        labels = fit_model.classes_
        sort_ascend = np.sort(confidences_matrix)
        # print(sort_ascend)
        sort_index_ascend = np.argsort(confidences_matrix)
        # print(sort_index_ascend)
        sort_descend = sort_ascend[0][::-1]
        # print(sort_descend)
        sort_index_descend = sort_index_ascend[0][::-1]
        # print(sort_index_descend)
        print("labels:")
        print(labels)
        items_ids=[]
        for i in range (0,len(labels)):
            items_ids.append(String(labels[sort_index_descend[i]]))

        # print(items_ids)
        confidences = sort_descend
        # print("Confidence=%s" % (confidences))
        
        return items_ids , confidences

    def start(self):
        service = rospy.Service('tnp_svm_second/identify_item', IdentifyItem, self.recognize_by_svm_callback)
        service_items_info = rospy.Service('tnp_svm_second/set_items_info', SetItemsInfo, self.set_items_info_callback)
        rate = rospy.Rate(10)


if __name__== "__main__":
    try:
        rospy.init_node('tnp_svm_second')
        Identification = Identification()
        Identification.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
