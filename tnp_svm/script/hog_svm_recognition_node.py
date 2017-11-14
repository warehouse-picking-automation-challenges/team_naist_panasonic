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
from skimage.feature import hog

hog_image_size = 64
save_model_path = "/root/catkin_ws/src/tnp/tnp_svm/script/"
fit_model = joblib.load(save_model_path + 'hog_svm_pick.pkl')

def ResizeImage(image):
	#create resized image
	height, width = image.shape[:2]
	if height>=64 and width>=64:
		#### when width is bigger than height
		if width >= height: 
			###### to crop
			resized_image = cv2.resize(image, (width*hog_image_size/height,hog_image_size))
		####when height is bigger than width
		else:  
			###### to crop
			resized_image = cv2.resize(image, (hog_image_size,height*hog_image_size/width))

	elif height>=64 and width<=64:
		resized_image = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png", cv2.IMREAD_GRAYSCALE)
		resized_image = cv2.resize(resized_image, (hog_image_size,hog_image_size))
		resized_image[0:hog_image_size,hog_image_size/2-int(width/2):hog_image_size/2+int(width/2)] = image[(height-hog_image_size)/2:height-(height-hog_image_size)/2,0:width]
	elif height<=64 and width>=64:
		resized_image = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png", cv2.IMREAD_GRAYSCALE)
		resized_image = cv2.resize(resized_image, (hog_image_size,hog_image_size))
		resized_image[hog_image_size/2-int(height/2):hog_image_size/2+int(height/2),0:hog_image_size] = image[0:height,(width-hog_image_size)/2:width-(width-hog_image_size)/2]
	elif height<=64 and width<=64:
		resized_image = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png", cv2.IMREAD_GRAYSCALE)
		resized_image = cv2.resize(resized_image, (hog_image_size,hog_image_size))
		resized_image[hog_image_size/2-int(height/2):hog_image_size/2+int(height/2),hog_image_size/2-int(width/2):hog_image_size/2+int(width/2)] = image[0:int(height/2)*2,0:int(width/2)*2]

	return resized_image

def CropCenter(image_in):
    height, width = image_in.shape[:2]
    center_h = int(height/2)
    center_w = int(width/2)
    if height >= width :
        cropped_image = image_in[center_h-hog_image_size/2:center_h+hog_image_size/2,0:width]
    elif width >  height:
        cropped_image = image_in[0:height,center_w-hog_image_size/2:center_w+hog_image_size/2]  
    return cropped_image

def ComputeHOGDescriptor(image):
    des = hog(image, orientations=9, pixels_per_cell=(8,8),cells_per_block=(3,3), visualise=False)
    return des

class Identification:
    def __init__(self):
        self.bridge = CvBridge()
        self.Trigger_imshow = False
        self.Trigger_rgb = False
        self.Trigger_depth = False
        self.items_in_use_list = []

    def preprocess_rgb(self, Image_rgb):
        try:
            rgb_image_cv2 = self.bridge.imgmsg_to_cv2(Image_rgb,"bgr8")
            rgb_image_cv2 = cv2.cvtColor(rgb_image_cv2, cv2.COLOR_RGB2GRAY)
            hog_image = np.array(rgb_image_cv2, dtype=np.uint8)
            hog_image = ResizeImage(hog_image)
            hog_image = CropCenter(hog_image)
            self.Trigger_rgb = True
            return hog_image
        except CvBridgeError,e:
            print e
        
    def recognize_by_svm_callback(self, req):
        print 'recognize_items_callback called'
        self.hog_image_0 = self.preprocess_rgb(req.rgb_data[0]) 
        self.hog_image_1 = self.preprocess_rgb(req.rgb_data[1])
        self.hog_image_2 = self.preprocess_rgb(req.rgb_data[2])
        self.hog_image_3 = self.preprocess_rgb(req.rgb_data[3])
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
            self.items_in_use_list.append(item.data)
            count = count + 1
        response = SetItemsInfoResponse()
        response.total_items = count

        return response

    def identify(self):
        if self.Trigger_rgb == True:
            features = {}
            des_test_0 = ComputeHOGDescriptor(self.hog_image_0)
            des_test_1 = ComputeHOGDescriptor(self.hog_image_1)
            des_test_2 = ComputeHOGDescriptor(self.hog_image_2)
            des_test_3 = ComputeHOGDescriptor(self.hog_image_3)
            
            if len(des_test_0) == 0:
                print "no features extracted"
                pass
            else:
                ###Predict probability
                confidences_matrix_0 = fit_model.predict_proba(des_test_0.reshape(1, -1))
                confidences_matrix_1 = fit_model.predict_proba(des_test_1.reshape(1, -1))
                confidences_matrix_2 = fit_model.predict_proba(des_test_2.reshape(1, -1))
                confidences_matrix_3 = fit_model.predict_proba(des_test_3.reshape(1, -1))
                confidences_matrix = (confidences_matrix_0 + confidences_matrix_1 + confidences_matrix_2 + confidences_matrix_3)                

                labels = fit_model.classes_
                sort_ascend = np.sort(confidences_matrix)
                sort_index_ascend = np.argsort(confidences_matrix)
                sort_descend = sort_ascend[0][::-1]
                sort_index_descend = sort_index_ascend[0][::-1]

                items_ids=[]
                for i in range (0,len(labels)):
                    items_ids.append(String(labels[sort_index_descend[i]]))
                   
                confidences = sort_descend /4
        else:
            print("invalid images or not both (rgb and depth) obtained")
            rospy.sleep(1.0)
        return items_ids , confidences

    def start(self):
        service = rospy.Service('tnp_svm/identify_item', IdentifyItem, self.recognize_by_svm_callback)
        service_items_info = rospy.Service('tnp_svm/set_items_info', SetItemsInfo, self.set_items_info_callback)

if __name__== "__main__":
    try:
        rospy.init_node('tnp_svm')
        Identification = Identification()
        Identification.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
