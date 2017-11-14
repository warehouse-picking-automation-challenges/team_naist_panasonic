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
import os ,re
import argparse
from tnp_svm.srv import *
import pickle
from skimage import data, color, exposure
from skimage.feature import hog
from sklearn import svm
from sklearn.svm import LinearSVC
from sklearn.calibration import CalibratedClassifierCV
from sklearn import cross_validation
from sklearn.cross_validation import train_test_split
from sklearn.grid_search import GridSearchCV
from sklearn.externals import joblib
from sklearn.metrics import roc_curve
import math
from scipy import ndimage

codebook_size = 15
group_name = "competition_test"

hog_image_size= 64

def getArgs():
    parser = argparse.ArgumentParser()

    parser.add_argument("--codebook-size", type=int, default=15, help="codebook size")
    parser.add_argument("--data-dir", choices=["./data", "./data_group", "./data_amazon"], type=str, default="./realdata", help="choose data directory")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--kernel", choices=["linear", "rbf"], type=str, default="linear", help="kernel functions")
    group.add_argument("--linear-svm", action="store_true", default=False, help="use LinearSVM")

    return parser.parse_args()

def Remove_black(image, original_height, original_width, angle):
	height, width = image.shape[:2]
	center_h = int(height/2)
	center_w = int(width/2)
	if original_width>=original_height:
		if angle == 0 or angle == 5 or angle == 355 or angle == 175 or angle == 180 or angle == 185:
			cropped_image = image[int(original_height*0.02)+center_h-original_height/2:center_h+original_height/2-int(original_height*0.02),int(original_width*0.02):width-int(original_width*0.02)]
		else:
			cropped_image = image[int(original_height*0.02):height-int(original_height*0.02),int(original_width*0.02)+center_w-original_height/2:center_w+original_height/2-int(original_width*0.02)]
	else:
		if angle == 0 or angle == 5 or angle == 355 or angle == 175 or angle == 180 or angle == 185:
			cropped_image = image[int(original_height*0.02):height-int(original_height*0.02),int(original_width*0.02)+center_w-original_width/2:center_w+original_width/2-int(original_width*0.02)]
		else:
			cropped_image = image[int(original_height*0.02)+center_h-original_width/2:center_h+original_width/2-int(original_height*0.02),int(original_width*0.02):width-int(original_width*0.02)]
	return cropped_image


def Resize_image(image):
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
	

def Crop_center(image_in):
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
	
def LoadFile(filename, rgb_image_path):
	label = rgb_image_path.replace('./realdata/' + group_name + '/','')
	label = label.lower()
	#load amazon images                
	Path = rgb_image_path+ "/" + filename  
	image = cv2.imread(Path, cv2.IMREAD_GRAYSCALE)
	image = Resize_image(image)
	image = Crop_center(image)
	return label, image

def Matching():
	args = getArgs()
	group_dir = group_name
	start = time.time()
	category_dirs = os.listdir(os.path.join(args.data_dir, group_dir))    
	image = {}
	des = []
	labels=[]
	
	svm_best = joblib.load('32items.pkl')
	
	label, test_image0 = LoadFile("rectified_Windex52_cam0_masked.png", "/root/catkin_ws/src/tnp/tnp_svm/script/realdata/For_test_ver3_out")
	label, test_image1 = LoadFile("rectified_Windex52_cam1_masked.png", "/root/catkin_ws/src/tnp/tnp_svm/script/realdata/For_test_ver3_out")
	label, test_image2 = LoadFile("rectified_Windex52_cam2_masked.png", "/root/catkin_ws/src/tnp/tnp_svm/script/realdata/For_test_ver3_out")
	label, test_image3 = LoadFile("rectified_Windex52_cam3_masked.png", "/root/catkin_ws/src/tnp/tnp_svm/script/realdata/For_test_ver3_out")

	des_test0 = ComputeHOGDescriptor(test_image0)
	des_test1 = ComputeHOGDescriptor(test_image1)
	des_test2 = ComputeHOGDescriptor(test_image2)
	des_test3 = ComputeHOGDescriptor(test_image3)

	###accuracy_test_from_loadimage (training data itself)
	total_counter=0
	true_counter=0
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(os.path.join(args.data_dir, group_dir, category_dir)):
			for file in os.listdir(os.path.join(args.data_dir, group_dir, category_dir)):
				if (file[-4:] == '.png'):
					label, image = LoadFile(file ,rgb_image_path)
					des_test = ComputeHOGDescriptor(image)
					testResponse = svm_best.predict(des_test)
					total_counter += 1 
					print(file)
					print("label=%s" % (label))
					print("response=%s" % (testResponse))
					if label == testResponse:
						true_counter +=1
	print("true_counter=%s" % (true_counter))
	print("total_counter=%s" % (total_counter))
	print("accuracy=%s" % (true_counter/total_counter)) 
	######

	###### SVM recognition
	classes = svm_best.classes_
	print("class length" , len(classes))
	confidences_matrix_0 = svm_best.predict_proba(des_test0.reshape(1, -1))
	confidences_matrix_1 = svm_best.predict_proba(des_test1.reshape(1, -1))
	confidences_matrix_2 = svm_best.predict_proba(des_test2.reshape(1, -1))
	confidences_matrix_3 = svm_best.predict_proba(des_test3.reshape(1, -1))
	
	confidences_matrix = (confidences_matrix_0 + confidences_matrix_1 + confidences_matrix_2 + confidences_matrix_3)/4                

	print "type", type(confidences_matrix)
	labels = svm_best.classes_
	sort_ascend = np.sort(confidences_matrix)
	# print(sort_ascend)
	sort_index_ascend = np.argsort(confidences_matrix)
	# print(sort_index_ascend)
	sort_descend = sort_ascend[0][::-1]
	print "sort_descend" ,sort_descend[0:10]
	sort_index_descend = sort_index_ascend[0][::-1]
	# print(sort_index_descend) 
	items_ids=[]
	for i in range (0,len(labels)):
	    items_ids.append(labels[sort_index_descend[i]])
	print(items_ids)
	print "len(labels)", len(labels)


if __name__== "__main__":
    Matching()
