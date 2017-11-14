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
from IPython.core.debugger import Tracer; keyboard = Tracer()
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
from sklearn.multiclass import OneVsRestClassifier
from sklearn.metrics import roc_curve
import math

argv = sys.argv

data_dir_path = argv[1]
load_group_name = argv[2]
save_model_name = argv[3]

hog_image_size= 64

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
	label = rgb_image_path.replace(data_dir_path +  '/' + load_group_name + '/','')
	label = label.lower()        
	Path = rgb_image_path+ "/" + filename  
	image = cv2.imread(Path, cv2.IMREAD_GRAYSCALE)
	image = Resize_image(image)
	image = Crop_center(image)
	return label, image

def Training():
	start = time.time()
	category_dirs = os.listdir(data_dir_path + "/" + load_group_name)    
	image = {}
	des = []
	labels=[]
	
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(data_dir_path + "/" + load_group_name + "/" + category_dir):
			for filename in os.listdir(data_dir_path + "/" + load_group_name + "/" + category_dir):
				if (filename[-4:] == '.png'):
					label, image = LoadFile(filename, rgb_image_path)
					des1 = ComputeHOGDescriptor(image)
					labels.append(label)
					print("ClassName=%s" % (label))
					des.append(des1)
	
	########## IF NOT DOING PARAMETER TUNING
	svm_best = LinearSVC(C=5.0)

	#### CalibratedClassifierCV is the calibrated classifier which can give probabilistic classifier
	####sigmoid will use Platt's scaling. Refer to documentation for other methods.
	svm_best = CalibratedClassifierCV(svm_best,method='sigmoid',cv=3) 
	svm_best.fit(des, labels)

	joblib.dump(svm_best, save_model_name, compress=9)

	end = time.time()
	timecost = end -start
	print "timecost" +str(timecost)

	print "finished training!"

if __name__== "__main__":
    Training()
