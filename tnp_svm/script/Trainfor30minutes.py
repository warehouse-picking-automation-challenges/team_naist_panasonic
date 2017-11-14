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

argv = sys.argv

data_dir_path = argv[1]

hog_image_size= 64

def PasteOnBlack(image_in):
	height, width = image_in.shape[:2]
	center_h = int(height/2)
	center_w = int(width/2)
	if width >= height:
		background = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png",cv2.IMREAD_GRAYSCALE)
		background = cv2.resize(background, (width,width))
		background[center_w-height/2:center_w+height/2,0:width] = image_in[0:int(height/2)*2,0:int(width/2)*2]
	else:
		background = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png",cv2.IMREAD_GRAYSCALE)
		background = cv2.resize(background, (height,height))
		background[0:height,center_h-width/2:center_h+width/2] = image_in[0:int(height/2)*2,0:int(width/2)*2]
	return background

def RemoveBlack(image, original_height, original_width, angle):
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

def CropCenter(image_in):
	height, width = image_in.shape[:2]
	center_h = int(height/2)
	center_w = int(width/2)
	if height >= width :
		cropped_image = image_in[center_h-center_w:center_h+center_w,0:width]
	elif width >  height:
		cropped_image = image_in[0:height,center_w-center_h:center_w+center_h]	
	return cropped_image

def Resize_image(image):
	#create  hog-image-size-image
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

def ComputeHOGDescriptor(image):
	des = hog(image, orientations=9, pixels_per_cell=(8,8),cells_per_block=(3,3), visualise=False)
	return des

def Training():
	start = time.time()
	category_dirs = os.listdir(data_dir_path)    
	image = {}
	des = []
	labels=[]
	angle = [0,90,180,270]

	print "loading..."
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(data_dir_path + "/" + category_dir):
			for filename in os.listdir(data_dir_path + "/" + category_dir):
				if (filename[-4:] == '.png'):
					Path = rgb_image_path+ "/" + filename  
					image = cv2.imread(Path, cv2.IMREAD_GRAYSCALE)
					print(filename)					
					original_height, original_width = image.shape[:2]
					for r in range(0,4):	
						### make square images
						pasted_image = PasteOnBlack(image)
						rotated_image = ndimage.rotate(pasted_image, angle[r], reshape=False) 
						removed_image = RemoveBlack(rotated_image, original_height, original_width, angle[r])
						cropped_image = CropCenter(removed_image)
						hog_image = Resize_image(cropped_image)
						label1 = rgb_image_path.replace(data_dir_path + '/','')
						label1 = label1.lower()
						labels.append(label1)
						des1 = ComputeHOGDescriptor(hog_image)
						des.append(des1)
						print("ClassName=%s" % (label1))

	svm_best = LinearSVC(C=5.0)

	#### CalibratedClassifierCV is the calibrated classifier which can give probabilistic classifier
	####sigmoid will use Platt's scaling. Refer to documentation for other methods.
	print "training..."
	svm_best = CalibratedClassifierCV(svm_best,method='sigmoid',cv=3) 
	svm_best.fit(des, labels)

	joblib.dump(svm_best, 'Trained_HOG_SVM0721.pkl', compress=9)

	end = time.time()
	timecost = end -start
	print "timecost" +str(timecost)

	print "finished training!"

if __name__== "__main__":
	Training()