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
from std_msgs.msg import String ,Float64 ,Bool
import numpy as np
import os ,re
import argparse
import math

data_dir_path = "/root/catkin_ws/src/tnp/tnp_svm/script/data"
group_name = "Training_items"
save_group_name = "Resized_Training_items"

image_size= 480

def Resize_image(image):
	#create resized image
	height, width = image.shape[:2]
	# when width is bigger than height
	if width >= height: 
		resized_image = cv2.resize(image, (image_size,height*image_size/width))
	#when height is bigger than width
	else:  
		resized_image = cv2.resize(image, (width*image_size/height,image_size))
	return resized_image

	
def LoadFile(filename, rgb_image_path):              
	Path = rgb_image_path+ "/" + filename  
	image = cv2.imread(Path)
	image = Resize_image(image)
	return image

def SaveFile(filename, image, rgb_image_path):
	ClassName = rgb_image_path.replace('./data/' + group_name + '/','')
	ClassName = ClassName.lower()
	if not os.path.exists("./data/" + save_group_name + "/" + ClassName):
		os.makedirs("./data/" + save_group_name + "/" + ClassName)
	Path = './data/' + save_group_name + "/" + ClassName + "/" + filename  
	cv2.imwrite(Path, image)

def Process():
	if not os.path.exists(data_dir_path + "/" + save_group_name):
		os.makedirs(data_dir_path + "/" + save_group_name)
	group_dir = group_name
	start = time.time()
	category_dirs = os.listdir(data_dir_path + "/" + group_dir)    
	image = {}

	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(data_dir_path + "/" + group_dir + "/" + category_dir):
			for filename in os.listdir(data_dir_path + "/" + group_dir + "/" + category_dir):
				if (filename[-4:] == '.png'):
					image = LoadFile(filename, rgb_image_path)
					SaveFile(filename, image, rgb_image_path)					
					print("ClassName=%s" % (filename))
					
if __name__== "__main__":
    Process()
