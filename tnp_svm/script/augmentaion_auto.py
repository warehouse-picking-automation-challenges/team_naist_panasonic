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
from PIL import Image, ImageOps
import math
from scipy import ndimage

argv = sys.argv

data_dir_path = argv[1]
load_group_name = argv[2]
save_group_name = argv[3]
folder_number_for_auto = int(argv[4])


def paste_on_black(image_in):
	height, width = image_in.shape[:2]
	center_h = int(height/2)
	center_w = int(width/2)
	if width >= height:
		background = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png")
		background = cv2.resize(background, (width,width))
		background[center_w-height/2:center_w+height/2,0:width] = image_in[0:int(height/2)*2,0:int(width/2)*2]
	else:
		background = cv2.imread("/root/catkin_ws/src/tnp/tnp_svm/script/background/blackimage.png")
		background = cv2.resize(background, (height,height))
		background[0:height,center_h-width/2:center_h+width/2] = image_in[0:int(height/2)*2,0:int(width/2)*2]
	return background

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

def SaveFile(filename, image, rgb_image_path):
	ClassName = rgb_image_path.replace(data_dir_path + '/' + load_group_name + '/','')
	ClassName = ClassName.lower()
	if not os.path.exists(data_dir_path + "/" + save_group_name + "/" + ClassName):
		os.makedirs(data_dir_path + "/" + save_group_name + "/" + ClassName)
	Path = data_dir_path + '/' + save_group_name + "/" + ClassName + "/" + filename  
	cv2.imwrite(Path, image)

def LoadAndSave():
	if not os.path.exists(data_dir_path + "/" + save_group_name):
		os.makedirs(data_dir_path + "/" + save_group_name)
	start = time.time()
	
	augmented_list = []
	while(True):
		category_dirs = os.listdir(data_dir_path + "/" + load_group_name)    
		
		if len(augmented_list) >= folder_number_for_auto:
			break
		else:
			for dir in category_dirs:
				if os.path.isdir(data_dir_path + "/" + load_group_name+"/"+dir):					
					if dir in augmented_list:
						time.sleep(0.5)
						pass
					else: 
						augmented_list.append(dir)
						image = {}
						des = {}
						labels=[]
						image_files = []
						angle = [0,90,180,270]

						for category_dir in category_dirs:                
							for rgb_image_path in glob.glob(data_dir_path + "/" + load_group_name + "/" + category_dir):
								for filename in os.listdir(data_dir_path + "/" + load_group_name + "/" + category_dir):
									if (filename[-4:] == '.png'):
										Path = rgb_image_path+ "/" + filename  
										image = cv2.imread(Path)
										print(filename)					
										original_height, original_width = image.shape[:2]
										for r in range(0,4):	
											### make square images
											pasted_image = paste_on_black(image)
											rotated_image = ndimage.rotate(pasted_image, angle[r], reshape=False) 
											removed_image = Remove_black(rotated_image, original_height, original_width, angle[r])
											label = filename.rstrip('.png')
											number = str(angle[r])
											savename=label+ "_" + "{}".format(number) + ".png"
											SaveFile(savename, removed_image, rgb_image_path)

		end = time.time()
		timecost = end -start
		print "timecost" +str(timecost)

if __name__== "__main__":
	LoadAndSave()