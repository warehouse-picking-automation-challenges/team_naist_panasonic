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

width = 480
height = 480

image_size= 480

angle = 10
codebook_size = 15
target_size = 480
Generate_folder = "./rotate_paste_bgr-img/"
group_name = "0719out"

def getArgs():
    parser = argparse.ArgumentParser()

    parser.add_argument("--codebook-size", type=int, default=15, help="codebook size")
    parser.add_argument("--data-dir", choices=["./data", "./data_group", "./data_amazon"], type=str, default="./data", help="choose data directory")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--kernel", choices=["linear", "rbf"], type=str, default="linear", help="kernel functions")
    group.add_argument("--linear-svm", action="store_true", default=False, help="use LinearSVM")

    return parser.parse_args()

def paste_on_black(image_in):
	height, width = image_in.shape[:2]
	center_h = int(height/2)
	center_w = int(width/2)
	if width >= height:
		background = cv2.imread("./background/blackimage.png")
		background = cv2.resize(background, (width,width))
		background[center_w-height/2:center_w+height/2,0:width] = image_in[0:int(height/2)*2,0:int(width/2)*2]
	else:
		background = cv2.imread("./background/blackimage.png")
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

def LoadAndSave():
	args = getArgs()
	group_dir = group_name
	start = time.time()
	category_dirs = os.listdir(os.path.join(args.data_dir, group_dir))    
	image = {}
	des = {}
	labels=[]
	image_files = []
	rIntr = 90
	rs = 0
	re = 270
	angle = [0,5,85,90,95,175,180,185,265,270,275,355]
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(os.path.join(args.data_dir, group_dir, category_dir)):
			for filename in os.listdir(os.path.join(args.data_dir, group_dir, category_dir)):
				if (filename[-4:] == '.png'):
					#load amazon images                
					Path = rgb_image_path+ "/" + filename  
					image = cv2.imread(Path)
					print(filename)					
					
					original_height, original_width = image.shape[:2]
					for r in range(0,12):	
						pasted_image = paste_on_black(image)
						rotated_image = ndimage.rotate(pasted_image, angle[r], reshape=False) 
						print(angle[r])
						label = filename.rstrip('.png')
						number = str(angle[r])
						savename=label+ "_" + "{}".format(number) + ".png"
						Imagefolder = Generate_folder+ rgb_image_path + "/" 
						Imagepath = Imagefolder + savename
						if not os.path.isdir(Imagefolder):
						    os.makedirs(Imagefolder)
						cv2.imwrite(Imagepath,rotated_image)

if __name__== "__main__":
	LoadAndSave()