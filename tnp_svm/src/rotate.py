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

Generate_folder = "./rotated_bgr-img/"
group_name = "0719out"

def getArgs():
    parser = argparse.ArgumentParser()

    parser.add_argument("--codebook-size", type=int, default=15, help="codebook size")
    parser.add_argument("--data-dir", choices=["./data", "./data_group", "./data_amazon"], type=str, default="./data", help="choose data directory")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--kernel", choices=["linear", "rbf"], type=str, default="linear", help="kernel functions")
    group.add_argument("--linear-svm", action="store_true", default=False, help="use LinearSVM")

    return parser.parse_args()

def Rotate(image, degree):
	height, width = image.shape[:2]
	rotationMatrix = cv2.getRotationMatrix2D((width/2, height/2), degree, 1.0)    
	rotated_image = cv2.warpAffine(image, rotationMatrix, (width, height))####, flags=cv2.INTER_LINEAR)
	return rotated_image

def LoadAndSave():
	args = getArgs()
	group_dir = group_name
	start = time.time()
	category_dirs = os.listdir(os.path.join(args.data_dir, group_dir))    
	image = {}
	des = {}
	labels=[]
	image_files = []
	degree = [0,5,85,90,95,175,180,185,265,270,275,355]
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(os.path.join(args.data_dir, group_dir, category_dir)):
			for filename in os.listdir(os.path.join(args.data_dir, group_dir, category_dir)):
				if (filename[-4:] == '.png'):
					#load amazon images                
					Path = rgb_image_path+ "/" + filename  
					image = cv2.imread(Path)
					print(filename)					
					for r in range(0,12):
						image = image.ROTATE_270
						cv2.imshow("image", image)
						cv2.waitKey(10000)
						label = filename.rstrip('.png')
						number = str(degree[r])
						savename=label+ "_" + "{}".format(number) + ".png"
						Imagefolder = Generate_folder+ rgb_image_path + "/" 
						Imagepath = Imagefolder + savename
						if not os.path.isdir(Imagefolder):
						    os.makedirs(Imagefolder)
						cv2.imwrite(Imagepath,image)

if __name__== "__main__":
	LoadAndSave()