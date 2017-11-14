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

import subprocess
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

def getArgs():
    parser = argparse.ArgumentParser()

    parser.add_argument("--codebook-size", type=int, default=15, help="codebook size")
    parser.add_argument("--data-dir", choices=["./data", "./data_group", "./data_amazon"], type=str, default="./data", help="choose data directory")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--kernel", choices=["linear", "rbf"], type=str, default="linear", help="kernel functions")
    group.add_argument("--linear-svm", action="store_true", default=False, help="use LinearSVM")

    return parser.parse_args()

def PCA():
	args = getArgs()
	group_dir = "bgr-img"
	start = time.time()
	category_dirs = os.listdir(os.path.join(args.data_dir, group_dir))
	
	for category_dir in category_dirs:                
		for rgb_image_path in glob.glob(os.path.join(args.data_dir, group_dir, category_dir)):
			for file in os.listdir(os.path.join(args.data_dir, group_dir, category_dir)):
				print("FileName=%s" % (file))
				output_name = "rectified_" + file
				cmd = './BackgroundRemoverBBoxExtractor ./file ./output_name 30 60 0.98' 
				subprocess.check_call(cmd.split(" "))

if __name__== "__main__":
    PCA()