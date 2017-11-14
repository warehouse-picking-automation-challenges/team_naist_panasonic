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

import cv2
import numpy as np
import glob
import os

print("loading image generator...")
input_width = 480
input_height = 480
background_path = "./background/"
Generate_folder = "./out/"
seriesname = "Generated_"

sets = 1
n_samples = 1
n_items=1

delta_hue=0.00
delta_sat_scale=0.0
delta_val_scale=0.0
min_item_scale=1.0
max_item_scale=1.2
rand_angle=10
minimum_crop=1.0

def restrict_value_in_image(anno,width,height):
    xmin = max(anno[0],0)
    xmax = min(anno[1],width)
    ymin = max(anno[2],0)
    ymax = min(anno[3],height)
    return [xmin,xmax,ymin,ymax]

def get_annotation_value(box):
    x = box[1]
    y = box[2]
    w = box[3]
    h = box[4]
    xmin = int(x-w/2)
    xmax = int((x + w/2))
    ymin = int(y-h/2)
    ymax = int((y + h/2))
    return [xmin,xmax,ymin,ymax]

def get_classname(filename):
    classname = filename.split("-")[0]
    return classname

sources = []

sources=glob.glob('items/Training_items/*')

#make output directory
for src in sources:
	print(src)
	path = os.path.join(*(src.split(os.path.sep)[1:]))
	head, tail = os.path.split(path)
	base, ext = os.path.splitext(tail)
	item_path = "./items/Training_items/" + tail + '/'
	number = 1
	for set in range(sets):
		image = cv2.imread(item_path)
		dataname=seriesname+"{}".format(number)
		imagename = dataname + ".png"
		Imagefolder = Generate_folder+"Images/" + head + "/" + tail + "/"
		Imagepath = Imagefolder + dataname+ "_" + base + ".png"
		if not os.path.isdir(Imagefolder):
		    os.makedirs(Imagefolder)
		cv2.imwrite(Imagepath,image)
		number = number+1
	print ("Set "+ str(set+1) + " Done.")
print ("Generate Finished.")

