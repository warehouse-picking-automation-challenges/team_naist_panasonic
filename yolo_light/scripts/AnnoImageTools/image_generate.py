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
from lib.image_generator import *
from lib.xmlgenerator import *
from IPython.terminal.debugger import set_trace as keyboard

print("loading image generator...")
input_width = 480
input_height = 480

#The object images(background removed) folders
item_path = "./items/"

#The background image folder
background_path = "./background/"

#This is the keyword that you want generator to distinguish the classes from filename. For example, you will have {cat, dog} classes from read files like {cat-1.png,dog-1.png, cat-2.png}. If you don't want to seperate you keep split_class_keywords="" will be fine
split_class_keywords="-"

#Output of your generated image and annotation
Generate_folder = "./data/"

#The sample name patterns
seriesname = "Tote_Generated_Test_Data_"


sets = 400 #Generate set of batch
n_samples = 100 #Generate samples per batch
n_items=15 #Max objects be placed in one image

#Item Generation Options
delta_hue=0.05 #"Hue" changes in HSV space
delta_sat_scale=0.4 #"Saturation" changes in HSV space
delta_val_scale=0.2 #"Value" changes in HSV space
min_item_scale=0.9 #Min item scaling
max_item_scale=1.2 #Max item scaling
rand_angle=90 #Max rotation angle of object
minimum_crop=0.85 #The minimum crop scale to the background image
range_of_overlay=[[0,480],[80,415]] #The range limitation [[xmin,xmax],[ymin,ymax]], None for no restriction
generate_once=True #This controls if you only want one class be generated once in each image
max_BeOverlaid=0.4 #This value controls not to generate if items be overlaid by other item. The overlaid area rate should never over this scale or it will tring to replace the item to another place that match this value

generator = ImageGenerator(item_path, background_path,split_class=split_class_keywords)

def restrict_value_in_image(anno,width,height):
    xmin = max(anno[0],0)
    xmax = min(anno[1],width)
    ymin = max(anno[2],0)
    ymax = min(anno[3],height)
    return [xmin,xmax,ymin,ymax]

def get_annotation_value(box):
    #classname,box_x,box_y,box_w,box_hs
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

number = 1
for set in range(sets):
    # generate random sample
    x, t = generator.generate_samples(
        n_samples=n_samples,
        n_items=n_items,
        crop_width=input_width,
        crop_height=input_height,
        min_item_scale=min_item_scale,
        max_item_scale=max_item_scale,
        rand_angle=rand_angle,
        minimum_crop=minimum_crop,
        delta_hue=delta_hue,
        delta_sat_scale=delta_sat_scale,
        delta_val_scale=delta_val_scale,
        range_of_overlay=range_of_overlay,
        max_BeOverlaid=max_BeOverlaid,
        generate_once=generate_once
    )
    for i, image in enumerate(x):
        image = np.transpose(image, (1, 2, 0)).copy()
        dataname=seriesname+"{}".format(number)
        imagename = dataname + ".png"
        XMLfolder = Generate_folder + "annotation/"
        XMLpath = XMLfolder+dataname+".xml"
        if not os.path.isdir(XMLfolder):
            os.makedirs(XMLfolder)
        prepareXML(XMLpath,imagename,input_width,input_height)
        width, height, _ = image.shape
        for truth_box in t[i]:
            box_x, box_y, box_w, box_h = truth_box['x']*width, truth_box['y']*height, truth_box['w']*width, truth_box['h']*height
            classname = truth_box['classname']
            box = [classname,box_x,box_y,box_w,box_h]
            anno = get_annotation_value(box)
            anno = restrict_value_in_image(anno,input_width,input_height)
            writeXML(XMLpath,box[0],anno)
        FinishXML(XMLpath)
        image=(image*255).astype('uint8')
        Imagefolder = Generate_folder+"images/"
        Imagepath = Imagefolder + dataname+".png"
        if not os.path.isdir(Imagefolder):
            os.makedirs(Imagefolder)
        cv2.imwrite(Imagepath,image*255)
        number = number+1
    print ("Set "+ str(set+1) + " Done.")
print ("Generate Finished.")
