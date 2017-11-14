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
import os,sys,math
from IPython.terminal.debugger import set_trace as keyboard
import numpy as np

def get_filename(xmlpath):
    with open(xmlpath,"r") as file:
        lines = file.readlines()
    for line in lines:
        if "<filename>" in line:
            filename = line.replace("\t","").replace("\n","").replace("\r","").replace("<filename>","").replace("</filename>","")

    return filename


def get_annotation(xmlpath):
    with open(xmlpath,"r") as file:
        lines = file.readlines()

        annotation_count=0
        object_index_list=[]
        line_read=0
        for line in lines:
                if "<object>" in line:
                        annotation_count += 1
                        object_index_list.append(line_read)
                line_read+=1

        annotation=[]
        for i in range(annotation_count):
                if i == (annotation_count - 1 ):
                        lines_view = lines[object_index_list[i]:]
                else:
                        lines_view = lines[object_index_list[i]:object_index_list[i+1]]

                for line in lines_view:
                    if "<xmin>" in line:
                        xmin = int(line.replace("\t","").replace("\n","").replace("\r","").replace("<xmin>","").replace("</xmin>",""))
                    elif "<xmax>" in line:
                        xmax = int(line.replace("\t","").replace("\n","").replace("\r","").replace("<xmax>","").replace("</xmax>",""))
                    elif "<ymin>" in line:
                        ymin = int(line.replace("\t","").replace("\n","").replace("\r","").replace("<ymin>","").replace("</ymin>",""))
                    elif "<ymax>" in line:
                        ymax = int(line.replace("\t","").replace("\n","").replace("\r","").replace("<ymax>","").replace("</ymax>",""))
                    elif "<name>" in line:
                        name = line.replace("\t","").replace("\n","").replace("\r","").replace("<name>","").replace("</name>","")
                annotation.append([xmin,xmax,ymin,ymax,name])
    return annotation

def get_info(xmlpath):
    filename = get_filename(xmlpath)
    annotation = get_annotation(xmlpath)
    return filename, annotation

def crop_image_by_annotation(img,annotation,percent=1.0):
    img = cv2.imread(img)
    max_x, max_y, channel = img.shape

    ymin,ymax,xmin,xmax,_ = annotation
    x_center = int((xmax+xmin)/2)
    y_center = int((ymax+ymin)/2)
    width = xmax-xmin
    height = ymax-ymin

    modified_width = int(width*percent)
    modified_height = int(height*percent)
    modified_xmin = int(x_center-(modified_width/2))
    modified_ymin = int(y_center-(modified_height/2))
    crop_x=[modified_xmin,modified_xmin+modified_width]
    crop_y=[modified_ymin,modified_ymin+modified_height]
    crop_img_shape=(crop_x[1]-crop_x[0],crop_y[1]-crop_y[0],channel)

    c_w =0
    c_h =0
    crop_img = np.zeros(crop_img_shape, np.uint8)
    for w in xrange(modified_xmin,modified_xmin+modified_width-1):
        for h in xrange(modified_ymin,modified_ymin+modified_height-1):
            if w < 0 or h < 0:
                crop_img[c_w,c_h] = (0,0,0)
            else:
                if w < max_x and h <max_y and w >=0 and h>=0:
                    crop_img[c_w,c_h] = img[w,h]
                else:
                    crop_img[c_w,c_h] = (0,0,0)
            c_h +=1
        c_h = 0
        c_w +=1
    
    return crop_img

def draw_box(img,annotation,percent=1.0):
    img = cv2.imread(img)
    max_x, max_y, _ = img.shape

    xmin,xmax,ymin,ymax,_ = annotation
    x_center = int((xmax+xmin)/2)
    y_center = int((ymax+ymin)/2)
    width = xmax-xmin
    height = ymax-ymin

    modified_width = int(width*percent)
    modified_height = int(height*percent)
    print("xmin {0:03d}".format(xmin) + " modified_xmin {0:03d}".format(modified_xmin))
    print("ymin {0:03d}".format(ymin) + " modified_ymin {0:03d}".format(modified_ymin))
    print("xmax {0:03d}".format(xmax) + " modified_xmax {0:03d}".format(modified_xmax))
    print("ymax {0:03d}".format(ymax) + " modified_ymax {0:03d}".format(modified_ymax))
    img = cv2.rectangle(img.copy(),(modified_xmin,modified_ymin),(modified_xmax,modified_ymax) ,(0, 255, 255), 3)
    return img


if __name__ == "__main__":
    argv = sys.argv
    if len(argv) != 5:
        print("image_cropper.py [XMLfolder] [Imgfolder] [OutputFolder] [Crop Percent]")
        exit()
    xmlfolder = argv[1]+"/"
    imgfolder = argv[2]+"/"
    cropped_img_folder = argv[3]+"/"
    crop_percentage=float(argv[4])

    if os.path.isdir(cropped_img_folder) == False:
        os.makedirs(cropped_img_folder)

    classes = []
    classes_amount = []
    filelist = os.listdir(xmlfolder)
    filelist.sort()
    for file in filelist:
        if file.endswith(".xml"):
            print(file)
            filepath = os.path.join(xmlfolder,file)
            imgfilename,annos = get_info(filepath)
            imgpath = os.path.join(imgfolder,imgfilename)
            print(imgpath)
            for anno in annos:
                try:
                    img_crop = crop_image_by_annotation(imgpath,anno,percent=crop_percentage)
                    if anno[4] not in classes:
                        classes.append(anno[4])
                        classes_amount.append(0)
                    number = classes_amount[classes.index(anno[4])]
                    cv2.imwrite(cropped_img_folder+anno[4]+"-{0:03d}.png".format(number),img_crop)
                    classes_amount[classes.index(anno[4])] = number + 1
                except:
                    pass
