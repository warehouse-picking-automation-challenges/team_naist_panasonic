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
import sys,os,glob,re
from scipy.interpolate import UnivariateSpline
from IPython.core.debugger import Tracer; keyboard = Tracer()
"""
This program is used for creating noisy data
"""

dataraw_dir = '/media/tnp/TrainingData/dataset-moc/' #Raw Images Folder for Input
noised_dataraw = '/media/tnp/TrainingData/dataset-noised/'
annovoc_dir = '/media/tnp/TrainingData/annotation-voc/' #Annotation XmlFiles Folder for Input
noised_annovoc_dir = '/media/tnp/TrainingData/annotation-voc-noised/' #Annotation XmlFiles Folder for Output
extensions = ('jpg', 'JPG', 'jpeg', 'JPEG', 'bmp', 'BMP', 'png', 'PNG')
color_temp_list = ("normal","cold","warming") #Always Start from normal or it will failed to apply


def get_annotation(xmlpath):
    with open(xmlpath,"r") as file:
        lines = file.readlines()
    for line in lines:
        if "<xmin>" in line:
            xmin = int(line.replace("\t","").replace("\n","").strip("<xmin>").strip("</xmin>"))

        elif "<xmax>" in line:
            xmax = int(line.replace("\t","").replace("\n","").strip("<xmax>").strip("</xmax>"))
        elif "<ymin>" in line:
            ymin = int(line.replace("\t","").replace("\n","").strip("<ymin>").strip("</ymin>"))
        elif "<ymax>" in line:
            ymax = int(line.replace("\t","").replace("\n","").strip("<ymax>").strip("</ymax>"))
        elif "<width>" in line:
            width = int(line.replace("\t","").replace("\n","").strip("<width>").strip("</width>"))
        elif "<height>" in line:
            height = int(line.replace("\t","").replace("\n","").strip("<height>").strip("</height>"))
        elif "<depth>" in line:
            depth = int(line.replace("\t","").replace("\n","").strip("<depth>").strip("</depth>"))
        elif "<name>" in line:
            name = line.replace("\t","").replace("\n","").strip("<name>").strip("</name>")
    return (xmin,xmax,ymin,ymax,width,height,depth,name)


def writeXML(orig_xml_path,new_xml_path,shifting,extensions):

    xmin,xmax,ymin,ymax,width,height,depth,name = get_annotation(orig_xml_path)
    new_shifted_point = [xmin+shifting[0],xmax+shifting[0],ymin+shifting[1],ymax+shifting[1]]

    head_new, xmlfilename = os.path.split(new_xml_path)

    voc = open(new_xml_path, 'w')

    line = '<annotation>\n'
    voc.write(line)
    line = '\t<folder>' + noised_dataraw + '</folder>\n'
    voc.write(line)
    line = '\t<filename>' + xmlfilename.replace(".xml",extensions) + '</filename>\n'
    voc.write(line)
    line = '\t<source>\n'
    voc.write(line)
    line = '\t\t<database>Unknown</database>\n'
    voc.write(line)
    line = '\t</source>\n'
    voc.write(line)
    line = '\t<size>\n'
    voc.write(line)
    line = '\t\t<width>' + str(width) + '</width>\n'
    voc.write(line)
    line = '\t\t<height>' + str(height) + '</height>\n'
    voc.write(line)
    line = '\t\t<depth>' + str(depth) + '</depth>\n'
    voc.write(line)
    line = '\t</size>\n'
    voc.write(line)
    line = '\t<segmented>Unspecified</segmented>\n'
    voc.write(line)
    line = '\t<object>\n'
    voc.write(line)
    line = '\t\t<name>' + name + '</name>\n'
    voc.write(line)
    line = '\t\t<pose>Unspecified</pose>\n'
    voc.write(line)
    line = '\t\t<truncated>Unspecified</truncated>\n'
    voc.write(line)
    line = '\t\t<difficult>Unspecified</difficult>\n'
    voc.write(line)
    line = '\t\t<bndbox>\n'
    voc.write(line)

    # Y positive down in MOC, same as VOC.
    line = '\t\t\t<xmin>' + str(new_shifted_point[0]) + '</xmin>\n'
    voc.write(line)
    line = '\t\t\t<ymin>' + str(new_shifted_point[2]) + '</ymin>\n'
    voc.write(line)
    line = '\t\t\t<xmax>' + str(new_shifted_point[1]) + '</xmax>\n'
    voc.write(line)
    line = '\t\t\t<ymax>' + str(new_shifted_point[3]) + '</ymax>\n'
    voc.write(line)

    line = '\t\t</bndbox>\n'
    voc.write(line)
    line = '\t</object>\n'
    voc.write(line)

    line = '</annotation>\n'
    voc.write(line)
    return new_shifted_point


def read_image(im):
    if type(im) is not np.ndarray:
        im = cv2.imread(im)
    print "Read image:" + im
    return im

def add_noise(img,seed,level):
    noiselevel = np.sqrt(level)
    np.random.seed(seed)
    gaussian_noise = np.random.normal(0,noiselevel,size = np.shape(img))
    noisy_img = img+gaussian_noise
    return noisy_img

def adjusting_gamma(image,gamma):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
       for i in np.arange(0, 256)]).astype("uint8")

    #apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def create_LUT_8UC1(x, y):
    #keyboard()
    spl = UnivariateSpline(x, y,k=2)
    return spl(xrange(256))

def apply_filter(imagefile,img_bgr_in,filter):
    img_gray = cv2.imread(imagefile,0)
    anchor_x = [0, 128, 255]
    anchor_y = [0, 192, 255]
    myLUT = create_LUT_8UC1(anchor_x, anchor_y)
    img_curved = cv2.LUT(img_gray, myLUT).astype(np.uint8)
    incr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256],
                [0, 70, 140, 210, 256])
    decr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256],
                [0, 30, 80, 120, 192])
    if filter == "warming":
        c_b, c_g, c_r = cv2.split(img_bgr_in)
        c_r = cv2.LUT(c_r, incr_ch_lut).astype(np.uint8)
        c_b = cv2.LUT(c_b, decr_ch_lut).astype(np.uint8)
        img_bgr_warm = cv2.merge((c_b, c_g, c_r))
        c_b = cv2.LUT(c_b, decr_ch_lut).astype(np.uint8)

        # increase color saturation
        c_h, c_s, c_v = cv2.split(cv2.cvtColor(img_bgr_warm,
            cv2.COLOR_BGR2HSV))
        c_s = cv2.LUT(c_s, incr_ch_lut).astype(np.uint8)
        img_bgr_warm = cv2.cvtColor(cv2.merge(
            (c_h, c_s, c_v)),
            cv2.COLOR_HSV2BGR)
        return img_bgr_warm

    elif filter == "cold":
        c_b, c_g, c_r = cv2.split(img_bgr_in)
        c_r = cv2.LUT(c_r, decr_ch_lut).astype(np.uint8)
        c_b = cv2.LUT(c_b, incr_ch_lut).astype(np.uint8)
        img_bgr_cold = cv2.merge((c_b, c_g, c_r))

        # decrease color saturation
        c_h, c_s, c_v = cv2.split(cv2.cvtColor(img_bgr_cold,
            cv2.COLOR_BGR2HSV))
        c_s = cv2.LUT(c_s, decr_ch_lut).astype(np.uint8)
        img_bgr_cold = cv2.cvtColor(cv2.merge(
            (c_h, c_s, c_v)),
            cv2.COLOR_HSV2BGR)
        return img_bgr_cold


def apply_shifting(img,x,y):
    rows,cols,chs = img.shape
    M = np.float32([[1,0,x],[0,1,y]])
    dst = cv2.warpAffine(img,M,(cols,rows))
    return dst


def sort_nicely(list):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    list.sort( key=alphanum_key )

seed_start = 1234

for root,dirs,files in os.walk(dataraw_dir):
    sort_nicely(files)

    for d in dirs:
        noised_data_dir = os.path.join(noised_dataraw+d)
        noised_annovoc_dir = os.path.join(noised_annovoc_dir+d)
        if not os.path.isdir(noised_data_dir):
            os.makedirs(noised_data_dir)
        if not os.path.isdir(noised_annovoc_dir):
            os.makedirs(noised_annovoc_dir)

    for imgfile in files:
        head = root
        tail = imgfile
        base, ext = os.path.splitext(tail)
        if ext.strip(".") not in extensions:
            continue

        orig_xml = annovoc_dir + base + ".xml"

        imagefile=os.path.join(root,imgfile)
        img = read_image(imagefile)
        for x in [-100,-50,-30,0,30,50,100]:
            for y in [-100,-50,-30,0,30,50,100]:
                img_shift = apply_shifting(img,x,y)
                for filteruse in color_temp_list:
                    if filteruse == "normal":
                        img_filted = img_shift
                    else:
                        img_filted = apply_filter(imagefile,img_shift,filteruse)
                    for blurkernel in [0,2,4]:
                        if blurkernel ==0:
                            img_blur = img_filted
                        else:
                            img_blur = cv2.blur(img_filted,(blurkernel,blurkernel))
                        for i in range(3):
                            gamma = 0.9 + 0.9*i
                            shifted_blur_gammaed_img = adjusting_gamma(img_blur,gamma)
                            img_for_gaussian = shifted_blur_gammaed_img/255.
                            for j in range(3):
                                if gamma < 0.1:
                                    level = 0.1
                                else:
                                    level =  0 + j*0.001
                                seed = seed_start + 1
                                shifted_blur_gammaed_noised_img = add_noise(img_for_gaussian,seed,level) # Adding Noise
                                shifted_blur_gammaed_noised_img = np.interp(shifted_blur_gammaed_noised_img,[-0.1,1.1],[0,255])
                                shifted_blur_gammaed_noised_img = np.ndarray.astype(shifted_blur_gammaed_noised_img,'uint8')
                                name = os.path.splitext(imgfile)[0] +"_"+ "shift({},{})_".format(x,y)+ filteruse +"_BlurK={}".format(blurkernel)+"_gamma{}".format(gamma) +"_Nlevel{}".format(level)
                                print name
                                #keyboard()
                                extensions = os.path.splitext(imgfile)[1]
                                cv2.imwrite(noised_dataraw+name+ extensions,shifted_blur_gammaed_noised_img)
                                newanno_path = noised_annovoc_dir + name + ".xml"
                                new_point = writeXML(orig_xml,newanno_path,[x,y],extensions) #Shifting annotation and write to XML file and return shift points in list [xmin,ymin,xmax,ymax]

                                #For Displaying
                                cv2.rectangle(shifted_blur_gammaed_noised_img,(new_point[0],new_point[2]),(new_point[1],new_point[3]),(255,0,0),6)
                                cv2.putText(shifted_blur_gammaed_noised_img,"shift:({},{}) ".format(x,y)+ filteruse +" BlurK={}".format(blurkernel)+ " g={}".format(gamma) + " Nlevel={}".format(level), (10, 30),
                		                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 2)

                                cv2.imshow("shifted_blur_gammaed_noised_img",shifted_blur_gammaed_noised_img)
                                cv2.waitKey(1)
