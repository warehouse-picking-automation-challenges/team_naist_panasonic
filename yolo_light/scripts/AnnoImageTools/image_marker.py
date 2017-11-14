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
import os,sys
from IPython.terminal.debugger import set_trace as keyboard


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



def draw_box(img,annotation,percent=1.0):
    max_x, max_y, _ = img.shape

    xmin,xmax,ymin,ymax,name = annotation
    x_center = int((xmax+xmin)/2)
    y_center = int((ymax+ymin)/2)
    width = xmax-xmin
    height = ymax-ymin

    modified_width = int(width*percent)
    modified_height = int(height*percent)
    modified_xmin = int(max(x_center-(modified_width/2),0))
    modified_xmax = int(min(x_center+(modified_width/2),max_x))
    modified_ymin = int(max(y_center-(modified_height/2),0))
    modified_ymax = int(min(y_center+(modified_height/2),max_y))
    print("xmin {0:03d}".format(xmin) + " modified_xmin {0:03d}".format(modified_xmin))
    print("ymin {0:03d}".format(ymin) + " modified_ymin {0:03d}".format(modified_ymin))
    print("xmax {0:03d}".format(xmax) + " modified_xmax {0:03d}".format(modified_xmax))
    print("ymax {0:03d}".format(ymax) + " modified_ymax {0:03d}".format(modified_ymax))
    img = cv2.rectangle(img.copy(),(modified_xmin,modified_ymin),(modified_xmax,modified_ymax) ,(0, 255, 255), 3)
    img = cv2.putText(img.copy(), name, (modified_xmin+5, modified_ymin + 15), cv2.FONT_HERSHEY_COMPLEX, 1e-3 * modified_height * 2.5, (0, 0, 255),1)
    return img


if __name__ == "__main__":
    argv = sys.argv
    if len(argv) != 5:
        print("image_cropper.py [XMLfolder] [Imgfolder] [OutputFolder] [Mark Percent]")
        exit()
    xmlfolder = argv[1]+"/"
    imgfolder = argv[2]+"/"
    marked_img_folder = argv[3]+"/"
    percent=float(argv[4])

    if os.path.isdir(marked_img_folder) == False:
        os.makedirs(marked_img_folder)

    classes = []
    classes_amount = []
    filelist = os.listdir(xmlfolder)
    filelist.sort()
    for file in list(filelist):
        if file.endswith(".xml"):
            print(file)
            filepath = os.path.join(xmlfolder,file)
            imgfilename,annos = get_info(filepath)
            imgpath = os.path.join(imgfolder,imgfilename)
            print(imgpath)
            img = cv2.imread(imgpath)
            for anno in annos:
                img = draw_box(img,anno,percent=percent)
            cv2.imwrite(marked_img_folder+imgfilename,img)
