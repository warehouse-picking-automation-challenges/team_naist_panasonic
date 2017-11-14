'''
This Python 3 script needs three specific subfolders to run:
    ./dataset-moc: Input images in img format.
    ./annotation-moc: Input annotations in MOC/TXT format.
    ./annotation-voc: Output annotations in VOC/XML format.
'''
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

from IPython.core.debugger import Tracer; keyboard = Tracer()
import glob
import os,re,sys
import csv

from PIL import Image

datamoc_dir = '/home/tnp/DataARC2017/dataset-moc/' #Training Images Folder for Input
annomoc_dir = '/home/tnp/DataARC2017/annotation-moc/' #Annotation XmlFiles Folder for Input
annovoc_dir = '/home/tnp/DataARC2017/annotation-voc/' #Annotation RawFiles Folder for Output


def sort_nicely(list):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    list.sort( key=alphanum_key )


extensions = ('jpg', 'JPG', 'jpeg', 'JPEG', 'bmp', 'BMP', 'png', 'PNG')
width = 0
height = 0
depth = 'Unspecified'

if not os.path.isdir(annovoc_dir):
    os.makedirs(annovoc_dir)

for root,dirs,files in os.walk(datamoc_dir):
    if root.startswith(annovoc_dir):
        continue
    sort_nicely(files)

    """
    for d in dirs:
        pass
    """

    for imgfile in files:
        head = root
        tail = imgfile
        base, ext = os.path.splitext(tail)
        if ext.strip(".") not in extensions:
            continue

        img = Image.open(os.path.join(root,imgfile))
        print str(imgfile)

        width, height = img.size

        if img.mode == 'RGB':
            depth = '3'
        if img.mode == 'L':
            depth = '1'

        voc = open(annovoc_dir + base + '.xml', 'w')

        line = '<annotation>\n'
        voc.write(line)
        line = '\t<folder>' + head + '</folder>\n'
        voc.write(line)
        line = '\t<filename>' + tail + '</filename>\n'
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
        line = '\t\t<depth>' + depth + '</depth>\n'
        voc.write(line)
        line = '\t</size>\n'
        voc.write(line)
        line = '\t<segmented>Unspecified</segmented>\n'
        voc.write(line)

        moc = root.replace(datamoc_dir,annomoc_dir)
        moc = moc + "/"+ base + '.txt'

        with open(moc) as txt:
            reader = csv.reader(txt, delimiter=' ')
            for obj in reader:

                line = '\t<object>\n'
                voc.write(line)
                line = '\t\t<name>' + obj[0] + '</name>\n'
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
                line = '\t\t\t<xmin>' + obj[4] + '</xmin>\n'
                voc.write(line)
                line = '\t\t\t<ymin>' + obj[5] + '</ymin>\n'
                voc.write(line)
                line = '\t\t\t<xmax>' + obj[6] + '</xmax>\n'
                voc.write(line)
                line = '\t\t\t<ymax>' + obj[7] + '</ymax>\n'
                voc.write(line)

                line = '\t\t</bndbox>\n'
                voc.write(line)
                line = '\t</object>\n'
                voc.write(line)

        line = '</annotation>\n'
        voc.write(line)
