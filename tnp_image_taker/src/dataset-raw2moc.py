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

'''
This Python 3 script needs two specific subfolders to run:
    ./dataset-raw: Input images in JPG, PNG or BMP format.
    ./dataset-moc: Output resized/cropped images in PNG format.
'''

import glob
import os,re

from PIL import Image, ImageOps

width = 480
height = 480

extensions = ('jpg', 'JPG', 'jpeg', 'JPEG', 'bmp', 'BMP', 'png', 'PNG')
sources = []

def sort_nicely(list):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    list.sort( key=alphanum_key )


for ext in extensions:
    filelist=glob.glob('dataset-raw/**/*.' + ext)
    sort_nicely(filelist)
    sources.extend(filelist)

i = 1;
ii = len(sources)

for src in sources:
    img = Image.open(src)

    path = os.path.join(*(src.split(os.path.sep)[1:]))
    head, tail = os.path.split(path)
    base, ext = os.path.splitext(tail)

    if not os.path.isdir('dataset-moc/' + head):
        os.makedirs('dataset-moc/' + head)
    dst = ImageOps.fit(img, (width, height), Image.ANTIALIAS)
    dst.save('dataset-moc/' + head + '/' + base + '.png')

    print(str(i) + '/' + str(ii) + ': dataset-moc/' + head + '/' + base + '.png')
    i = i + 1
