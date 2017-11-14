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

from PIL import Image,ImageDraw
import math
import numpy as np


def draw_image_PIL(self,image,grasp,color=(255,0,0),radius=2,linethick=1,normalize=True):
    orig_height = np.asarray(image).shape[0]
    orig_width = np.asarray(image).shape[1]

    if normalize==True:
        image = image.resize((224,224))
        x = int(self._remap(grasp[0],0.9,-0.9,1,0)*224)
        y = int(self._remap(grasp[1],0.9,-0.9,1,0)*224)
        sin2rad = self._remap(grasp[2],0.9,-0.9,1.0,-1.0)
        cos2rad = self._remap(grasp[3],0.9,-0.9,1.0,-1.0)
    else:
        x = int(grasp[0])
        y = int(grasp[1])
        sin2rad = grasp[2]
        cos2rad = grasp[3]

    draw = ImageDraw.Draw(image)

    #Draw point
    draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=color,outline=color)


    twice_rad = math.atan2(cos2rad,sin2rad)
    rad = twice_rad / 2.0


    if len(grasp)>2:
        if normalize==True:
            width = self._remap(grasp[4],0.9,-0.9,224,0)
        else:
            width = grasp[4]
        p2 = (int(x+(width/2)*math.sin(rad)),int(y+(width/2)*math.cos(rad)))
        p1 = (int(x-(width/2)*math.sin(rad)),int(y-(width/2)*math.cos(rad)))
        draw.line((p2,p1),color,linethick)
    del draw
    return image


def draw_image_PIL_Rad(self,image,grasp,color=(255,0,0),radius=2,linethick=1,normalize=True):
    orig_height = np.asarray(image).shape[0]
    orig_width = np.asarray(image).shape[1]

    if normalize==True:
        image = image.resize((224,224))
        x = int(self._remap(grasp[0],0.9,-0.9,1,0)*224)
        y = int(self._remap(grasp[1],0.9,-0.9,1,0)*224)
        rad = self._remap(grasp[2],0.9,-0.9,3.14,-3.14)
    else:
        x = int(grasp[0])
        y = int(grasp[1])
        rad = grasp[2]

    draw = ImageDraw.Draw(image)

    #Draw point
    draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=color,outline=color)

    if len(grasp)>2:
        if normalize==True:
            width = self._remap(grasp[3],0.9,-0.9,224,0)
        else:
            width = grasp[3]
        p2 = (int(x+(width/2)*math.sin(rad)),int(y+(width/2)*math.cos(rad)))
        p1 = (int(x-(width/2)*math.sin(rad)),int(y-(width/2)*math.cos(rad)))
        draw.line((p2,p1),color,linethick)
    del draw
    return image
