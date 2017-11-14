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
from scipy.interpolate import UnivariateSpline


def create_LUT_8UC1(x,y):
    spl = UnivariateSpline(x,y,k=2)
    return spl(xrange(256))

def apply_blur(img,kernellevel):
    if kernellevel == 0:
        img_blur = img
    else:
        img_blur = cv2.blur(img,(kernellevel,kernellevel))
    return img_blur

def apply_filter(img_bgr_in,filter):
    img_gray = cv2.cvtColor(img_bgr_in, cv2.COLOR_RGB2GRAY)
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

def adjusting_saturation(img,value):
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    hsv = hsv.astype(np.float64)
    hsv[:,:,1] = hsv[:,:,1]*value
    hsv[:,:,1] = np.clip(hsv[:,:,1],0.0,255.0)
    hsv = hsv.astype(np.uint8)
    image = cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB)
    return image

def adjusting_exposure(img,value):
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    hsv = hsv.astype(np.float64)
    hsv[:,:,2] = hsv[:,:,2]*value
    hsv[:,:,2] = np.clip(hsv[:,:,2],0.0,255.0)
    hsv = hsv.astype(np.uint8)
    image = cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB)
    return image



