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

import os
import tensorflow as tf
from tensorflow import flags
from .networks.build import Net
from .networks.utils import *
import numpy as np
from PIL import Image
import math
from IPython.terminal.debugger import set_trace as keyboard

class GraspWithYolo:
    def __init__(self,backup,modelname,load=-1,gpu="cpu"):
        print("Initializing GraspWithYolo...")
        options_for_GraspWithYolo = {"train":False, "backup":backup, "name":modelname,"load":load,"gpu_mode":gpu}
        self.net = Net(options_for_GraspWithYolo)
        print("GraspWithYolo Initialized")

    def predict(self,cropped_image):
        """
        cropped_image: Image , input with PIL image or numpy.ndarray
        """
        image = cropped_image
        output_from_net = self.net.predict(image)

        rad = output_from_net[2]
        width = output_from_net[3]
        pred = [output_from_net[0],output_from_net[1],rad,width]
        return pred
