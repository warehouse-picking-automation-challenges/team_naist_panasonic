#!/bin/bash
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

FolderRoot="/root/share/mountplace/Yolo9KTest/"
seed=1234

# With Pretrained Weight
python3 ./flow_ARC2017Settings \
                        --model "./cfg/yolo-9000.cfg" \
                        --load "/root/share/mountplace/weights/yolo-9000.weights" \
                        --label "/root/share/mountplace/Yolo9KTest/labels9K.txt" \
                        --batch "50" \
                        --gpu "0.0" \
                        --threshold "0.01" \
                        --use_gpu_num "0" \
                        --show_device_info "False" \
                        --test $FolderRoot"images-outputTest/"
