#!/bin/bash
# in docker. first param $1

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

echo "Current folder: ${pwd}"
echo "Source folder: $1"
echo "Destination folder $2"

read -n1 -r -p "Okay? (y/n)" key
if [[ "$key" == 'y' ]]; then
	
cp -R $1 $2
for dir in */ ; do
	for

	./BackgroundRemoverBBoxExtractor /root/share/amazon/burts_bees_baby_wipes/Burts_Bees_Baby_Wipes_Bottom_01.png test.png 4 10 60 0.98

rosservice call /tnp_recognition_space/recording_items "item_name: {data: ''}
suction_force: {data: 40}
num_rotations: {data: 3}
num_repetitions: {data: 3}
rotation_angle: {data: 30}"
successful: 
  data: False

done

amazon_data bgr must exist 
# ///////// Files for tnp_svm //////////////////

cd /root/share

# amazon pictures to tnp_svm/amazon_bgr
echo "Copying background-removed AMAZON data from feature_vision to tnp_svm..."
mkdir -p tnp_svm/amazon_bgr
cp -R tnp_feature_vision/color_histogram/amazon_data/* tnp_svm/amazon_bgr
if [ $? -eq 0 ]; then
	echo "OK - copied bgr images."
else
    else echo "ERROR copying bgr images failed." 
fi

# amazon pictures to tnp_svm/amazon_bgr
echo "Copying background-removed OWN data from feature_vision to tnp_svm..."
mkdir -p tnp_svm/own_bgr
cp -R tnp_feature_vision/color_histogram/own_data/* tnp_svm/own_bgr
if [ $? -eq 0 ]; then
	echo "OK - copied bgr images."
else
    else echo "ERROR copying bgr images failed." 
fi

# ///////// Files for svm_second //////////////////

cd /root/share

# item_bounding_boxes to tnp_svm_second/bounding_boxes
echo "Copying training data of item_bounding_boxes to tnp_svm_second..."
mkdir -p tnp_svm_second/bounding_boxes
cp -R item_bounding_boxes/* tnp_svm_second/bounding_boxes
if [ $? -eq 0 ]; then
	echo "OK - copied bounding boxes"
else
	echo "ERROR copying bounding boxes failed." 
fi

# item_weights to tnp_svm_second/weights
echo "Copying training data of item_weights to tnp_svm_second"
mkdir -p tnp_svm_second/weights
cp -R item_weights/* tnp_svm_second/weights
if [ $? -eq 0 ]; then
	echo "OK - copied weights"
else
	echo "ERROR copying weights failed."
fi