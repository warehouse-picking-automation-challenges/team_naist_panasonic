#!/bin/sh
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

# Maintained by Marcus Gall marcus.gall.lw3@is.naist.jp
# Script to remove black background from object using 
# histogram bl&wh thresholds and watershed region growing

#loop directory
cd data/bgr-img

for directory in * ; do
	if  [[ $directory == * ]]; then
		echo $directory
		cd ${directory}
		for img in *.png; do
			echo $img
			if  [[ $img == * ]]; then
				cd ../../../
				mkdir ./out
				mkdir -p ./out/$directory
				./BackgroundRemoverBBoxExtractor ./data/bgr-img/$directory/$img ./out/$directory/rectified_$img 4 10 60 0.98	
				cd data/bgr-img/$directory
			fi
		done
		cd ../
	fi
done
