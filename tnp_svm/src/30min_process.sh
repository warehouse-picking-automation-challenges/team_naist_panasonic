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

##### resize 480*480
python resize.py

# Maintained by Marcus Gall marcus.gall.lw3@is.naist.jp
# Script to remove black background from object using 
# histogram bl&wh thresholds and watershed region growing

# variable definitions
myHome=/root/catkin_ws/src/tnp/tnp_svm/script
imgSourceDir=/root/catkin_ws/src/tnp/tnp_svm/script/data/competition_test_0721

#folder structure setup
mkdir -p $myHome/auto-crop-img
mkdir -p $myHome/shapes-img
mkdir -p $myHome/bgr-img

cd $imgSourceDir

for dir in */ ; do
	echo "Processing $dir..."
	mkdir -p $myHome/auto-crop-img/$dir
	mkdir -p $myHome/shapes-img/$dir
	mkdir -p $myHome/bgr-img/$dir

	echo "Auto crop objects..."
	cd $imgSourceDir/$dir
	for img in *.png; do	
		convert $img -trim  $myHome/auto-crop-img/$dir/crop_$img
	done

	echo "Calculating object's shape..."
	cd $myHome/auto-crop-img/$dir

	for img in *.png; do
		if  [[ $img == crop_* ]]; then
			base_name=${img#crop_}	
			base_name=${base_name%.png}

			convert $img \
			-blur 0x.5 \
			-normalize \
			-median 10 \
			-level 1%,6% \
			-morphology Close:2 Disk \
			-bordercolor black -border 1x1 \
		    -fill none \
			-fuzz 3% \
			-draw 'matte 0,0 floodfill' \
			-fuzz 99% -fill \#FFFFFF -opaque \#000000 \
			../../shapes-img/$dir/shape_$base_name.png; \
		fi
	done

	echo "Overlying object's with its shape..."
	cd $myHome/shapes-img/$dir

	for sh_img in *.png; do
		if  [[ $sh_img == shape_* ]]; then
			base_name=${sh_img#shape_}
			base_name=${base_name%.png}	
			convert $sh_img ../../auto-crop-img/$dir/crop_$base_name.png  $sh_img -compose multiply -composite ../../bgr-img/$dir/bgr_$base_name.png
		fi
	done
		
	echo "Applying advanced auto cut..."
	cd $myHome/bgr-img

	for img in *.png; do
		if  [[ $img == bgr_* ]]; then
			convert $img -crop \
			    `convert $img -virtual-pixel edge -blur 0x15 -fuzz 15% \
				     -trim -format '%wx%h%O' info:`   +repage   $img		
		fi
	done
		
done

cd ../bgr-img

for directory in * ; do
	if  [[ $directory == * ]]; then
		echo $directory
		cd ${directory}
		for img in *.png; do
			echo $img
			if  [[ $img == * ]]; then
				cd ../../../
				mkdir ./data/out
				mkdir -p ./data/out/$directory
				./BackgroundRemoverBBoxExtractor ./data/bgr-img/$directory/$img ./out/$directory/rectified_$img 4 10 60 0.98	
				cd data/bgr-img/$directory
			fi
		done
		cd ../
	fi
done
