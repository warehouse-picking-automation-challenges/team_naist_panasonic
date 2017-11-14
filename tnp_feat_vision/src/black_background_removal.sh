#!/bin/bash -x

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

# variable definitions
myHome=~/sharedfiles_linked/tnp_feature_vision/color_histogram
imgSourceDir=~/sharedfiles_linked/amazon

#folder structure setup
mkdir -p $myHome/auto_crop_img
mkdir -p $myHome/shapes_img
mkdir -p $myHome/bgr_img

# remove not used items in the competition
# lowercase folders
echo "Name To lowercase source folders"
cd $imgSourceDir
for amaDir in */ ; do
	if [[ $amaDir != $(echo $amaDir | tr '[A-Z]' '[a-z]') ]]; then
		mv $amaDir $(echo $amaDir | tr '[A-Z]' '[a-z]')
	fi
done

echo "Removing precalculated items, which are not used in the competition"
cd $myHome/bgr_img
for dir in */ ; do
	amazonfolder=$imgSourceDir/$dir
	if [ ! -d "$amazonfolder" ]; then
		echo "- Removing bgr_img/$dir"
		rm -r $dir
	fi;
done

echo "Start picture processing..."
cd $imgSourceDir
for dir in */ ; do
	if [[ ! -d $myHome/bgr_img/$dir ]]; then

		echo "Processing $dir..."
		mkdir -p $myHome/auto_crop_img/$dir
		mkdir -p $myHome/shapes_img/$dir
		mkdir -p $myHome/bgr_img/$dir

		echo "Convert to png"
		cd $imgSourceDir/$dir
		for img in *.JPG; do
			mogrify -format png $img
		done
		for img in *.jpg; do
			mogrify -format png $img 
		done

		echo "Resizing..."
		for img in *.png; do convert $img \
        	-resize 1080 \
		crop_$img; done

		echo "Auto crop objects..."
		cd $imgSourceDir/$dir
		for img in *.png; do	
			convert $img -trim  $myHome/auto_crop_img/$dir/crop_$img
		done		

		echo "Calculating object's shape..."
		cd $myHome/auto_crop_img/$dir

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
				../../shapes_img/$dir/shape_$base_name.png; \
			fi
		done

		echo "Overlying object's with its shape..."
		cd $myHome/shapes_img/$dir

		for sh_img in *.png; do
			if  [[ $sh_img == shape_* ]]; then
				base_name=${sh_img#shape_}
				base_name=${base_name%.png}	
				convert $sh_img ../../auto_crop_img/$dir/crop_$base_name.png  $sh_img -compose multiply -composite ../../bgr_img/$dir/bgr_$base_name.png
			fi
		done
			
		echo "Applying advanced auto cut..."
		cd $myHome/bgr_img

		for img in *.png; do
			if  [[ $img == bgr_* ]]; then
				convert $img -crop \
				    `convert $img -virtual-pixel edge -blur 0x15 -fuzz 15% \
					     -trim -format '%wx%h%O' info:`   +repage   $img		
			fi
		done
			
	fi
done