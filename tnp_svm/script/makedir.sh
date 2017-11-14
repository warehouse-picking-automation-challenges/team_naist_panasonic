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

# Maintained by Marcus Gall marcus.gall.lw3@is.naist.jp
# Script to remove black background from object using 
# histogram bl&wh thresholds and watershed region growing

#variable definitions
myHome=/root/catkin_ws/src/tnp/tnp_svm/script
imgSourceDir=/root/catkin_ws/src/tnp/tnp_svm/script/data/realdata_origin0720

#folder structure setup
mkdir -p $myHome/data/realdata_origin0720/avery_binder 
mkdir -p $myHome/data/realdata_origin0720/balloons 
mkdir -p $myHome/data/realdata_origin0720/band_aid_tape 
mkdir -p $myHome/data/realdata_origin0720/bath_sponge 
mkdir -p $myHome/data/realdata_origin0720/black_fashion_gloves 
mkdir -p $myHome/data/realdata_origin0720/burts_bees_baby_wipes 
mkdir -p $myHome/data/realdata_origin0720/colgate_toothbrush_4pk 
mkdir -p $myHome/data/realdata_origin0720/composition_book 
mkdir -p $myHome/data/realdata_origin0720/crayons 
mkdir -p $myHome/data/realdata_origin0720/duct_tape 
mkdir -p $myHome/data/realdata_origin0720/epsom_salts 
mkdir -p $myHome/data/realdata_origin0720/expo_eraser 
mkdir -p $myHome/data/realdata_origin0720/fiskars_scissors 
mkdir -p $myHome/data/realdata_origin0720/flashlight 
mkdir -p $myHome/data/realdata_origin0720/glue_sticks 
mkdir -p $myHome/data/realdata_origin0720/hand_weight 
mkdir -p $myHome/data/realdata_origin0720/hanes_socks 
mkdir -p $myHome/data/realdata_origin0720/hinged_ruled_index_cards 
mkdir -p $myHome/data/realdata_origin0720/ice_cube_tray 
mkdir -p $myHome/data/realdata_origin0720/irish_spring_soap 
mkdir -p $myHome/data/realdata_origin0720/laugh_out_loud_jokes 
mkdir -p $myHome/data/realdata_origin0720/marbles 
mkdir -p $myHome/data/realdata_origin0720/measuring_spoons 
mkdir -p $myHome/data/realdata_origin0720/mesh_cup 
mkdir -p $myHome/data/realdata_origin0720/mouse_traps 
mkdir -p $myHome/data/realdata_origin0720/pie_plates 
mkdir -p $myHome/data/realdata_origin0720/plastic_wine_glass 
mkdir -p $myHome/data/realdata_origin0720/poland_spring_water 
mkdir -p $myHome/data/realdata_origin0720/reynolds_wrap 
mkdir -p $myHome/data/realdata_origin0720/robots_dvd 
mkdir -p $myHome/data/realdata_origin0720/robots_everywhere 
mkdir -p $myHome/data/realdata_origin0720/scotch_sponges 
mkdir -p $myHome/data/realdata_origin0720/speed_stick 
mkdir -p $myHome/data/realdata_origin0720/table_cloth 
mkdir -p $myHome/data/realdata_origin0720/tennis_ball_container 
mkdir -p $myHome/data/realdata_origin0720/ticonderoga_pencils 
mkdir -p $myHome/data/realdata_origin0720/tissue_box 
mkdir -p $myHome/data/realdata_origin0720/toilet_brush 
mkdir -p $myHome/data/realdata_origin0720/white_facecloth 
mkdir -p $myHome/data/realdata_origin0720/windex

