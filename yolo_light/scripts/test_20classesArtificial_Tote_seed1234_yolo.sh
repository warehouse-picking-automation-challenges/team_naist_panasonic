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

export PYTHONPATH="/usr/local/lib/python3.5/dist-packages:$PYTHONPATH"

FolderRoot="/root/share/mountplace/20classesArtificial_Tote/"
seed=1234
name_of_net="arc2017_20classes_full"
AdditionalDescription="_PretrainedWeight"

ResultRoot=$FolderRoot"TrainingResult/"
ResultFolder=$ResultRoot$name_of_net"_seed"$seed$AdditionalDescription"/"

SummaryFolder=$ResultFolder"summary/"

Ckpt=$ResultFolder"ckpt/"


# With Pretrained Weight
python3 ./flow_ARC2017Settings \
                        --model "./cfg/"$name_of_net".cfg" \
                        --load "-1" \
                        --test $FolderRoot"data/images-outputTest/" \
                        --test_output $ResultFolder"TestOutput" \
                        --label $FolderRoot"labels_20_Formal.txt" \
                        --backup $Ckpt \
                        --batch "2" \
                        --summary $SummaryFolder \
                        --gpu "0.0" \
                        --use_gpu_num "0"
