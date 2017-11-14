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

targetDir=/sys/bus/pci/drivers/xhci_hcd

lanesOutput=$(ls -la $targetDir | grep -oP "\s\d{4}:\d{2}:\d{2}.\d{1}")

mapfile -t lanes <<< "$lanesOutput"

echo "--- unbind all busses"
for ((i = 0; i < ${#lanes[@]}; i++))
do
       echo -n ${lanes[$i]} | tee $targetDir/unbind
       echo ''
done

echo "--- bind all busses again"
for ((i = 0; i < ${#lanes[@]}; i++))
do
       echo -n ${lanes[$i]} | tee $targetDir/bind
       echo ''
done

# template function, what should be done:
#echo -n "0000:03:00.0" | tee $targetDir/unbind
#echo -n "0000:03:00.0" | tee $targetDir/bind
