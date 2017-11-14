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

extension_file1=".xml"
extension_file2=".png"

folder1 = "/root/share/mountplace/40classesReal_Bin_New0715/data/annotation-train"
folder2 = "/root/share/mountplace/40classesReal_Bin_New0715/data/images-train"
#folder1 = "/root/share/mountplace/40classesReal_Tote_New0715/data/annotation-train"
#folder2 = "/root/share/mountplace/40classesReal_Tote_New0715/data/images-train"

folder1_files = os.listdir(folder1)
folder2_files = os.listdir(folder2)
notusedfolder1 = folder1+"/../notusedfolder1"
notusedfolder2 = folder2+"/../notusedfolder2"
for file in folder1_files:
    folder2_correspondingname = file.replace(".xml",".png")
    if folder2_correspondingname not in folder2_files:
        if not os.path.isdir(notusedfolder1):
            os.makedirs(notusedfolder1)
        os.rename(folder1+"/"+file,notusedfolder1+"/"+file)
        print folder2_correspondingname
    else:
        folder2_files.remove(folder2_correspondingname)

if len(folder2_files) != 0:
    for file in folder2_files:
        folder1_correspondingname = file.replace(".png",".xml")
        if folder1_correspondingname not in folder1_files:
            if not os.path.isdir(notusedfolder2):
                os.makedirs(notusedfolder2)
            os.rename(folder2+"/"+file,notusedfolder2+"/"+file)
            print folder1_correspondingname
        else:
            folder2_files.remove(folder1_correspondingname)

if len(folder2_files) == 0:
    print("Now your files should be match to each other!")
else:
    print("Caution: Some file might not with the extensions you pointed. Please check your settings.")
