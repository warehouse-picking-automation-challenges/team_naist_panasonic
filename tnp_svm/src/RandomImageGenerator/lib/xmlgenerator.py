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

def prepareXML(xml_path,filename,width,height,depth=3):
    with open(xml_path, 'w') as voc:
        line = '<annotation>\n'
        voc.write(line)
        line = '\t<filename>' + filename + '</filename>\n'
        voc.write(line)
        line = '\t<source>\n'
        voc.write(line)
        line = '\t\t<database>Unknown</database>\n'
        voc.write(line)
        line = '\t</source>\n'
        voc.write(line)
        line = '\t<size>\n'
        voc.write(line)
        line = '\t\t<width>' + str(width) + '</width>\n'
        voc.write(line)
        line = '\t\t<height>' + str(height) + '</height>\n'
        voc.write(line)
        line = '\t\t<depth>' + str(depth) + '</depth>\n'
        voc.write(line)
        line = '\t</size>\n'
        voc.write(line)
        line = '\t<segmented>Unspecified</segmented>\n'
        voc.write(line)

def writeXML(xml_path,objname,anno):
    xmin = anno[0]
    xmax = anno[1]
    ymin = anno[2]
    ymax = anno[3]
    with open(xml_path,"a") as voc:
        line = '\t<object>\n'
        voc.write(line)
        line = '\t\t<name>' + objname + '</name>\n'
        voc.write(line)
        line = '\t\t<pose>Unspecified</pose>\n'
        voc.write(line)
        line = '\t\t<truncated>Unspecified</truncated>\n'
        voc.write(line)
        line = '\t\t<difficult>Unspecified</difficult>\n'
        voc.write(line)
        line = '\t\t<bndbox>\n'
        voc.write(line)

        # Y positive down in MOC, same as VOC.
        line = '\t\t\t<xmin>' + str(xmin) + '</xmin>\n'
        voc.write(line)
        line = '\t\t\t<ymin>' + str(ymin) + '</ymin>\n'
        voc.write(line)
        line = '\t\t\t<xmax>' + str(xmax) + '</xmax>\n'
        voc.write(line)
        line = '\t\t\t<ymax>' + str(ymax) + '</ymax>\n'
        voc.write(line)

        line = '\t\t</bndbox>\n'
        voc.write(line)
        line = '\t</object>\n'
        voc.write(line)
    return True

def FinishXML(xml_path):
    with open(xml_path,"a") as voc:
        line = '</annotation>\n'
        voc.write(line)
    return True

