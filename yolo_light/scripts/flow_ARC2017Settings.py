#! /usr/bin/env python
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

from net.build import TFNet
from tensorflow import flags
import tensorflow as tf
import os

flags.DEFINE_string("test", "", "path to testing directory")
flags.DEFINE_string("test_output","","path to the output of test data")
flags.DEFINE_string("binary", "", "path to .weights directory")
flags.DEFINE_string("config", "./cfg/", "path to .cfg directory")
flags.DEFINE_string("dataset", "", "path to dataset directory")
flags.DEFINE_string("annotation", "", "path to annotation directory")
flags.DEFINE_string("test_accuracy_dataset","","path to test data directory for testing accuracy")
flags.DEFINE_string("test_accuracy_annotation","","path to test annotation directory for testing accuracy")
flags.DEFINE_string("backup", "", "path to backup folder")
flags.DEFINE_string("label","./labels.txt", "path to classes label")
flags.DEFINE_string("summary", "./summary/", "path to TensorBoard summaries directory")
flags.DEFINE_float("threshold", 0.5, "detection threshold")
flags.DEFINE_float("threshold_max",1.0,"max detection threshold")
flags.DEFINE_string("model", "", "configuration of choice")
flags.DEFINE_string("log","","Simple log file output path")
flags.DEFINE_string("trainer","adam","training algorithm")
flags.DEFINE_float("momentum", 0.0, "applicable for rmsprop and momentum optimizers")
flags.DEFINE_boolean("verbalise", True, "say out loud while building graph")
flags.DEFINE_boolean("train", False, "train the whole net")
flags.DEFINE_string("load", "", "how to initialize the net? Either from .weights or a checkpoint, or even from scratch")
flags.DEFINE_boolean("savepb", False, "save net and weight to a .pb file")
flags.DEFINE_float("gpu", 0.8, "how much gpu (from 0.0 to 1.0)")
flags.DEFINE_float("lr", 0.001, "learning rate")
flags.DEFINE_integer("seed",1234,"Random Seed for training")
flags.DEFINE_float("noiselevel",0,"Gaussian Noise for training")
flags.DEFINE_integer("blur",0,"The kernel size of blur for training")
flags.DEFINE_float("gamma",0,"The gamma adjust for training")
flags.DEFINE_float("saturation",0,"The saturation value factor adjust for training")
flags.DEFINE_float("exposure",0,"The exposure value factor adjust for training")
flags.DEFINE_integer("keep",10,"Number of most recent training results to save")
flags.DEFINE_integer("batch", 20, "batch size")
flags.DEFINE_integer("epoch", 5000, "number of epoch")
flags.DEFINE_integer("save", 15000, "save checkpoint every ? training examples")
flags.DEFINE_string("demo", '', "demo on webcam")
flags.DEFINE_boolean("profile", False, "profile")
flags.DEFINE_boolean("json", False, "Outputs bounding box information in json format.")
flags.DEFINE_boolean("saveVideo", False, "Records video from input video or camera")
flags.DEFINE_string("use_gpu_num","","Use pointed GPU")
flags.DEFINE_boolean("show_device_info",False,"Show device information")
flags.DEFINE_boolean("pred_grasp",False,"Use with GraspWithYolo")
flags.DEFINE_float("pred_grasp_crop_percent",1.0,"The image percent feed to GraspWithYolo")


def checkPathOrDie(path):
    if not os.path.exists(path):
        print (path +  ' does not exists!')
        raise ValueError

def get_right_path(path):
    if path.startswith("./") or path.startswith("../"):
        process_folder_path = os.path.split(os.path.realpath(__file__))[0]
        path = os.path.realpath(os.path.join(process_folder_path,path))
        return path
    else:
        return path

FLAGS = flags.FLAGS
FLAGS.label = get_right_path(FLAGS.label)
checkPathOrDie(FLAGS.label)
checkPathOrDie(FLAGS.config)
if FLAGS.train == True: checkPathOrDie(FLAGS.dataset)
if FLAGS.train == True: checkPathOrDie(FLAGS.annotation)
if FLAGS.train == True: checkPathOrDie(FLAGS.test_accuracy_dataset)
if FLAGS.train == True: checkPathOrDie(FLAGS.test_accuracy_annotation)
if FLAGS.test != "": checkPathOrDie(FLAGS.test)
if FLAGS.binary != "":
    checkPathOrDie(FLAGS.binary)

# make sure all necessary dirs exist
def get_dir(dirs):
	for d in dirs:
		this = os.path.abspath(os.path.join(os.path.curdir, d))
		if not os.path.exists(this): os.makedirs(this)
get_dir([FLAGS.test, FLAGS.binary, FLAGS.backup, os.path.join(FLAGS.test,'out'), FLAGS.summary])

# fix FLAGS.load to appropriate type
try: FLAGS.load = int(FLAGS.load)
except: pass

tfnet = TFNet(FLAGS)

if FLAGS.profile:
	tfnet.framework.profile(tfnet)
	exit()

if FLAGS.demo:
	tfnet.camera(FLAGS.demo, FLAGS.saveVideo)
	exit()

if FLAGS.train:
    print('Enter training ...'); tfnet.train()
    if not FLAGS.savepb: exit('Training finished')

if FLAGS.savepb:
	print('Rebuild a constant version ...')
	tfnet.savepb(); exit('Done')

tfnet.predict()
