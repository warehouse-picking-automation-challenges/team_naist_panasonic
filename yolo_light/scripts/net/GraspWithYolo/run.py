import os
import tensorflow as tf
from tensorflow import flags
#from networks import build
from networks.build import Net
from networks.utils import *
import numpy as np


flags.DEFINE_string("images", "./data/images-train/", "path to training images directory")
flags.DEFINE_string("grasp", "./data/grasp-train/", "path to training grasp data directory")
flags.DEFINE_string("images_test", "./data/images-test/", "path to testing images directory")
flags.DEFINE_string("grasp_test","./data/grasp_test/","path to testing grasp data directory")
flags.DEFINE_boolean("train", False, "train the whole net")
flags.DEFINE_string("load", "-1", "load weights from checkpoint")
flags.DEFINE_string("backup", "./backup/", "path to backup folder")
flags.DEFINE_float("lr", 0.001, "learning rate")
flags.DEFINE_integer("seed",1234,"Random Seed for training")
flags.DEFINE_integer("keep",10,"Number of most recent training results to save")
flags.DEFINE_integer("batch", 50, "batch size")
flags.DEFINE_integer("epoch", 5000, "number of epoch")
flags.DEFINE_integer("save", 50, "save checkpoint every ? training examples")
flags.DEFINE_string("name","","Give a name for better manage your trained profiles")
flags.DEFINE_string("gpu_mode","cpu","'cpu','gpu:1','gpu:2'")
FLAGS = flags.FLAGS

try: FLAGS.load = int(FLAGS.load)
except: pass

#Initiate the model
net = Net(FLAGS)
print("Net Initialized")

np.random.seed(FLAGS.seed)
tf.set_random_seed(FLAGS.seed)
if FLAGS.train:
    print('Enter training ...')
    net.train()
    exit('Training finished')

net.predict_test_data("./output/")
