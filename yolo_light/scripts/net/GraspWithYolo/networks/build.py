import os
import numpy as np
import tensorflow as tf
from . import *
#from .model.CNN3L import CNNL
from .model.CNN4L import CNNL
from . import train,saver,data,draw_grasp,inference
from .utils import normalize_lib
from IPython.terminal.debugger import set_trace as keyboard
import math
import matplotlib.pyplot as plt
from PIL import Image

class dotdict(dict):
    """dot.notation access to dictionary attributes to replace FLAGS when not needed"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class Net(object):

    # imported methods from other pyfile to this class
    build_train_op = train.build_train_op
    loss_function = train.loss_function
    loss_function2 = train.loss_function2
    train = train.train
    test = inference.test
    load_from_ckpt = saver.load_from_ckpt
    save_checkpoint = saver.save_checkpoint
    draw_image_PIL = draw_grasp.draw_image_PIL
    fit_original_image = normalize_lib.fit_original_image
    fit_grasp_to_net = data.fit_grasp_to_net
    fit_grasp_to_net_TF = data.fit_grasp_to_net_TF
    _remap = normalize_lib._remap
    prepare_data_queue = data.prepare_data_queue
    prepare_test_data = data.prepare_test_data
    train_data_process = data.train_data_process
    prepare_csv_data_list = data.prepare_csv_data_list
    draw_image_PIL_Rad = draw_grasp.draw_image_PIL_Rad
    fit_original_image_Rad = normalize_lib.fit_original_image_Rad
    predict = inference.predict
    predict_test_data = inference.predict_test_data
    test_successrate = inference.test_successrate

    def __init__(self,FLAGS):
        if isinstance(FLAGS, dict):
            defaultSettings = { "batch": 20, "train": False, "gpu_mode": "cpu", "summary":None}
            defaultSettings.update(FLAGS)
            FLAGS = dotdict(defaultSettings)

        self.FLAGS = FLAGS
        print("Now using :"+self.FLAGS.gpu_mode)
        cfg = dict({
            'allow_soft_placement': False,
            'log_device_placement': False
        })
        if self.FLAGS.summary is not None:
            self.summary_op = tf.summary.merge_all()
            self.writer = tf.summary.FileWriter(self.FLAGS.summary + 'train')

        if self.FLAGS.gpu_mode == "cpu":
            arg_gpu='/cpu:0'

        else:
            arg_gpu="/gpu:"+str(self.FLAGS.gpu_mode)
            cfg['gpu_options'] = tf.GPUOptions()
            cfg['allow_soft_placement'] = True #If you uncomment this line makes your model always can be built in CPU mode if machine cannot find GPU device
            cfg['log_device_placement'] = True

        config = tf.ConfigProto(**cfg)
        self.arg_gpu = arg_gpu
        print(config)


        with tf.device(arg_gpu):
            self.build_model()
            if self.FLAGS.train:
                self.prepare_csv_data_list()
                self.prepare_data_queue()
                self.prepare_test_data()
                self.build_train_op()
                self.build_test_op()
            self.prepare_sess(config)
            self.build_forward_op()


        self.prepare_saver()

        if self.FLAGS.summary is not None:
            self.writer.add_graph(self.sess.graph)

        if self.FLAGS.load != "":
            self.load_from_ckpt()


    def build_model(self):
        self.model = CNNL(name='6CNN3L',in_channels=3,seed=self.FLAGS.seed)

    def prepare_sess(self,config):
        sess = tf.Session(config = config)
        sess.run(tf.global_variables_initializer())
        tf.train.start_queue_runners(sess=sess)
        self.sess = sess

    def prepare_saver(self):
        self.saver = tf.train.Saver(tf.global_variables(),max_to_keep = self.FLAGS.keep)


    def build_forward_op(self):
        x_image=tf.placeholder(tf.float32,[None,224,224,3],name="input")
        self.x_image = x_image

        #Fowarding reshaped image to model and get grasp prediction
        self.forward_op = self.model(x_image,1,base='relu',train=False)[0]


    def build_test_op(self):
        test_image=tf.placeholder(tf.float32,[None,224,224,3],name="input_test")
        test_grasp=tf.placeholder(tf.float32,[None,4],name="teach_grasp_test")

        #Fowarding reshaped image to model and get grasp prediction
        test_op = self.model(test_image,self.FLAGS.batch,base='relu',train=False)
        self.test_loss = self.loss_function(test_op,test_grasp)
        self.test_op=test_op
        self.test_image_batch = test_image
        self.test_grasp_batch = test_grasp
