import tensorflow as tf
from .links import *
from .BaseModel import BaseModel
from .links.Convolution2D import Convolution2D
from .links.BatchNormalization import BatchNormalization
from .links.Linear import Linear

class CNNL(BaseModel):
    """
    This is the 6CNN3L.
    """
    def __init__(self, name, in_channels,seed=1):
        super(CNNL, self).__init__(name,seed)
        with tf.variable_scope(self.name):
            self.conv0 = Convolution2D('conv0',in_channels,64,5,stride=2,nobias=False)
            self.conv1 = Convolution2D('conv1',64,128,3,stride=2,nobias=False)
            self.bn1 = BatchNormalization('bn1',128,dim=3)
            self.conv2 = Convolution2D('conv2',128,128,3,stride=2,nobias=False)
            self.bn2 = BatchNormalization('bn2',128,dim=3)
            self.conv3 = Convolution2D('conv3',128,128,3,stride=2,nobias=False)
            self.bn3 = BatchNormalization('bn3',128,dim=3)
            self.conv4 = Convolution2D('conv4',128,256,3,stride=2,nobias=False)
            self.bn4 = BatchNormalization('bn4',256,dim=3)
            self.l5 = Linear('l5',7*7*256,512,nobias=False)
            self.bn5 = BatchNormalization('bn5',512,dim=1)
            self.l6 = Linear('l6',512,512,nobias=False)
            self.bn6 = BatchNormalization('bn6',512,dim=1)
            self.l7 = Linear('l7',512,5,nobias=False)

    def __call__(self,input,batchsize,base='tanh',train=False):
        if base=='tanh':
            output = self.forward_tanh(input,batchsize,train)
        elif base=='relu':
            output = self.forward_relu(input,batchsize,train)
        return output

    def forward_tanh(self,x,batchsize,train):
        output = tf.nn.tanh(self.conv0(x))
        output = tf.nn.tanh(self.bn1(self.conv1(output),train=train))
        output = tf.nn.tanh(self.bn2(self.conv2(output),train=train))
        output = tf.nn.tanh(self.bn3(self.conv3(output),train=train))
        output = tf.nn.tanh(self.bn4(self.conv4(output),train=train))
        output = tf.reshape(output,[batchsize,7*7*256])
        output = tf.nn.tanh(self.bn5(self.l5(output),train=train))
        output = tf.nn.tanh(self.bn6(self.l6(output),train=train))
        output = tf.nn.tanh(self.l7(output),train=train)
        return output

    def forward_relu(self,x,batchsize,train):
        output = tf.nn.relu(self.conv0(x))
        output = tf.nn.relu(self.bn1(self.conv1(output),train=train))
        output = tf.nn.relu(self.bn2(self.conv2(output),train=train))
        output = tf.nn.relu(self.bn3(self.conv3(output),train=train))
        output = tf.nn.relu(self.bn4(self.conv4(output),train=train))
        output = tf.reshape(output,[batchsize,7*7*256])
        output = tf.nn.tanh(self.bn5(self.l5(output),train=train))
        output = tf.nn.tanh(self.bn6(self.l6(output),train=train))
        output = tf.nn.tanh(self.l7(output))
        return output
