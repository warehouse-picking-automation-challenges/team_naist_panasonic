import tensorflow as tf
import math


class Convolution2D:
    """
    A convolution layer for images, with biases

    input shape : [batch, in_height, in_width, in_channels]
    
    name : string. Name of variable scope
    in_channels : integer. Number of input channel
    out_channels : integer. Number of output channel
    ksize : integer. Size of convolution kernel i.e. weight size
    stride : integer. Stride of convolutional operation
    pad : string. Mode to decide padding size. 'SAME' or 'VALID'
    wscale : float. Scaling factor to intitialize weights
    nobias : Boolean. True means the layer dones not have bias
    """
    def __init__(self, name, in_channels, out_channels, ksize, 
                    stride=1, pad='SAME', wscale=1, nobias=False, trainable=True):
        self.name = name
        with tf.variable_scope(self.name):
            self.stride = stride
            self.pad = pad
            self.nobias = nobias
            with tf.name_scope('convolution2d') as scope:
                self.W = tf.Variable( tf.truncated_normal([ksize, ksize, in_channels, out_channels],
                                    stddev=wscale * 1.0/math.sqrt(ksize * ksize * in_channels)) , \
                                    name='weights', trainable=trainable)
                if not self.nobias:
                    self.b = tf.Variable( tf.zeros([out_channels]) , name='biases', trainable=trainable)

    def __call__(self, x):
        with tf.variable_scope(self.name):
            conv = tf.nn.conv2d(x, self.W, strides=[1, self.stride, self.stride, 1], padding=self.pad)
            if not self.nobias:
                return tf.nn.bias_add(conv, self.b)
            else:
                return conv
