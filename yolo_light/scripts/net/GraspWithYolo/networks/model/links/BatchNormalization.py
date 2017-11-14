#Python3 Version

import tensorflow as tf
import math
import functools


from IPython.terminal.debugger import set_trace as keyboard

class BatchNormalization(object):
    def __init__(self, name, shape, dim, decay=0.9, eps=1e-5, trainable=True):
        self.name = name
        self.shape = shape
        self.dim = dim
        with tf.variable_scope(self.name):
            with tf.name_scope('batchnormalization') as scope:
                self.mean = tf.Variable(tf.constant(0.0, shape=[shape]), name='mean', trainable=False)
                self.variance = tf.Variable(tf.constant(1.0, shape=[shape]), name='variance', trainable=False)
                self.scale = tf.Variable(tf.constant(1.0, shape=[shape]), name='scale', trainable=trainable)
                self.offset = tf.Variable(tf.constant(0.0, shape=[shape]), name='offset', trainable=trainable)
                self.decay = decay
                self.eps = eps

    def __call__(self, x, train=True):
        in_shape = x.get_shape().with_rank(self.dim + 1)
        if len(x.get_shape()) - 1 != self.dim:
             raise ValueError('input shape and dimension size of normalization do not match')

        with tf.variable_scope(self.name):
            if train:
                mean, variance = tf.nn.moments(x, axes=list(range(self.dim)))
                #in_size = reduce(lambda a, b: a * b, in_shape) #This works in python2
                in_size = functools.reduce(lambda a, b: a*b, in_shape) #This should work on python2 and python3
                area = int(in_size) // self.shape
                adjust = area / max(area - 1., 1.)
                new_mean = self.decay * self.mean + (1 - self.decay) * mean
                new_variance = self.decay * self.variance
                new_variance += (1 - self.decay) * adjust * variance
                assign_mean = self.mean.assign(new_mean)
                assign_variance = self.variance.assign(new_variance)

                with tf.control_dependencies([assign_mean, assign_variance]):
                    return tf.nn.batch_normalization(
                        x, mean, variance, self.offset,
                        self.scale, self.eps)
            else:
                return tf.nn.batch_normalization(
                        x, self.mean, self.variance, self.offset, None, self.eps)
