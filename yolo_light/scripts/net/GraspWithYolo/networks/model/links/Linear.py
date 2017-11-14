import tensorflow as tf
import math

class Linear:
    """
    A fully connected layer without activation function, actually calculates W*x

    input shape : [batch, dim]

    name : string. Name of variable scope
    indim : integer. Number of input data's dimension
    outdim : integer. Number of output dimension
    wscale : float. Scaling factor to initialize weights
    nobias : Boolean. True means the layer does not have bias
    """
    def __init__(self, name, indim, outdim, wscale=1.0, nobias=False, trainable=True, initBias=None):
        self.name = name
        with tf.variable_scope(self.name):
            self.nobias = nobias
            with tf.name_scope('linear') as scope:
                self.W = tf.Variable( tf.truncated_normal([indim, outdim], \
                            stddev=wscale * 1.0/math.sqrt(float(indim))), \
                            name='weights', \
                            trainable=trainable)
                if not self.nobias:
                    if initBias is None:
                        self.b = tf.Variable( tf.zeros([outdim]), name='biases', trainable=trainable )
                    elif isinstance(initBias, float) or isinstance(initBias, int):
                        self.b = tf.Variable( tf.constant(initBias, shape=[outdim], dtype=tf.float32), name='biases', trainable=trainable )
                    else:
                        self.b = initBias # customised bias
    def __call__(self, x):
        with tf.variable_scope(self.name):
            if not self.nobias:
                return tf.matmul(x, self.W) + self.b
            else:
                return tf.matmul(x, self.W)
