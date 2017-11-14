from IPython.core.debugger import Tracer; keyboard = Tracer()
import tensorflow as tf
import abc

class BaseModel(object):
    """
    Abstracted class for neural network models using tensorflow.
    """

    def __init__(self, name, seed):
        tf.set_random_seed(seed)
        self.seed = seed
        self.name = name

    def getAllVariables(self):
        # returns dictionary wihch contrains all model's variable
        # e.x: modelname := 'CNN'  ->  'CNN/foo/bar', 'foo/CNN/bar' are OK, 'CTCNN/foo/bar' will be ignored
        r_dict = {}
        for v in tf.all_variables():
            strNum = v.name.find(self.name+'/')
            if strNum >= 0:
                if strNum == 0 or v.name[strNum-1] == '/':
                    r_dict[v.name] = v
        return r_dict

    def checkWeightSum(self):
        # retuens sum of all variables in the model
        s = 0.0
        for name, v in self.getAllVariables().iteritems():
            s += tf.reduce_sum(v)
        return s
