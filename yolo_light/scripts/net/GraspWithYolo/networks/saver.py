import tensorflow as tf
import os


def save_checkpoint(self,step):
    if not os.path.isdir(self.FLAGS.backup):
        os.makedirs(self.FLAGS.backup)
    series_name = self.FLAGS.name
    path = os.path.join(self.FLAGS.backup,series_name)
    self.saver.save(self.sess,path,global_step=step)
    print("Checkpoint "+str(step)+" saved!")

def load_from_ckpt(self):
    if self.FLAGS.load == -1: # load lastest ckpt
        with open(self.FLAGS.backup + '/' +'checkpoint', 'r') as f:
            last = f.readlines()[-1].strip()
            load_point = last.split(' ')[1]
            load_point = load_point.split('"')[1]
            load_point = load_point.split('-')[-1]
            self.FLAGS.load = int(load_point)
    load_point = os.path.join(self.FLAGS.backup, self.FLAGS.name)
    load_point = '{}-{}'.format(load_point, self.FLAGS.load)
    print('Loading from {}'.format(load_point))
    self.saver.restore(self.sess, load_point)
