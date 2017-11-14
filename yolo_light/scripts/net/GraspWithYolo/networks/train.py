import tensorflow as tf
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from IPython.terminal.debugger import set_trace as keyboard

def build_train_op(self):
    with tf.device('/cpu:0'):
        min_queue_examples =20
        num_preprocess_threads = 16
        imagefile_batch,image_batch,grasp_batch = tf.train.shuffle_batch(
            [self.imagefile,self.image,self.grasp],
            batch_size=self.FLAGS.batch,
            num_threads=num_preprocess_threads,
            capacity=min_queue_examples + 3 * self.FLAGS.batch,
            min_after_dequeue=min_queue_examples)
    with tf.device(self.arg_gpu):
        pred = self.model(image_batch,self.FLAGS.batch,base='relu',train=True)

        loss = self.loss_function(grasp_batch,pred)
        loss2 = self.loss_function2(grasp_batch,pred)
        lr = self.FLAGS.lr
        optimizer = tf.train.AdamOptimizer(learning_rate=lr,
                beta1=0.9,
                beta2=0.999,
                epsilon=1e-08,
                use_locking=False,
                name='Adam')
        grads = optimizer.compute_gradients(loss)
        train_op = optimizer.apply_gradients(grads)

        self.truth_grasp = grasp_batch
        self.pred = pred
        self.imagefile_batch = imagefile_batch
        self.train_op = train_op
        self.loss = loss
        self.loss2 = loss2

def loss_function(self,teach,predict):
    """
    loss function is calculate by difference of all predict & teach values
    """
    loss = tf.reduce_mean(tf.squared_difference(teach,predict))
    with tf.device('/cpu:0'):
        tf.summary.scalar('loss', loss)
    return loss

def loss_function2(self,teach,predict):
    """
    loss function2 is calculate by distance difference of grasp center and the difference of turning angle
    """
    dx = teach[0] - predict[0]
    dy = teach[1] - predict[1]
    pt_distance = tf.sqrt(dx**2 + dy**2)


    sin2rad_t = teach[2]
    cos2rad_t = teach[3]
    rad_t = tf.atan2(cos2rad_t,sin2rad_t) / 2

    sin2rad_p = predict[2]
    cos2rad_p = predict[3]
    rad_p = tf.atan2(cos2rad_p,sin2rad_p) / 2
    d_rad = rad_t - rad_p

    w_t = teach[4]
    w_p = predict[4]
    dw = w_t-w_p

    loss_distance_sum = tf.reduce_sum(pt_distance)
    loss_rad_sum = tf.reduce_sum(tf.sqrt(d_rad**2))
    loss_w_sum = tf.reduce_sum(tf.sqrt(dw**2))

    loss2 = (loss_distance_sum+loss_rad_sum)/self.FLAGS.batch #Calculate mean

    with tf.device('/cpu:0'):
        tf.summary.scalar('loss2', loss2)
    return loss2


def train(self):
    epochs = self.FLAGS.epoch
    loss_mva = None
    plt.ion()
    if not self.FLAGS.load == "":
        step = self.FLAGS.load
    else:
        step = 0

    for _ in range(epochs+1):
        step += 1

        _,loss,loss2,pred,truth_grasp,imagefile_batch = self.sess.run([self.train_op,self.loss,self.loss2,self.pred,self.truth_grasp,self.imagefile_batch])
        if step%self.FLAGS.save == 0 and step != 0:
            #Test Operation
            self.test()
            self.test_successrate()

            self.save_checkpoint(step)


        if loss_mva is None: loss_mva = loss
        loss_mva = .9 * loss_mva + .1 * loss

        print("Step"+str(step)+" loss = "+str(loss) + " loss2 = "+str(loss2)) + " loss_mva = "+str(loss_mva)

        print("Image: " + str(imagefile_batch[0]))
        print("pred[0] = "+str(pred[0]))
        print("teach_grasp[0] = "+str(truth_grasp[0]))
        image = Image.open(self.FLAGS.images+"/"+imagefile_batch[0])
        image = self.draw_image_PIL_Rad(image,truth_grasp[0],'blue',radius=2,linethick=3)
        image = self.draw_image_PIL_Rad(image,pred[0],'red',radius=2)
        plt.imshow(np.asarray(image))
        plt.title(str(imagefile_batch[0]))
        plt.pause(.01)
        plt.show()
        plt.clf()
        
