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

import os
import numpy as np
from PIL import Image
import math,random
from IPython.terminal.debugger import set_trace as keyboard
from .utils.evaluation import success_evaluate

def predict(self,image):
    """
    Input with 1 PIL image, Output with predicted grasp corresponding to the original image coordinate
    """
    image_tf_input = image.resize((224,224))
    image_tf_input = np.asarray(image_tf_input)
    image_tf_input = np.reshape(image_tf_input,(1,224,224,3))

    #Remain original image for drawing
    image_tf_orig = np.asarray(image)

    #Foward 224x224 sized image to model and get raw value from model
    pred = self.sess.run([self.forward_op],feed_dict={self.x_image:image_tf_input})[0]

    #Denormalize to original image
    pred_denormalized = self.fit_original_image_Rad(image_tf_orig,pred)
    return pred_denormalized


def predict_test_data(self,output=None):
    """
    Read image files in image_test and output with predicted images of grasp
    """
    #Create the output dir
    if output == None:
        output = os.path.join(self.FLAGS.images_test,"output")
    if not os.path.isdir(output):
        os.makedirs(output)
    print("Doing prediction..")
    #Read the files in images_test
    file_list = os.listdir(self.FLAGS.images_test)
    file_list.sort()
    for file in file_list:
        if file.endswith(".png"):
            print(file)
            image_path = os.path.join(self.FLAGS.images_test,file)
            image_tf = Image.open(image_path)
            real_pred = self.predict(image_tf)
            image_pred = self.draw_image_PIL_Rad(image_tf,real_pred,linethick=3,normalize=False)

            image_pred.save(os.path.join(output,file))


def test(self):
    if len(self.test_list) == 0:
        print("Test list Empty, Skip Testing Operation!")
        return False

    test_file_pick = random.sample(self.test_list,self.FLAGS.batch)

    image_tf_input_stack =[]
    grasp_stack=[]
    for test_file in test_file_pick:
        image_file = test_file[0]
        image_tf = Image.open(os.path.join(self.FLAGS.images_test,image_file))
        image_tf_input = image_tf.resize((224,224))
        image_tf_input = np.asarray(image_tf_input)
        image_tf_input = np.reshape(image_tf_input,(1,224,224,3))
        image_tf_input_stack.append(image_tf_input)
        grasp = test_file[1]
        grasp_normalized = self.fit_grasp_to_net(image_tf.width,image_tf.height,grasp[0],grasp[1],grasp[2],grasp[3])
        grasp_stack.append(grasp_normalized)
    image_tf_input_stack = np.vstack(image_tf_input_stack)
    grasp_stack = np.vstack(grasp_stack)
    pred,test_loss = self.sess.run([self.test_op,self.test_loss],feed_dict={self.test_image_batch:image_tf_input_stack, self.test_grasp_batch:grasp_stack})

    print("Random Picked Test loss = " + str(test_loss))


def test_successrate(self):
    files_num = len(self.test_list)
    success_count = 0
    for test_file in self.test_list:
        image_file = test_file[0]
        image_tf = Image.open(os.path.join(self.FLAGS.images_test,image_file))
        image_tf_input = image_tf.resize((224,224))
        image_tf_input = np.asarray(image_tf_input)
        image_tf_input = np.reshape(image_tf_input,(1,224,224,3))
        grasp = test_file[1]
        grasp_normalized = self.fit_grasp_to_net(image_tf.width,image_tf.height,grasp[0],grasp[1],grasp[2],grasp[3])

        pred = self.sess.run([self.forward_op],feed_dict={self.x_image:image_tf_input})[0]
        if success_evaluate(pred,grasp_normalized):
            success_count += 1
    success_rate = float(success_count)/float(files_num) * 100
    print("Evaluate Success Rate: " + str(success_rate))
