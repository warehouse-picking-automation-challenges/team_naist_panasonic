import tensorflow as tf
import os,sys
from PIL import Image
import matplotlib.pyplot as plt
import random,math
import numpy as np
from IPython.terminal.debugger import set_trace as keyboard

def prepare_csv_data_list(self):
    print("Generating Traing List...")
    generate_csv = "./training-list.txt"
    Grasp_filelist = os.listdir(self.FLAGS.grasp)
    Image_filelist = os.listdir(self.FLAGS.images)
    random.shuffle(Grasp_filelist)
    with open(generate_csv,"w") as generatefile:
        size = len(Grasp_filelist)
        for i,grasp in enumerate(Grasp_filelist):
            # progress bar
            sys.stdout.write('\r')
            percentage = 1. * (i+1) / size
            progress = int(percentage * 20)
            bar_arg = [progress*'=', ' '*(19-progress), percentage*100]
            bar_arg += [grasp]
            sys.stdout.write('[{}>{}]{:.0f}%  {}'.format(*bar_arg))
            sys.stdout.flush()

            grasp_correspondingname = grasp.replace(".dat",".png")
            if grasp_correspondingname in Image_filelist:
                with open(os.path.join(self.FLAGS.grasp,grasp),"r") as graspfile:
                    grasp = graspfile.readline().replace("\t",",")
                    im = Image.open(os.path.join(self.FLAGS.images,grasp_correspondingname))
                    width = im.width
                    height = im.height
                line = grasp_correspondingname+","+str(width)+","+str(height)+","+grasp
                generatefile.write(line)
    print("Traing List Generated!")
    print("="*20)


def prepare_data_queue(self):
    #Create a data queue and read the data row by row
    data_queue = tf.train.string_input_producer(["./training-list.txt"])
    reader = tf.TextLineReader()
    key,value = reader.read(data_queue)
    record_defaults = [['aa'],[1.0],[1.0],[1.0],[1.0],[1.0],[1.0]] #Define how data will be read
    imagefile,orig_width,orig_height,x_center,y_center,rad,width = tf.decode_csv(value,record_defaults=record_defaults)

    #Read the image data name from files and decoded it to image tensor
    png_r = tf.read_file(self.FLAGS.images+"/"+imagefile)
    image_orig = tf.image.decode_png(png_r)
    self.imagefile = imagefile
    self.train_data_process(image_orig,orig_width,orig_height,x_center,y_center,rad,width)

def prepare_test_data(self):
    print("Loading Testing Data...")
    print("="*20)
    #Read Data in Test Folder
    test_images_files = sorted(os.listdir(self.FLAGS.images_test))
    test_grasp_files = sorted(os.listdir(self.FLAGS.grasp_test))

    size = len(test_grasp_files)
    test_list = []
    for i,grasp_filename in enumerate(test_grasp_files):

        # progress bar
        sys.stdout.write('\r')
        percentage = 1. * (i+1) / size
        progress = int(percentage * 20)
        bar_arg = [progress*'=', ' '*(19-progress), percentage*100]
        bar_arg += [grasp_filename]
        sys.stdout.write('[{}>{}]{:.0f}%  {}'.format(*bar_arg))
        sys.stdout.flush()

        grasp_correspondingname = grasp_filename.replace(".dat",".png")
        if grasp_correspondingname in test_images_files:
            with open(os.path.join(self.FLAGS.grasp_test,grasp_filename),"r") as graspfile:
                im = Image.open(os.path.join(self.FLAGS.images_test,grasp_correspondingname))
                width = im.width
                height = im.height
                grasp_orig = graspfile.readline().split("\t")
                grasp_orig = np.asarray(grasp_orig,np.float32)
                test_list.append([grasp_correspondingname,grasp_orig])
    self.test_list = test_list
    print("Testing Data Loaded!")
    print("="*20)


def train_data_process(self,image_orig,orig_width,orig_height,x_center,y_center,rad,width):
    #Resize image to 224x224 to input CNN
    image = tf.image.resize_images(image_orig,[224,224])
    image.set_shape((224, 224, 3))
    self.image = image

    [x_center_normalized,y_center_normalized,rad_normalized,width_normalized] = self.fit_grasp_to_net_TF(orig_width,orig_height,x_center,y_center,rad,width)

    self.grasp = tf.stack([x_center_normalized,y_center_normalized,rad_normalized,width_normalized])

def fit_grasp_to_net_TF(self,orig_width,orig_height,x_center,y_center,rad,width):
    #Get points to define width
    p2 = [x_center+(width/2)*tf.sin(rad),y_center+(width/2)*tf.cos(rad)]
    p1 = [x_center-(width/2)*tf.sin(rad),y_center-(width/2)*tf.cos(rad)]

    #Do shifting and rotating to grasping since we have a resize image
    change_ratio = [224/orig_width,224/orig_height]
    p1_shift = [change_ratio[0]*p1[0],change_ratio[1]*p1[1]]
    p2_shift = [change_ratio[0]*p2[0],change_ratio[1]*p2[1]]

    #Recalculate shifted grasp line angles
    shift_p1_p2_displacement_x = p2_shift[0]-p1_shift[0]
    shift_p1_p2_displacement_y = p1_shift[1]-p2_shift[1]
    rad_resized = tf.atan2(shift_p1_p2_displacement_x,shift_p1_p2_displacement_y)
    cos2rad = tf.cos(2*rad_resized)
    sin2rad = tf.sin(2*rad_resized)

    #Recalculate shifted grasp line length
    width_resized = tf.norm(tf.stack([shift_p1_p2_displacement_x,shift_p1_p2_displacement_y]))

    #Calculate shifted grasp center point
    x_center_shift = change_ratio[0]*x_center
    y_center_shift = change_ratio[1]*y_center

    #Normalize all values to [-0.9~0.9]
    x_center_normalized = self._remap(x_center_shift/224,1.0,0,0.9,-0.9)
    y_center_normalized = self._remap(y_center_shift/224,1.0,0,0.9,-0.9)
    rad_normalized = self._remap(rad_resized,3.14,-3.14,0.9,-0.9)
    cos2rad_normalized = self._remap(cos2rad,1,-1,0.9,-0.9)
    sin2rad_normalized = self._remap(sin2rad,1,-1,0.9,-0.9)

    width_normalized = self._remap(width_resized,224,0,0.9,-0.9)

    return [x_center_normalized,y_center_normalized,rad_normalized,width_normalized]


def fit_grasp_to_net(self,orig_width,orig_height,x_center,y_center,rad,width):
    #Get points to define width
    p2 = [x_center+(width/2)*math.sin(rad),y_center+(width/2)*math.cos(rad)]
    p1 = [x_center-(width/2)*math.sin(rad),y_center-(width/2)*math.cos(rad)]

    #Do shifting and rotating to grasping since we have a resize image
    change_ratio = [224/orig_width,224/orig_height]
    p1_shift = [change_ratio[0]*p1[0],change_ratio[1]*p1[1]]
    p2_shift = [change_ratio[0]*p2[0],change_ratio[1]*p2[1]]

    #Recalculate shifted grasp line angles
    shift_p1_p2_displacement_x = p2_shift[0]-p1_shift[0]
    shift_p1_p2_displacement_y = p1_shift[1]-p2_shift[1]
    rad_resized = math.atan2(shift_p1_p2_displacement_x,shift_p1_p2_displacement_y)
    cos2rad = math.cos(2*rad_resized)
    sin2rad = math.sin(2*rad_resized)

    #Recalculate shifted grasp line length
    width_resized = math.sqrt(shift_p1_p2_displacement_x**2 + shift_p1_p2_displacement_y**2)

    #Calculate shifted grasp center point
    x_center_shift = change_ratio[0]*x_center
    y_center_shift = change_ratio[1]*y_center

    #Normalize all values to [-0.9~0.9]
    x_center_normalized = self._remap(x_center_shift/224,1.0,0,0.9,-0.9)
    y_center_normalized = self._remap(y_center_shift/224,1.0,0,0.9,-0.9)
    rad_normalized = self._remap(rad_resized,3.14,-3.14,0.9,-0.9)
    cos2rad_normalized = self._remap(cos2rad,1,-1,0.9,-0.9)
    sin2rad_normalized = self._remap(sin2rad,1,-1,0.9,-0.9)

    width_normalized = self._remap(width_resized,224,0,0.9,-0.9)

    return [x_center_normalized,y_center_normalized,rad_normalized,width_normalized]
