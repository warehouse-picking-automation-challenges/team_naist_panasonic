#import sys; sys.path.append('/root/catkin_ws/src/team-naist-panasonic/yolo_light/scripts/net')
from IPython.terminal.debugger import set_trace as keyboard
import sys
import os
import cv2
import time
import numpy as np
import tensorflow as tf
import pickle
from PIL import Image

from .image_batch_modifier import *

from utils.pascal_voc_clean_xml import pascal_voc_clean_xml

train_stats = (
    'Training statistics: \n'
    '\tLearning rate : {}\n'
    '\tBatch size    : {}\n'
    '\tEpoch number  : {}\n'
    '\tBackup every  : {}'
)


def crop_image_by_box(img,box_boundary,crop_percentage):
    image = img.copy()
    max_x, max_y, _ = image.shape
    xmin,xmax,ymin,ymax = box_boundary

    xmin_padding = 0
    xmax_padding = 0
    ymin_padding = 0
    ymax_padding = 0
    #Cropping with modified_range
    width = xmax - xmin + 1
    height = ymax - ymin + 1
    x_center = int(xmin + width * 0.5)
    y_center = int(ymin + height * 0.5)

    modified_width = int(width*crop_percentage)
    modified_height = int(height*crop_percentage)
    modified_xmin = int(x_center-(modified_width/2))
    modified_ymin = int(y_center-(modified_height/2))
    modified_xmax = modified_xmin+modified_width
    modified_ymax = modified_ymin+modified_height
    if crop_percentage > 1.0:
        if modified_xmin < 0:
            xmin_padding = int(math.fabs(modified_xmin))
        if modified_xmax > (max_x-1):
            xmax_padding = int(modified_xmax-max_x)
        if modified_ymin < 0:
            ymin_padding = int(math.fabs(modified_ymin))
        if modified_ymax > (max_y-1):
            ymax_padding = int(modified_ymax-max_y)
        image = cv2.copyMakeBorder(img,ymin_padding,ymax_padding,xmin_padding,xmax_padding,cv2.BORDER_CONSTANT)
        crop_x=[xmin_padding+modified_xmin,xmin_padding+modified_xmin+modified_width]
        crop_y=[ymin_padding+modified_ymin,ymin_padding+modified_ymin+modified_height]
    else:
        crop_x=[modified_xmin,modified_xmin+modified_width]
        crop_y=[modified_ymin,modified_ymin+modified_height]

    crop_img = image[crop_y[0]:crop_y[1],crop_x[0]:crop_x[1]]
    crop_coord_shift = [xmin_padding,ymin_padding]
    return crop_img,crop_coord_shift

def cv22PIL(imgcv2):
    imagePIL = imgcv2
    imagePIL = imgcv2[::-1, :, ::-1].copy() # BGR->RGB
    imagePIL = imgcv2[:,:,::-1].copy() #flip image
    imagePIL = Image.fromarray(imagePIL)
    return imagePIL

def _save_ckpt(self, step, loss_profile):
    file = '{}-{}{}'
    model = self.meta['name']

    profile = file.format(model, step, '.profile')
    profile = os.path.join(self.FLAGS.backup, profile)
    with open(profile, 'wb') as profile_ckpt:
        pickle.dump(loss_profile, profile_ckpt)

    ckpt = file.format(model, step, '')
    ckpt = os.path.join(self.FLAGS.backup, ckpt)
    self.say('Checkpoint at step {}'.format(step))
    self.saver.save(self.sess, ckpt)


def get_test_namelist(dumps,labels):
    namelist = []
    for dataset in dumps:
        if not len(dataset[1][2]) == 0:
            namelist.append(dataset[0])
    return namelist

def get_label_from_annofile(xmlpath):
    with open(xmlpath,"r") as file:
        lines = file.readlines()
    for line in lines:
        if "<name>" in line:
            label = line.strip("/t").strip("/n").strip("<name>").strip("</name>")
            return label

def test_accuracy(self):
    inp_path = self.FLAGS.test_accuracy_dataset
    inp_anno_path = self.FLAGS.test_accuracy_annotation
    all_inps = os.listdir(inp_path)
    all_inps = [i for i in all_inps if self.framework.is_inp(i)]
    if not all_inps:
        msg = 'Failed to find any test files for testing accuracy in {} .'
        exit('Error: {}'.format(msg.format(inp_path)))
    dumps = pascal_voc_clean_xml(inp_anno_path,self.meta['labels'],False,showinfo=False)
    namelist = get_test_namelist(dumps,self.meta['labels'])
    total_num = len(namelist)
    if total_num == 0:
        msg = 'Failed to find any test files that matches the classes write in label  {} .'
        exit('Error: {}'.format(msg.format(inp_path)))

    threshold = 0.5
    self.say("="*25 + "Accuracy Test"+"="*25)
    for i in range(5):
        detect_count = 0
        correct_count = 0
        for file in namelist:
            imgfile = os.path.join(inp_path,file)
            img = cv2.imread(imgfile)
            _, boxesInfo, ImageDetectionsInfo = self.return_predict(img,"Test",threshold)
            if boxesInfo != []:
                classified_label = boxesInfo[0]['label']
                detect_count = detect_count + 1
                annofile = os.path.join(inp_anno_path,os.path.splitext(file)[0]+".xml")
                if classified_label in get_label_from_annofile(annofile):
                    correct_count = correct_count + 1

        if detect_count != 0:
            succes_detect = float(detect_count)/float(total_num)
            succes_recog = float(correct_count)/float(total_num)
            form = 'Test in threshold {} : Detect Rate {}% - Recognition Succes Rate for box1 {}%'
            self.say(form.format(threshold,succes_detect*100, succes_recog*100))
            break
        else:
            form = 'Test in threshold {} : Dectect failed. No bounding box found.'
            self.say(form.format(threshold))
        threshold=threshold-0.1
        threshold=max(threshold,0.05)
    self.say("="*63)

def train(self):
    loss_ph = self.framework.placeholders
    loss_mva = None; profile = list()
    batches = self.framework.shuffle()
    loss_op = self.framework.loss
    np.random.seed(self.FLAGS.seed)

    DivergeTrigger=False
    for i, (x_batch, datum) in enumerate(batches):
        if not i: self.say(train_stats.format(
            self.FLAGS.lr, self.FLAGS.batch,
            self.FLAGS.epoch, self.FLAGS.save
        ))

        #Train with Data Modification
        x_batch = trans2uint(x_batch)
        if not self.FLAGS.blur == 0:
            x_batch = apply_blur_batch(x_batch,self.FLAGS.blur)

        if not self.FLAGS.gamma == 0:
            x_batch = adjust_gamma_batch(x_batch,(1,self.FLAGS.gamma))

        if not self.FLAGS.saturation == 0:
            x_batch = adjust_saturation_batch(x_batch,self.FLAGS.saturation)
        if not self.FLAGS.exposure == 0:
            x_batch = adjust_exposure_batch(x_batch,self.FLAGS.exposure)

        x_batch = trans2float(x_batch)
        if not self.FLAGS.noiselevel == 0:
            x_batch = add_noise_batch(x_batch,self.FLAGS.noiselevel)

        feed_dict = {
            loss_ph[key]: datum[key]
                for key in loss_ph }
        feed_dict[self.inp] = x_batch
        feed_dict.update(self.feed)

        fetches = [self.train_op, loss_op, self.summary_op]
        fetched = self.sess.run(fetches, feed_dict)
        loss = fetched[1]

        if loss_mva is None: loss_mva = loss
        loss_mva = .9 * loss_mva + .1 * loss
        step_now = self.FLAGS.load + i + 1

        self.writer.add_summary(fetched[2], step_now)

        form = 'step {} - loss {} - moving ave loss {}'
        self.say(form.format(step_now, loss, loss_mva))
        if str(loss) == "nan":
            self.say("Diverge Occurs!Force Stop!")
            DivergeTrigger=True
            break

        profile += [(loss, loss_mva)]


        ckpt = (i+1) % (self.FLAGS.save // self.FLAGS.batch)
        args = [step_now, profile]
        if not ckpt:
            _save_ckpt(self, *args)
            if self.FLAGS.test_accuracy_dataset is not "":
                test_accuracy(self)

    if ckpt and (DivergeTrigger!=True):
        _save_ckpt(self, *args)
        if self.FLAGS.test_accuracy_dataset is not "":
            test_accuracy(self)

def return_predict(self,im, imname = None, threshold=None):
    if not imname == None:
        h, w, _ = im.shape # Training / Python.
        useROS = False
    else:
        _, h, w, _ = im.shape  # Detection / ROS.
        useROS = True
        import rospy
    imgcv = np.copy(im)
    im = self.framework.resize_input(im,useROS)
    h2, w2, _ = im.shape
    this_inp = np.expand_dims(im, 0)
    feed_dict = {self.inp : this_inp}

    out = self.sess.run(self.out, feed_dict)[0]
    boxes = self.framework.findboxes(out)
    if threshold is None:
        threshold = self.FLAGS.threshold
    else:
        self.FLAGS.threshold = threshold
    boxesInfo = list()
    ImageDetectionsInfo = list()

    # meta
    meta = self.meta

    crop_percentage = self.FLAGS.pred_grasp_crop_percent
    colors = meta['colors']
    labels = meta['labels']

    for box in boxes:
        tmpBox = self.framework.process_box(box, h, w, threshold)
        tmpDetection = self.framework.filter_box(box, threshold)
        if tmpBox is None:

            continue
        max_indx = tmpBox[5]
        thick = int((h + w) // 300)
        if self.pred_grasp:
            #Get the bounding box area [xmin,xmax,ymin,ymax]
            box_boundary=[tmpBox[0],tmpBox[1],tmpBox[2],tmpBox[3]]

            #Crop the box area from whole image
            crop_img,crop_coord_shift = crop_image_by_box(imgcv[0],box_boundary,crop_percentage)

            crop_img_PIL = cv22PIL(crop_img)
            grasp_pred = self.GraspPredictNet.predict(crop_img_PIL)

            #Initialize the grasp
            tmpDetection.x_grasp = 0.0
            tmpDetection.y_grasp = 0.0
            tmpDetection.theta_grasp = 0.0
            tmpDetection.width_grasp = 0.0
            if self.pred_grasp:
                tmpDetection = self.framework.process_grasp(tmpDetection,grasp_pred,h,w,crop_percentage)
            boxesInfo.append({
                "label": tmpBox[4],
                "confidence": tmpBox[6],
                "topleft": {
                    "x": tmpBox[0],
                    "y": tmpBox[2]},
                "bottomright": {
                    "x": tmpBox[1],
                    "y": tmpBox[3]},
                "grasp": {
                    "x": tmpBox[0] + grasp_pred[0] - crop_coord_shift[0],
                    "y": tmpBox[2] + grasp_pred[1] - crop_coord_shift[1],
                    "theta": grasp_pred[2],
                    "width": grasp_pred[3]}} )
        else:
            boxesInfo.append({
                "label": tmpBox[4],
                "confidence": tmpBox[6],
                "topleft": {
                    "x": tmpBox[0],
                    "y": tmpBox[2]},
                "bottomright": {
                    "x": tmpBox[1],
                    "y": tmpBox[3]},
                "grasp": {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "width": 0.0}})

        ImageDetectionsInfo.append(tmpDetection)
    return imgcv, boxesInfo, ImageDetectionsInfo


import math

def predict(self):
    inp_path = self.FLAGS.test
    all_inps = os.listdir(inp_path)
    all_inps = [i for i in all_inps if self.framework.is_inp(i)]
    if not all_inps:
        msg = 'Failed to find any test files in {} .'
        exit('Error: {}'.format(msg.format(inp_path)))

    if self.FLAGS.test_output == "":
        os.path.join(self.FLAGS.test, 'out')
    else:
        if not os.path.isdir(self.FLAGS.test_output):
            os.makedirs(self.FLAGS.test_output)

    batch = min(self.FLAGS.batch, len(all_inps))

    # predict in batches
    n_batch = int(math.ceil(len(all_inps) / batch))
    for j in range(n_batch):
        from_idx = j * batch
        to_idx = min(from_idx + batch, len(all_inps))

        # collect images input in the batch
        inp_feed = list(); new_all = list()
        this_batch = all_inps[from_idx:to_idx]
        for inp in this_batch:
            new_all += [inp]
            this_inp = os.path.join(inp_path, inp)
            this_inp = self.framework.preprocess(this_inp)
            expanded = np.expand_dims(this_inp, 0)
            inp_feed.append(expanded)
        this_batch = new_all
        # Feed to the net
        feed_dict = {self.inp : np.concatenate(inp_feed, 0)}
        self.say('Forwarding {} inputs ...'.format(len(inp_feed)))
        start = time.time()
        out = self.sess.run(self.out, feed_dict)
        stop = time.time(); last = stop - start
        self.say('Total time = {}s / {} inps = {} ips'.format(
            last, len(inp_feed), len(inp_feed) / last))

        # Post processing
        self.say('Post processing {} inputs ...'.format(len(inp_feed)))
        start = time.time()
        for i, prediction in enumerate(out):
            self.framework.postprocess(prediction,
                os.path.join(inp_path, this_batch[i]))
        stop = time.time(); last = stop - start
        # Timing
        self.say('Total time = {}s / {} inps = {} ips'.format(
            last, len(inp_feed), len(inp_feed) / last))
