#!/usr/bin/env python

import threading
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from yolo_light.msg import *
from cv_bridge import CvBridge, CvBridgeError
from IPython.core.debugger import Tracer; keyboard = Tracer()
import math

class TLightNode(object):
    def __init__(self, get_model_callback, model_callback):
        rospy.init_node('tlight_model')
        self.model = get_model_callback()
        self.get_model = get_model_callback
        self.predict = model_callback
        self.bridge = CvBridge()
        self.boxes = None
        self.img =  None
        self.img_out = None
        self.image_lock = threading.RLock()
        self.myDetections = ImageDetections()
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.updateImage, queue_size=1,buff_size=2**24)
        self.pub_img = rospy.Publisher('/out_image', Image, queue_size=1)
        self.pub_det = rospy.Publisher('/out_detection', ImageDetections, queue_size=1)
        rospy.Timer(rospy.Duration(0.04), self.callbackImage)


    def updateImage(self, img):
        arr = self.bridge.imgmsg_to_cv2(img,"bgr8")
        if self.image_lock.acquire(True):
            self.img = arr
            if self.model is None:
                self.model = self.get_model()
            self.img_out, self.boxes, self.myDetections = self.predict(self.model, self.img)
            self.img_out = np.asarray(self.img_out[0,:,:,:])
            for box in self.boxes:
                cv2.rectangle(self.img_out,(box['topleft']['x'],
                                            box['topleft']['y']),
                                            (box['bottomright']['x'],
                                            box['bottomright']['y']),
                                            (255,0,0), 6)
                cv2.putText(self.img_out, box['label'],
                           (box['topleft']['x'],
                           box['topleft']['y'] - 12), 0, 0.6, (255,0,0) ,1)

                if box['grasp']['width'] != 0:
                    #Draw Grasp
                    line_pt1 = [float(box['grasp']['x'])+float(box['grasp']['width'])*math.cos(box['grasp']['theta'])/2., float(box['grasp']['y'])+float(box['grasp']['width'])*math.sin(box['grasp']['theta'])/2.]
                    line_pt2 = [float(box['grasp']['x'])-float(box['grasp']['width'])*math.cos(box['grasp']['theta'])/2., float(box['grasp']['y'])-float(box['grasp']['width'])*math.sin(box['grasp']['theta'])/2.]
                    line_pt1 = (int(line_pt1[0]),int(line_pt1[1]))
                    line_pt2 = (int(line_pt2[0]),int(line_pt2[1]))

                    cv2.line(self.img_out,line_pt1,line_pt2,(0,0,255),2)
                    cv2.circle(self.img_out,(box['grasp']['x'],box['grasp']['y']),3,(0,0,255),-1)
            self.image_lock.release()

    def callbackImage(self, event):
        if self.img_out is None:
            return
        if self.img_out.dtype != np.uint8 or self.img_out.shape[2] != 3:
            print "tlight_node callbackImage wrong type!!!!"
            return
        self.pub_det.publish(self.myDetections)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(self.img_out, "bgr8"))
