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

import glob
import sys
import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String ,Float64 ,Bool
import numpy as np
import matplotlib.pyplot as plt
import os ,re
from tnp_svm.srv import *
import pickle
from sklearn import svm
from sklearn.svm import LinearSVC
from sklearn.calibration import CalibratedClassifierCV
from sklearn import cross_validation
from sklearn.cross_validation import train_test_split
from sklearn.grid_search import GridSearchCV
from sklearn.externals import joblib
from sklearn.multiclass import OneVsRestClassifier
from sklearn.metrics import roc_curve
import math
import numpy as np
from IPython.terminal.debugger import set_trace as keyboard
import random

def Matching():
    start = time.time()

    des = [] #Features
    labels=[] #label
    """
    #Create Label

    for category_dir in category_dirs:
        for rgb_image_path in glob.glob(os.path.join(args.data_dir, group_dir, category_dir)):
            for file in os.listdir(os.path.join(args.data_dir, group_dir, category_dir)):
                if file.endswith('.png'):
                    label, image = LoadFile(file, rgb_image_path)
                    des1 = ComputeHOGDescriptor(image)
                    labels.append(label)
                    print("ClassName=%s" % (label))
                    des.append(des1)
    """

    train_labels = ["hand_weight","windex","bag","meshcup","pen","Ramen"]
    #Create simulate data
    for _ in range(950):
        train_vectors = np.random.rand(3) #data simulation
        des.append(train_vectors)
        chose_label = random.choice(train_labels)
        print(chose_label)
        labels.append(chose_label)

    """
    print 'start Grid Search'
    #### for color
    # tuned_parameters = [{'C': [0.6, 0.7, 0.8, 0.9]}]
    ### 12 12 3 3 C=11.0  32crop C= 10.0
    # tuned_parameters = [{'C': [9.5, 10.0, 11.0]}]

    ### 8 8 1 1 paste on black C=0.5
     # tuned_parameters = [{'C': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}]

    ## 8 8 3 3 rotate C=5.0 32_paste:C=5.0 32_crop:C=5.0 crop:C=4.5
    tuned_parameters = [{'C': [4.5, 5.0, 5.5]}]

    ### 8 8 4 4 C=7.0
    # tuned_parameters = [{'C': [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]}]

    ## 8 8 4 4 rotate:C=2.8 paste:C=2.8
    # tuned_parameters = [{'C': [2.5, 2.6, 2.7, 2.8]}]

    ### 9 9 3 3 rotate C=6.0
    # tuned_parameters = [{'C': [5.5, 5.7, 6.0, 6.2, 6.5]}]

    # tuned_parameters = [{'C': [0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]}]
    # parameters = {'estimator__kernel':('linear', 'rbf'), 'estimator__C':[1, 10]}

    linear_svc = LinearSVC()#SMN Modoul
    gscv = GridSearchCV(linear_svc, tuned_parameters ,cv=5) #parameters grid search
    print 'gscv.fit'
    gscv.fit(des, labels)#Training parameters tuning (Find the best parameters automatically)

    #print 'svm_best'
    svm_best = gscv.best_estimator_#Choose the best tuned parameter
    print 'searched result of  C =', svm_best.C
    print 'start re-learning SVM with best parameter set.'
    """
    #### CalibratedClassifierCV is the calibrated classifier which can give probabilistic classifier
    ####sigmoid will use Platt's scaling. Refer to documentation for other methods.

    svm_best = LinearSVC(C=4.0)
    svm_best = CalibratedClassifierCV(svm_best,method='sigmoid',cv=3) #Parameter setup
    svm_best.fit(des, labels) #Training

    #Save Model
    joblib.dump(svm_best, './bbox_svm_parameter.pkl', compress=9)

    #Load Model
    svm_best = joblib.load('./bbox_svm_parameter.pkl')

    ####load test image    (load test data)
    #label, test_image = LoadFile("crop_burts2.png", "/root/catkin_ws/src/tnp/tnp_svm/script/data/RealData")
    des_test = des[0]

    ###### SVM recognition
    classes = svm_best.classes_ #Model load label Must do

    #Forward Predict
    testResponse = svm_best.predict(des_test)
    print("testResponse=", (testResponse))

    # print(classes)
    confidences_matrix = svm_best.predict_proba(des_test)
    # print(confidences_matrix)
    labels = svm_best.classes_ #read label again(You must do this or your label will changed by machine itself)
    sort_ascend = np.sort(confidences_matrix) #ranking the list of predicted data
    # print(sort_ascend)
    sort_index_ascend = np.argsort(confidences_matrix) #get index and sort
    # print(sort_index_ascend)
    sort_descend = sort_ascend[0][::-1] #ranked from big to small
    print "sort_descend" ,sort_descend
    sort_index_descend = sort_index_ascend[0][::-1] #ranked from big to small (index)
    # print(sort_index_descend)
    items_ids=[]
    for i in range (0,len(labels)):
        items_ids.append(labels[sort_index_descend[i]])
    print(items_ids)
    keyboard()
    i=0

if __name__== "__main__":
    Matching()
