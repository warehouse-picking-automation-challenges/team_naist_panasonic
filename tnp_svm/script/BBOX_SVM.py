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
import argparse

import pandas as pd

def getArgs():
    parser = argparse.ArgumentParser()

    parser.add_argument("--data-dir", default="./amazon_data/bbox_data/", help="choose data directory")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--kernel", choices=["linear", "rbf"], type=str, default="linear", help="kernel functions")

    return parser.parse_args()

def remap(OldValue,OldMax=0.45,OldMin=0,NewMax=0.9,NewMin=0.1):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue

def remap_volume(OldValue,OldMax=0.01,OldMin=0,NewMax=0.9,NewMin=0.1):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue



def Matching():
    args = getArgs()
    start = time.time()

    category_dirs = sorted(os.listdir(args.data_dir))
    """
    des = [] #Features
    labels=[] #label
    #Create Label
    volumes = []
    # We will redo this part in pandas
    
    for category_dir in category_dirs:
        if os.path.isdir(os.path.join(args.data_dir,category_dir)):
            label = category_dir
            for file in os.listdir(os.path.join(args.data_dir, category_dir)):
                if file.endswith('bbox.yml'):
                    fs = cv2.FileStorage(os.path.join(args.data_dir,category_dir,file),0)

                    vector = fs.getNode("bbox").mat()[0].tolist()
                    vector_normalized = []
                    for value in vector:
                        vector_normalized.append(remap(value,0.45,0,0.9,0.1))
                    volume = vector[0]*vector[1]*vector[2]
                    volume_normalized = remap_volume(volume,0.01,0,0.9,0.1)
                    vector_normalized.append(volume_normalized)
                    #volumes.append(volume)
                    labels.append(label)
                    #print("ClassName=%s" % (label))
                    des.append(vector_normalized)
    """

    # Read in the data
    des=[]
    used_category=[]
    labels=[]
    for file in os.listdir(args.data_dir):
        if file.endswith(".csv"):
            my_dataframe = pd.read_csv(os.path.join(args.data_dir,file))
            category_tmp = my_dataframe["item name"][0]
            if category_tmp not in used_category:
                used_category.append(category_tmp)
            x_value = remap(my_dataframe[" x"][0])
            y_value = remap(my_dataframe[" y"][0])
            z_value = remap(my_dataframe[" z"][0])
            xyz = [x_value,y_value,z_value]
            xyz = sorted(xyz)   
            des.append(xyz)

            labels.append(category_tmp)

    print 'start Grid Search'
    search_list=[29.5]
    tuned_parameters = [{'C': search_list}]

    linear_svc = LinearSVC()#SMN Modoul
    gscv = GridSearchCV(linear_svc, tuned_parameters ,cv=5) #parameters grid search
    print 'gscv.fit'
    gscv.fit(des, labels)#Training parameters tuning (Find the best parameters automatically)

    svm_best = gscv.best_estimator_#Choose the best tuned parameter
    print 'start re-learning SVM with best parameter set.'

    #### CalibratedClassifierCV is the calibrated classifier which can give probabilistic classifier
    ####sigmoid will use Platt's scaling. Refer to documentation for other methods.
    print 'searched result of  C =', svm_best.C
    best_C_for_record = svm_best.C

    svm_best = CalibratedClassifierCV(svm_best,method='sigmoid',cv=3) #Parameter setup
    svm_best.fit(des, labels) #Training

    #Save Model
    joblib.dump(svm_best, './bbox_svm_parameter.pkl', compress=9)

    #Load Model
    svm_best = joblib.load('./bbox_svm_parameter.pkl')

    ####load test image    (load test data)
    #label, test_image = LoadFile("crop_burts2.png", "/root/catkin_ws/src/tnp/tnp_svm/script/data/RealData")
    correct_count = 0
    for n in range(len(des)):
        des_test = des[n]

        ###### SVM recognition
        classes = svm_best.classes_ #Model load label Must do

        #Forward Predict
        testResponse = svm_best.predict(des_test)[[0]]
        #print("testResponse="+ str(testResponse[0]))

        # print(classes)
        confidences_matrix = svm_best.predict_proba(des_test)
        # print(confidences_matrix)
        labels_class = svm_best.classes_ #read label again(You must do this or your label will changed by machine itself)
        #print("Real: "+str(labels[n]))
        sort_ascend = np.sort(confidences_matrix) #ranking the list of predicted data
        # print(sort_ascend)
        sort_index_ascend = np.argsort(confidences_matrix) #get index and sort
        # print(sort_index_ascend)
        sort_descend = sort_ascend[0][::-1] #ranked from big to small
        #print "sort_descend" ,sort_descend
        sort_index_descend = sort_index_ascend[0][::-1] #ranked from big to small (index)
        # print(sort_index_descend)
        items_ids=[]
        for i in range (0,len(labels_class)):
            items_ids.append(labels_class[sort_index_descend[i]])
        #print(items_ids)
        if testResponse[0] == labels[n]:
            correct_count+=1

    print("Success Rate: " + str(float(correct_count)/float(len(des))))
    keyboard()


def test_data(x_list):
    end = time.time()
    timecost = end - start
    print 'searched result of  C =', best_C_for_record
    print("Time cost:" + str(timecost))
    print("Train End")


if __name__== "__main__":
    Matching()
