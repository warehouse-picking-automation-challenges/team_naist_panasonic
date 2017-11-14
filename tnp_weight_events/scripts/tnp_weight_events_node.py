#!/usr/bin/env python
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

import rospy
import math
from std_msgs.msg import String, Float64, Bool
from tnp_weight_events.srv import *
from tnp_optoforce.srv import *
from collections import deque

from std_srvs.srv import Empty

import numpy as np

# Great job python san
def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

### ============================== math functions for get_similarity
# See https://repl.it/J3Ns/12 for comparison graphs

# Sigmoid with p a list of parameters
# https://stackoverflow.com/questions/4308168/sigmoidal-regression-with-scipy-numpy-python-etc
def sigmoid(x, x0, k):
    y = 1 / (1 + np.exp( -k*(x-x0) ))
    return y

# Difference of two fuzzy sigmoid membership functions.
# http://pythonhosted.org/scikit-fuzzy/api/skfuzzy.html
# b : Midpoint of sigmoid; f1(b1) = 0.5
# c : Width and sign of sigmoid.
# sigmoid1(x) = 1 / (1. + exp[- c1 * (x - b1)])
def dsigmf(x, b1, c1, b2, c2):
    y = sigmoid(x, b1, c1) - sigmoid(x, b2, c2)
    return y

# Normal distribution.
# https://docs.scipy.org/doc/numpy/reference/generated/numpy.random.normal.html
def gauss(x, mu, sigma):
  # y = 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (x - mu)**2 / (2 * sigma**2))      # Normal distribution
  y = np.exp( - (x - mu)**2 / (sigma**2))      # Non-normalized
  return y

def get_similarity_gauss(input_weight, item_weight):
  tolerance_width = 30
  # "width" grams of difference -> .37 similarity
  # "2*width" grams of difference -> .02 similarity
  y = gauss(x, item_weight, tolerance_width)
  return y

def get_similarity_sigmoid(input_weight, item_weight):
  tolerance_width = 30    # 30 grams of difference -> .5 similariy
  midpoint_1 = item_weight - tolerance_width
  midpoint_2 = item_weight + tolerance_width

  y = dsigmf(input_weight, midpoint_1, .2, midpoint_2, .2)
  return y

# Rodrigo's original proposal. Proportional difference.
def get_similarity_proportional(input_weight, item_weight):
  absolute_difference = abs(item_weight - input_weight)
  proportional_difference = absolute_difference / item_weight
  if proportional_difference > 1:
    proportional_difference = 1 - ( 1 / proportional_difference )
  similarity = 1 - proportional_difference
  return similarity

### ======================= End of similarity math


# For reference: http://wiki.ros.org/PyStyleGuide
class WeightEventsNode(object):

    def __init__(self, publish_rate=10):
        self.bin_a_items_ = []
        self.bins_bc_items_ = []
        self.tote_items_ = []
        return

    # Check how close input_weight is to that of a certain item
    # The closer the input weight to the reference weight, the closer the output should be to 1
    def _get_similarity(self, input_weight, item_id):
        try:
            item_weight_in_grams = self.item_weights_[item_id]*1000
        except KeyError, e:
            print 'Item is not supossed to be in the list:', e
            return 0.0 #item is not in list

        try:
            similarity = get_similarity_proportional(input_weight, item_weight_in_grams)
        except:
            rospy.logwarn("The weight for item " + item_id + " was not found. Assigning similarity: " + str(0))
            similarity = 0.0
        return similarity


    def _get_weight_change_since_ready(self, container_id):
        # Poll the sensor for the current weight (to be called after get_ready_to_pick)
        srvname = 'optoforce_' + container_id + '/get_weight_change_since_ready'
        rospy.wait_for_service(srvname)
        get_weight_change_since_ready_srv = rospy.ServiceProxy(srvname, GetWeightChangeSinceReady)
        response = GetWeightChangeSinceReadyResponse()
        try:
            response = get_weight_change_since_ready_srv()
        except rospy.ServiceException as exc:
            rospy.logerr("GetWeightChangeSinceReady for container " + container_id + "failed!")
            exc.what()
        return response.weight


    # Return other items in container whose weights are close to input_weight
    def _get_other_similar_items(self, input_weight, container_id):
        if container_id == "bin_A":
            items_in_container = self.bin_a_items_
        elif container_id == "bin_B" or container_id == "bin_C" or container_id == "bins_BC":
            items_in_container = self.bins_bc_items_
        elif container_id == "tote":
            items_in_container = self.tote_items_

        similar_items = dict()
        for item in items_in_container:
            similarity = self._get_similarity(input_weight, item)

            similar_items[item] = similarity
            
        return similar_items

    def recognize_items_callback(self, req):
        rospy.loginfo("callback recognize_items_callback called")
        rospy.loginfo("target_container: " + str(req.target_container.data))
        rospy.loginfo("target_item: " + str(req.target_item_id.data))

        rospy.loginfo("Start item classification")

        container_in_lower_case = str.lower(str(req.target_container.data))
        if container_in_lower_case == "bin_a":
            container_name_considers_combined = "bin_A"
        elif container_in_lower_case == "bin_b" or container_in_lower_case == "bin_c" or container_in_lower_case == "bins_bc":
            container_name_considers_combined = "bins_BC"
        elif container_in_lower_case == "tote":
            container_name_considers_combined = "tote"

        container_id = container_name_considers_combined
        target_item_id = req.target_item_id.data

        measured_item_weight = self._get_weight_change_since_ready(container_id)
        recognized_items = self._get_other_similar_items(measured_item_weight, container_id)
        recognized_items[str(target_item_id)] = self._get_similarity(measured_item_weight, target_item_id)


        response = RecognizeItemsResponse()
        for item_id, similarity in recognized_items.iteritems():
            response.items_ids.append(String(item_id))
            response.confidences.append(Float64(similarity))

        response.weight = measured_item_weight

        rospy.loginfo("The measured weight was " + str(measured_item_weight) + " and " + str(len(response.items_ids)) + " items were returned.")
        rospy.loginfo("Items_ids from weight sensing: %s",response.items_ids)
        rospy.loginfo("Confidences from weight sensing: %s",response.confidences)

        return response

    def set_items_info_callback(self, req):
        rospy.loginfo("callback set_items_info_callback called")
        self.item_weights_ = dict(zip([s.data for s in req.items_ids], [n.data for n in req.items_weights]))
        # To get the weight of an item: self.item_weights_["balloons"]

        rospy.loginfo("Received global item data")
        response = SetItemsInfoResponse()
        response.total_items.data = len(self.item_weights_)
        return response

    def set_items_location_callback(self, req):
        rospy.loginfo("callback set_items_location_callback called")
        self.bin_a_items_ = [s.data for s in req.bin_a_items]
        self.bins_bc_items_ = [s.data for s in req.bins_bc_items]
        self.tote_items_ = [s.data for s in req.tote_items]
        rospy.loginfo("Received container item locations")
        rospy.loginfo("bin_A "+str(len(self.bin_a_items_)))
        rospy.loginfo("bins_BC "+str(len(self.bins_bc_items_)))
        rospy.loginfo("tote "+str(len(self.tote_items_)))
        response = SetItemsLocationResponse()
        response.total_items.data = len(self.bin_a_items_) + len(self.bins_bc_items_) + len(self.tote_items_)

        return response

    def get_ready_for_pick_callback(self, req):
        rospy.loginfo("callback get_ready_for_pick_callback called'")
        rospy.loginfo("target_item_id: " + str(req.target_id.data))
        rospy.loginfo("target_container: " + str(req.target_container.data))

        container_in_lower_case = str.lower(str(req.target_container.data))

        if container_in_lower_case == "bin_a":
            container_name_considers_combined = "bin_A"
        elif container_in_lower_case == "bin_b" or container_in_lower_case == "bin_c" or container_in_lower_case == "bins_bc":
            container_name_considers_combined = "bins_BC"
        elif container_in_lower_case == "tote":
            container_name_considers_combined = "tote"
        else:
            rospy.logerr("get_ready_for_pick_callback: invalid container_in_lower_case " + str(container_in_lower_case))

        srvname = 'optoforce_' + container_name_considers_combined + '/get_ready_for_pick'
        rospy.wait_for_service(srvname)
        get_ready_for_pick = rospy.ServiceProxy(srvname, Empty)
        response = GetReadyForPickResponse()
        try:
            get_ready_for_pick()
            response.ready = True
            return response
        except rospy.ServiceException as exc:
            rospy.logerr("Get ready for pick for container " + str(req.target_container.data) + "failed!")
            exc.what()
            response.ready = False
            return response

    def start_ros(self):
        container_serial_numbers = dict()
        container_serial_numbers["CMU0A002"] = "bin_A"
        container_serial_numbers["NIH0A009"] = "tote"
        container_serial_numbers["NIH0A015"] = "bins_BC"

        optoforce_nodes = rospy.get_param("list_of_active_nodes")
        for node in optoforce_nodes:
            rospy.loginfo("Weight sensors under " + node + " are active.")

        # services we advertise
        rospy.Service('tnp_weight_events/get_ready_for_pick', GetReadyForPick, self.get_ready_for_pick_callback)
        rospy.Service('tnp_weight_events/recognize_items', RecognizeItems, self.recognize_items_callback)
        rospy.Service('tnp_weight_events/set_items_info', SetItemsInfo, self.set_items_info_callback)
        rospy.Service('tnp_weight_events/set_items_location', SetItemsLocation, self.set_items_location_callback)
        rospy.loginfo("Weight_events_node is ready")

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tnp_weight_events')
    try:
        node = WeightEventsNode()
        node.start_ros()
    except rospy.ROSInterruptException:
        pass
