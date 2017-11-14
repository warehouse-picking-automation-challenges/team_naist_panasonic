#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import logging
import optoforce
import serial # required to handle exceptions raised in the optoforce module
from rospkg import RosPack
import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
import operator

from signal_filters import filter_factory

from tnp_optoforce.srv import *

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Float64, Float64MultiArray

PKG_PATH = RosPack().get_path("tnp_optoforce")

class OptoforceNode(object):
    """
    ROS interface for Optoforce sensors
    """

    def __init__(self, *args, **kwargs):
        """
        Initialize OptoforceDriver object
        """
        # 0 = x, 1 = y, 2 = z
        # self._force_index = rospy.get_param("~force_index")
        self._force_index = 2    # Probably added by James The Programmer

        port = rospy.get_param("~port", "/dev/ttyACM0")
        sensor_type = rospy.get_param("~type", "m-ch/3-axis")
        starting_index = rospy.get_param("~starting_index", 0)
        scaling_factors = rospy.get_param("~scale")

        serial_number_received = False
        while not serial_number_received and not rospy.is_shutdown():
            try:
                # Initialize optoforce driver
                self._initialize_driver(port, sensor_type, scaling_factors, starting_index)

                # Get filter settings
                filter_id = rospy.get_param("~filter_id", "running_avg")
                # Use <rosparam> tag to define a dictionary in the parameter server
                filter_kwargs = rospy.get_param("~filter_kwargs", {"window_size": 50})
                # Construct filters
                self._sensor_values_before_pick = []
                self._sensor_signal_filters = []
                for i in range(self._driver.nb_sensors()):
                    self._sensor_signal_filters.append(filter_factory(filter_id, **filter_kwargs))
                    self._sensor_values_before_pick.append(0.0)

                # Read parabolic adjustment factors
                self._serial_number = self._read_serial_number()
                self._parabolic_adjustment_factors = self._read_parabolic_adjustment_file(self._serial_number)
                serial_number_received = True
            except:
                print("Something went wrong during an Optoforce sensor startup. Trying again.")
                rospy.sleep(2)


        # Advertise the services
        rospy.Service('report_serial_number', ReportSerialNumber, self.report_serial_number_callback)
        rospy.Service('get_ready_for_pick', Empty, self.get_ready_for_pick_callback)
        #rospy.Service('initialize_calibration', InitializeCalibration, self.initialize_calibration_callback)
        rospy.Service('get_weight_change_since_ready', GetWeightChangeSinceReady, self.get_weight_change_since_ready_callback)
        #rospy.Service('zero_sensors', ZeroSensors, self.zero_sensors_callback)
        #rospy.Service('add_calibration_measurement', AddCalibrationMeasurement, self.add_calibration_measurement_callback)
        #rospy.Service('finish_calibration', FinishCalibration, self.finish_calibration_callback)


        # Deal with publishing
        self._publish_rate = 100
        self._pub_adjusted_weight = rospy.Publisher('adjusted_weight', Float64, queue_size=100)
        self._pub_sensor_z_sum = rospy.Publisher('sensor_z_sum', Float64, queue_size=100)

    def config(self):
        """
        Set the sensor's configuration based on parameters.

        The options to configure are speed (frequency at which data is
        delivered), filter frequency and zeroing of the values. The related
        ros param names are ~speed, ~filter and ~zero.
        """
        speed = rospy.get_param("~speed", "100Hz")
        filter_hz = rospy.get_param("~filter", "1.5Hz")
        zero = rospy.get_param("~zero", "false")

        self._driver.config(speed, filter_hz, zero)

    def run(self):
        """
        Runs the read loop.

        It listens to the serial port and publishes all force data it receives.
        """
        rospy.loginfo("Starting to listen to the sensor")
        while not rospy.is_shutdown():
            data = self._driver.read()
            if isinstance(data, optoforce.OptoforceData):
                self._filter_data(data)
                self._publish()
            elif isinstance(data, optoforce.OptoforceSerialNumber):
                self._serial_number = str(data)
                rospy.loginfo("The sensor's serial number is " + str(data))

    def report_serial_number_callback(self, req):
        response = ReportSerialNumberResponse()
        response.serial_number = self._serial_number
        return response

    def get_ready_for_pick_callback(self, req):
        self._sensor_values_before_pick = self._fetch_filter_responses()
        return EmptyResponse()
        
    def get_weight_change_since_ready_callback(self, req):
        sensor_values_after_pick = self._fetch_filter_responses()
        before_after = zip(self._sensor_values_before_pick, sensor_values_after_pick)
        sensor_value_deltas = [abs(operator.sub(before, after)) for (before, after) in before_after]

        response = GetWeightChangeSinceReadyResponse()
        response.weight = self._get_adjusted_weight_change(sensor_value_deltas)    # This is the parabola-adjusted method
        #response.weight = sum(sensor_value_deltas)    # This is the simple sum of all the sensors
        return response

    # Returns weight for 4 sensor values, each obtained by recording the difference between before and after the item was removed.
    # This is only inside the Optoforce node because it holds the coefficients for this particular sensor arrangement.
    def _get_adjusted_weight_change(self, sensor_value_deltas):
        """
        Explanation from Yamamoto:
        # We start with: (F meaning weight)
        # F = a1*x+a2*x^2+a3*y+a4*y^2+a5
        # where  -1 < x < 1,  and   -1 < y < 1.
        # In real measurement, x and y cannot directly be measured. So we estimate using following equations:
        #   x = -1+2*(F1+F3)/(F1+F2+F3+F4)
        #   y = -1+2*(F3+F4)/(F1+F2+F3+F4)
        # In order to calculate matrix A=[a1,a2,a3,a4,a5],
        # we take as many data as possible and makes X, Y, F vectors.
        # Then A can be calculated by multiple regression:
        #   A = F * pinv([X X.^2 Y Y.^2 1])
        #  Open questions:
        #  Is each sensor calibrated before this separately?
        """

        s = sensor_value_deltas
        x = -1+2*(s[0]+s[2])/((s[0]+s[1]+s[2]+s[3])+1e-6) # the +1e-6 avoid division by zero
        y = -1+2*(s[2]+s[3])/(s[0]+s[1]+s[2]+s[3]+1e-6) # the +1e-6 avoid division by zero

        a = self._parabolic_adjustment_factors
        adjusted_coefficient  = a[0]*x + a[1]*x*x + a[2]*y + a[3]*y*y + a[4]
        adjusted_weight = (s[0]+s[1]+s[2]+s[3])/(adjusted_coefficient + 1e-6)
        # print("Adjusted weight calculated using x = " + str(x) + ", y = " + str(y))
        return adjusted_weight

    def _filter_data(self, data):
        for i, signal_filter in enumerate(self._sensor_signal_filters):
            signal_filter.filter(data.force[i][self._force_index])

    def _fetch_filter_responses(self):
        responses = []
        for signal_filter in self._sensor_signal_filters:
            responses.append(signal_filter.calculate_response())
        return responses

    def _read_serial_number(self):
        self._driver.request_serial_number() 
        serialInfo = self._driver.read()
        if isinstance(serialInfo, optoforce.OptoforceSerialNumber): 
            rospy.loginfo("The sensor's serial num is " + str(serialInfo))
            return str(serialInfo)
        else:
            raise RuntimeError("Driver did not return a serial number.")

    def _read_scale_factor_file(self, prefix):
        scaling_factor_file = prefix+"-calibration-data.yaml"
        with open(PKG_PATH + "/config/" + scaling_factor_file, "r") as filestream:
            docs = yaml.load_all(filestream)
            return docs.next()["scale"]

    def _read_parabolic_adjustment_file(self, prefix):
        parabolic_adjustment_factors_file = prefix+"-parabolic-adjustment-factors.yaml"
        with open(PKG_PATH + "/config/" + parabolic_adjustment_factors_file, "r") as filestream:
            docs = yaml.load_all(filestream)
            return docs.next()["parabolic_adjustment_factors"]

    def _initialize_driver(self, port, sensor_type, scaling_factors, starting_index):
        try:
            self._driver = optoforce.OptoforceDriver(port,
                                                     sensor_type,
                                                     scaling_factors,
                                                     starting_index)
        except serial.SerialException as e:
            rospy.logfatal("Cannot connect to the sensor " + port
                           + (e.message if e.message else ''))
            rospy.signal_shutdown("Serial connection failure")
            raise

    def _publish(self):
        rate= rospy.Rate(self._publish_rate) 

        if not self._sensor_values_before_pick:
            self._pub_adjusted_weight(0.0)
        else:
            # These three lines are copied from self.get_weight_change_since_ready_callback
            sensor_values_after_pick = self._fetch_filter_responses()
            before_after = zip(self._sensor_values_before_pick, sensor_values_after_pick)
            sensor_value_deltas = [abs(operator.sub(before, after)) for (before, after) in before_after]
            self._pub_adjusted_weight.publish(self._get_adjusted_weight_change(sensor_value_deltas))

        sensor_values = self._fetch_filter_responses()
        self._pub_sensor_z_sum.publish(sum(sensor_values))

class ConnectPythonLoggingToROS(logging.Handler):
    """
    Class interfacing logs using Python's standard logging facility and ROS's
    way of logging. It is meant to be used as a Handler for the logging module.

    Source: https://gist.github.com/nzjrs/8712011
    """

    MAP = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))

if __name__ == '__main__':
    rospy.init_node("optoforce")
    try:
        node = OptoforceNode()
    except Exception as e:
        rospy.logfatal("Caught exception: " + str(e))
    else:
        #reconnect logging calls which are children of this to the ros log system
        logging.getLogger('optoforce').addHandler(ConnectPythonLoggingToROS())
        #logs sent to children of trigger with a level >= this will be redirected to ROS
        logging.getLogger('optoforce').setLevel(logging.DEBUG)

        node.config()
        node.run()
