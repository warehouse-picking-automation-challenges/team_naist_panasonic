#!/usr/bin/env python
import configparser
from IPython.core.debugger import Tracer; keyboard = Tracer()
from net.build import TFNet
import cv2
import rospy
from tlight_node import TLightNode
import argparse
import os


def checkload(input):
    """
    Check if the load input is integer or not. If int return int, if not return string
    """
    try:
        return int(input)
    except ValueError:
        return input

def load_config(path,UseModel=''):
    """
    Load the config file from path.
    """
    inifile = configparser.ConfigParser()
    if len(inifile.read(path)) == 0:
        rospy.logerr("Nothing read from config file!")
        exit()
    else:
        rospy.loginfo("Read Config File Successfully.")
        rospy.loginfo("You have following Model Setting can be used:")
        rospy.loginfo(inifile.sections())

    if UseModel=='': UseModel=inifile['DEFAULT']['use_as_default']
    detect_threshold = inifile['DEFAULT']['threshold']
    model_path = inifile[UseModel]['model_path']
    backup_path = inifile[UseModel]['backup_path']
    label_file = inifile[UseModel]['label_file']
    load = inifile[UseModel]['load']
    grasp_backup_path = inifile[UseModel]['grasp_backup_path']
    grasp_net_name = inifile[UseModel]['grasp_net_name']
    return model_path,backup_path,label_file,load,detect_threshold,grasp_backup_path,grasp_net_name

def process(model, img):
    result = model.return_predict(img[None, :, :, :])
    return result

def get_model():
    config_path = "./darkflow_config.ini"
    config_path = get_right_path(config_path)
    model_path,backup_path,label_file,load,detect_threshold,grasp_backup_path,grasp_net_name = load_config(config_path)
    model_path = get_right_path(model_path)
    backup_path = get_right_path(backup_path)
    label_file = get_right_path(label_file)
    grasp_backup_path = get_right_path(grasp_backup_path)
    load = checkload(load)
    detect_threshold = float(detect_threshold)
    print("Model Files in:" + model_path)
    options = {"model": model_path, "backup": backup_path,
    			"load": load, "gpu": 0.5, "label": label_file,
    			"threshold": detect_threshold ,
    			"pred_grasp" : True,
                "pred_grasp_crop_percent": 1.1, # Original trained data value: 1.1
    			"grasp_backup": grasp_backup_path,
				"grasp_net_name": grasp_net_name
    			}
    model = TFNet(options)
    return model

def get_right_path(path):
    if path.startswith("./") or path.startswith("../"):
        process_folder_path = os.path.split(os.path.realpath(__file__))[0]
        path = os.path.realpath(os.path.join(process_folder_path,path))
        return path
    else:
        return path

def main():
    import platform
    rospy.loginfo("Python Version in Darkflow: "+platform.python_version())
    rospy.loginfo("Starting Darkflow")
    node = TLightNode(lambda: get_model(), process)
    rospy.loginfo("TLight Node Start Completed!")
    rospy.loginfo("Python Version in Darkflow: "+platform.python_version())
    rospy.spin()


if __name__ == '__main__':
    main()
