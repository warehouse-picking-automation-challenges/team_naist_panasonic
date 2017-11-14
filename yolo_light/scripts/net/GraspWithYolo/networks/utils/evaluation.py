import cv2
import numpy as np
import math

PI = 3.141592653589793

def calculate_iou(orig_image,rectangle1,rectangle2):
    h,w,_ = orig_image.shape
    black_img = np.zeros([h, w], np.uint8)


def success_evaluate(pred,teach,d_distance_accept=0.50,d_angle_accept=70,d_width_accept=0.25):
    if len(pred) > 4:
        print "Warning! Your prediction is more than 4 elements, calculation might wrong"

    #Check Angle
    d_rad_accept = d_angle_accept * PI / 180.
    rad_p = pred[2]
    rad_t = teach[2]
    d_rad = math.fabs(rad_p - rad_t)

    if not d_rad_accept > d_rad:
        return False

    #Check Distance
    x_p=pred[0]
    y_p=pred[1]
    x_t=teach[0]
    y_t=teach[1]
    distance = math.sqrt((x_t-x_p)**2+(y_t-y_p)**2)

    if not d_distance_accept > distance:
        return False

    #Check Width
    w_p = pred[3]
    w_t = teach[3]
    d_w = w_p - w_t
    d_w_percent = math.fabs(d_w/w_t)

    if not d_width_accept > d_w_percent:
        return False
    return True
