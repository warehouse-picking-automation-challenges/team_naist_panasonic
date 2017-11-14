import tensorflow as tf
import math
from IPython.terminal.debugger import set_trace as keyboard

def _remap(self,OldValue,OldMax,OldMin,NewMax,NewMin):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue


def fit_original_image(self,image,pred_from_net):
    orig_height = float(image.shape[0])
    orig_width = float(image.shape[1])

    #Reposition the point to fit input image
    x = int(self._remap(pred_from_net[0],0.9,-0.9,1,0)*224)
    y = int(self._remap(pred_from_net[1],0.9,-0.9,1,0)*224)
    x = int(self._remap(x,224,0,orig_width,0))
    y = int(self._remap(y,224,0,orig_height,0))

    #Get the points that defines grasp width and grasp angle in 224x224 image
    #rad = self._remap(pred_from_net[2],0.9,-0.9,3.14,0)
    sin2rad = self._remap(pred_from_net[2],0.9,-0.9,1.0,-1.0)
    cos2rad = self._remap(pred_from_net[3],0.9,-0.9,1.0,-1.0)
    twice_rad = math.atan2(cos2rad,sin2rad)
    rad = twice_rad / 2.0

    width = self._remap(pred_from_net[4],0.9,-0.9,224,0)
    p2 = (int(x+(width/2)*math.sin(rad)),int(y+(width/2)*math.cos(rad)))
    p1 = (int(x-(width/2)*math.sin(rad)),int(y-(width/2)*math.cos(rad)))
    #Do shifting and rotating to grasping since we have a resize image
    change_ratio = [float(orig_width/224),float(orig_height/224)]
    p1_shifted = [change_ratio[0]*p1[0],change_ratio[1]*p1[1]]
    p2_shifted = [change_ratio[0]*p2[0],change_ratio[1]*p2[1]]

    #Recalculate shifted grasp line angles
    shift_p1_p2_displacement_x = p2_shifted[0]-p1_shifted[0]
    shift_p1_p2_displacement_y = p2_shifted[1]-p1_shifted[1]
    rad_resized = math.atan2(shift_p1_p2_displacement_x,shift_p1_p2_displacement_y)
    sin2rad_resized = math.sin(2*rad_resized)
    cos2rad_resized = math.cos(2*rad_resized)


    #Recalculate shifted grasp line length
    width_resized = math.sqrt(shift_p1_p2_displacement_x**2 + shift_p1_p2_displacement_y**2)

    real_pred = [x,y,sin2rad_resized,cos2rad_resized,width_resized]
    return real_pred

def fit_original_image_Rad(self,image,pred_from_net):
    orig_height = float(image.shape[0])
    orig_width = float(image.shape[1])

    #Reposition the point to fit input image
    x = int(self._remap(pred_from_net[0],0.9,-0.9,orig_width,0))
    y = int(self._remap(pred_from_net[1],0.9,-0.9,orig_height,0))

    #Get the points that defines grasp width and grasp angle in 224x224 image
    rad = self._remap(pred_from_net[2],0.9,-0.9,3.14,-3.14)

    width = self._remap(pred_from_net[3],0.9,-0.9,224,0)
    p2 = (int(x+(width/2)*math.sin(rad)),int(y+(width/2)*math.cos(rad)))
    p1 = (int(x-(width/2)*math.sin(rad)),int(y-(width/2)*math.cos(rad)))
    #Do shifting and rotating to grasping since we have a resized image

    change_ratio = [float(orig_width/224),float(orig_height/224)]
    p1_shifted = [change_ratio[0]*p1[0],change_ratio[1]*p1[1]]
    p2_shifted = [change_ratio[0]*p2[0],change_ratio[1]*p2[1]]

    #Recalculate shifted grasp line angles
    shift_p1_p2_displacement_x = p2_shifted[0]-p1_shifted[0]
    shift_p1_p2_displacement_y = p2_shifted[1]-p1_shifted[1]
    rad_resized = math.atan2(shift_p1_p2_displacement_x,shift_p1_p2_displacement_y)


    #Recalculate shifted grasp line length
    width_resized = math.sqrt(shift_p1_p2_displacement_x**2 + shift_p1_p2_displacement_y**2)

    real_pred = [x,y,rad_resized,width_resized]
    return real_pred
