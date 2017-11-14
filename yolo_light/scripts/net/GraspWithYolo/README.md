# GraspWithYolo Guidelines

**GraspWithYolo** is a compact Deep Learning based  that could predict grasp from raw images cropped from Yolo. It is a SISO system constructed with Tensorflow by PIN-CHU, YANG. It is simple to utilize and provide relatively reliable grasp prediction through RGB image(s).

**Required & Workable Packages**
----
* tensorflow 1.2.0
* matplotlib 2.0.2
* pillow 4.2.1
* ipython 6.1.0
* numpy 1.13.0


Using for Prediction
---

In order to predict the grasp, you will need to do following procedures:

1.  Put the **GraspWithYolo** folder under your script
2.  Add `from .GraspWithYolo.GraspWithYolo import GraspWithYolo` to your python code
3.  Initialize the class with path of your checkpoint backup and the network name you used during training
4.  Input with numpy arrayed PIL image
5.  Get the prediction correspond to original image coordination

Here is an example:  

```
import tensorflow as tf
import numpy as np
from .GraspWithYolo.GraspWithYolo import GraspWithYolo
from PIL import Image

path_backup = "/path/to/trained/checkpoint/folder/"
name_of_net = "NAME_OF_YOUR_NET_WHEN_YOU_USED_DURING_TRAINING"
load = -1 #[OPTIONAL] The checkpoint(weights) you want to used, -1 represent to latest trained weights
gpu_mode = "cpu" #[OPTIONAL] You can use 0,1,2 to use gpu if you have gpu cards in your device

image = Image.open("./test.png")
image = np.asarray(image)

GraspPredictNet = GraspWithYolo(path_backup,name_of_net,load,gpu_mode) #Initialize GraspWithYolo

#grasp_pred = [grasp_x_center,grasp_y_center,rotation(rad),width_of_grasp]
grasp_pred = GraspPredictNet.predict(image) #Predict with image

print(str(grasp_pred)) #Print out the result

```


Training
---

In order to train GraspWithYolo, you need to do following procedures:

1. Prepare training data sets contains images and grasps files
2. [OPTIONAL]Prepare testing data
3. Execute `run.py --train --images /IMAGES/FOLDER --grasp /GRASPS/FOLDER/` for basic training function

It is recommended to use shell scripted train file to proceed the training. Here is an example:

```
FolderRoot="./GraspWithYoloResult/"
seed=1234
name_of_net="Model"
AdditionalDescription="Train1"

ResultRoot=$FolderRoot"TrainingResult/"
mkdir $ResultRoot
ResultFolder=$ResultRoot$name_of_net"_seed"$seed$AdditionalDescription"/"
mkdir $ResultFolder

SummaryFolder=$ResultFolder"summary/"
mkdir $SummaryFolder

Ckpt=$ResultFolder"ckpt/"
mkdir $Ckpt


python ../run.py --train "True"\
              --images "../data/images-train/" \
              --grasp "../data/grasp-train/" \
              --images_test "../data/images-test/" \
              --grasp_test "../data/grasp-test/" \
              --test_output $ResultFolder"test_output/" \
              --summary $SummaryFolder \
              --load "" \
              --backup $Ckpt  \
              --lr "0.001" \
              --seed $seed \
              --keep "50" \
              --save "1000" \
              --epoch "5000" \
              --batch "50" \
              --name $name_of_net \
              --gpu_mode "0"
```




Other
---

If you trying to input with cv2 numpy array, you will need to consider with transformation between PIL image array arrangement and cv2 image array arrangement.

The most simplest is to apply following function to your cv2 array then you can get correct input

```
def cv22PIL(imgcv2):
    imagePIL = imgcv2
    imagePIL = imgcv2[::-1, :, ::-1].copy() # BGR->RGB
    imagePIL = imgcv2[:,:,::-1].copy() #flip image
    imagePIL = Image.fromarray(imagePIL)
    return imagePIL
```
