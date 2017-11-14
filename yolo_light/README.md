# TNP version YOLO Light Guidance

This package contains three main tools:
* A ROS node to perform object recognition by using a trained YOLO (v2) model and a RGB stream from a R200/SR300 camera.
* A Python script to train a YOLO (v2) model with TensorFlow on GPU using annotated images.
* An Image Generator generate multiple object with background images. Details in ./scripts/ImageGenerator/README.md

This package is a modification of YOLO Light (https://github.com/chrisgundling/yolo_light), which is a ROS implementation of DarkFlow (https://github.com/thtrieu/darkflow), which is itself a TensorFlow implementation of Darknet (https://github.com/pjreddie/darknet), the CNN required to train the YOLO model from annotated images (https://pjreddie.com/darknet/yolo/). Before pursuing, please read the documentation of behind these links to get the big picture. Also, please confirm the required dependencies.

# Main usages

**Attention (1):** By default, the current implementation is configured to work with the 20 classes of the VOC 2012 challenge and the Tiny YOLOv2 configuration. You may need to download the VOC 2012 dataset to (re)train the model: http://host.robots.ox.ac.uk/pascal/VOC/voc2012/index.html#devkit. See path options in `./scripts/darkflow.py` (ROS detection) and `./scripts/flow` (TensorFlow training).

## Detecting objects with ROS

Inputs (see `./scripts/darkflow.py`):
* The configuration of the trained model/network, as a .cfg file.
* The trained model, as a TensorFlow Checkpoint (4 files).
* A RGB stream of a R200/SR300 camera as a subscribed topic.

Outputs (see `./scripts/tlight_node.py`):
* A vector containing the detected objects' classes & ID, confidence and bounding box information as a published topic.
* A labeled image stream as a published topic (mainly to use for debug with rqt_image_view).

Use the launch files/nodes of the TNP Vision package for a quick demo:
* `roslaunch tnp_vision tnp_yolo_manager_r200.launch`
* `roslaunch tnp_vision tnp_yolo_manager_sr300.launch`

## Training a model with TensorFlow

Inputs (see `./scripts/flow_ARC2017Settings`):
* Annotated images in VOC-PASCAL (.xml) format. Attention: The images and their annotations must be in two separate subfolders inside the same folder.
* The configuration of the model/network, as a .cfg file (the same .cfg will be used as an input for the ROS detection).
* Optional: A pre-trained YOLO model, as a Darknet .weight format or as a TensorFlow Checkpoint format, to start the training from existing weights (pre-trained model or backup).

Output (see `./scripts/flow`):
* A trained model as a TensorFlow Checkpoint (4 files).

To run the training, first configure the options in `./scripts/flow` then go to `./scripts/` and execute `./flow --train`. Refer to the documentation of DarkFlow for more options: https://github.com/thtrieu/darkflow.

**NOTE:**

Before you training with your model. Please check `./scripts/cfg/*.cfg`

* When you doing training with your own setups of Network structure, you need to carefully check if the "classes" settings in the structure file.
* The classes amount and filters amount in last convolutional layer are related parameters. The filters amount should be ` filters = num * (classes + coords + 1 ) `


Please check `./scripts/cfg/*.cfg` and `./scripts/labels.txt`

* The classes amount in label.txt should be as same as defined in .cfg file.

About Training data

* When you doing trying with yolo structure that is default provided in the folder, this program will automatically use their default classes even you put a label.txt file. It is highly recommended choose a new file name by yourself.
Please check `./scripts/yolo/misc.py`, their are lists describe about it.


# Troubleshooting

* Before being able to use the Python code, you may need to deploy it with Cython. Please refer to https://github.com/chrisgundling/yolo_light#getting-things-running.
* After changing the model and/or labels, you will have to delete the cached labels in `./scripts/net/yolo/parse-history.txt`.
* If you experience memory issues on the GPU, please reduce the GPU load (scale from 0.0 to 1.0 in `./scripts/flow` and/or `./scripts/darkflow.py`) or use a smaller model/network such as Tiny YOLOv2.
* YOLOv2 is also commonly referred as YOLO-VOC (a COCO version exists too).
* darkflow is currently not ready for YOLO-9000 for classification, it can currently only be used as a detector.

# Future work

* Add documentation about TensorBoard to monitor the training.
* Train the model on multiple GPUs.
