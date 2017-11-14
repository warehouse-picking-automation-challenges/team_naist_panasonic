Add simple camera image capture package.
This package displays color and registered depth images with grid lines to check misalignment between them.

Usage:
'''
rosrun simple_capture simple_capture
rosservice call /simple_capture/capture [typeToSave] [saveFileName]
'''
typeToSave: color (for RGB image), depth (for depth raw image), depth_reg (for registered depth image), ir (for IR image)
Images will be saved to /root/downloads_host/SimpleCapture/saveFileName_typeToSave_#.png