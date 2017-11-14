/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

#ifndef SRC_TNP_YOLO_MANAGER_H_
#define SRC_TNP_YOLO_MANAGER_H_

#include "tnp_deep_vision/recognize_items.h"

#include <yolo_light/ImageDetections.h>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>

#define ROI_SIZE 20
#define DEPTH_SCALE 0.001
#define TXT_HEIGHT 15.0

class YoloManager {
public:
  YoloManager(ros::NodeHandle &nh);

private:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_color_;
  image_transport::Subscriber sub_depth_;
  image_transport::Publisher pub_color_;
  image_transport::Publisher pub_depth_;

  ros::Subscriber sub_detections_;
  ros::Subscriber sub_target_item_;
  ros::Subscriber sub_camera_info_;
  ros::ServiceServer srv_classify_with_dl;

  const int font_face = cv::FONT_HERSHEY_DUPLEX;
  const double font_scale = 0.6;
  const int font_thick = 1;
  const int font_ltype = CV_AA;
  const std::string str_frame_id = "tnp_ee_camera_frame";
  const double THRES_MIN_DEPTH = 100.0;
  const double THRES_MAX_DEPTH = 1500.0;

  bool yolo_det_ishere_ = false;
  bool color_img_ishere_ = false;
  bool depth_img_ishere_ = false;
  bool target_item_ishere_ = false;
  bool cam_info_ishere_ = false;
  bool is_recognizing_ = false;

  yolo_light::ImageDetections yolo_det_;
  cv::Mat color_img_;
  cv::Mat depth_img_;
  sensor_msgs::CameraInfo cam_info_;
  std::string target_item_;

  std::vector<std_msgs::String> recog_classes_;
  std::vector<geometry_msgs::PoseStamped> recog_poses_;
  std::vector<std_msgs::Float64> recog_confidences_;

  geometry_msgs::PoseStamped calculatePose(yolo_light::Detection det, int poseType);

  void detectionsCallback(const yolo_light::ImageDetections &msg);
  void colorCallback(const sensor_msgs::ImageConstPtr &msg);
  void depthCallback(const sensor_msgs::ImageConstPtr &msg);
  void targetItemCallback(const std_msgs::StringConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
  bool recognize_items(tnp_deep_vision::recognize_items::Request &req, tnp_deep_vision::recognize_items::Response &res);
  void allThere();
};

static cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &msg, const std::string type);

#endif /* SRC_TNP_YOLO_MANAGER_H_ */
