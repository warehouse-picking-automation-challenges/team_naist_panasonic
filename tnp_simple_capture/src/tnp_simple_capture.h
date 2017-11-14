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

#ifndef SIMPLE_CAPTURE_H_
#define SIMPLE_CAPTURE_H_

#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// services
#include "tnp_simple_capture/capture.h"

#define DEPTH_SCALE 0.001

class SimpleCap
{
public:
  SimpleCap(ros::NodeHandle &nh);
  ~SimpleCap();
  bool setupROSConfig();
  std::string getImgSaveDir();
  void setImgSaveDir(std::string save_dir);

private:
  void allThere();

  // Topic subscribing callback
  void colorCallback(const sensor_msgs::ImageConstPtr &msg);
  void depthCallback(const sensor_msgs::ImageConstPtr &msg);
  void depthRegCallback(const sensor_msgs::ImageConstPtr &msg);
  void irCallback(const sensor_msgs::ImageConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
  // Service callback
  bool captureCallback(tnp_simple_capture::capture::Request &req,
                       tnp_simple_capture::capture::Response &res);

  // private variables
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber sub_color_;
  image_transport::Subscriber sub_depth_;
  image_transport::Subscriber sub_depth_reg_;
  image_transport::Subscriber sub_ir_;
  ros::Subscriber sub_camera_info_;

  ros::ServiceServer capture_service_;

  std::string img_save_dir_ = "/root/downloads_host/SimpleCapture/";
  const int font_face = cv::FONT_HERSHEY_DUPLEX;
  const double font_scale = 0.6;
  const int font_thick = 1;
  const int font_ltype = CV_AA;

  bool color_img_ishere_ = false;
  bool depth_img_ishere_ = false;
  bool depth_reg_img_ishere_ = false;
  bool ir_img_ishere_ = false;
  bool cam_info_ishere_ = false;

  cv::Mat color_img_;
  cv::Mat depth_img_;
  cv::Mat depth_reg_img_;
  cv::Mat ir_img_;
  cv::Mat depth_8u_save_;
  cv::Mat depth_reg_8u_save_;
  sensor_msgs::CameraInfo cam_info_;

  std::vector<double> depth_max_vals_;
  std::vector<double> depth_reg_max_vals_;
  int save_count_color_;
  int save_count_depth_;
  int save_count_depth_reg_;
  int save_count_ir_;
};

static cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &msg, const std::string type);

#endif /* SIMPLE_CAPTURE_H_ */
