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

#ifndef SRC_TNP_MONITOR_H_
#define SRC_TNP_MONITOR_H_

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tnp_msgs/StateInfo.h>
#include <tnp_msgs/TaskList.h>

class Monitor {
public:
  Monitor(ros::NodeHandle &nh);
  void monitorPanelInit();

private:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_yolo_color_img_;
  image_transport::Subscriber sub_yolo_depth_img_;
  image_transport::Subscriber sub_rs_camc_color_img_;
  image_transport::Subscriber sub_rs_caml_color_img_;
  image_transport::Subscriber sub_rs_camr_color_img_;
  image_transport::Subscriber sub_rs_camb_color_img_;
  image_transport::Subscriber sub_rs_camc_bb_img_;
  image_transport::Subscriber sub_rs_caml_bb_img_;
  image_transport::Subscriber sub_rs_camr_bb_img_;
  image_transport::Subscriber sub_rs_camb_bb_img_;

  ros::Subscriber sub_status_msg_;
  ros::Subscriber sub_status_state_;
  ros::Subscriber sub_status_run_;
  ros::Subscriber sub_status_task_;

  bool yolo_color_img_ishere_ = false;
  bool yolo_depth_img_ishere_ = false;
  bool rs_camc_color_img_ishere_ = false;
  bool rs_caml_color_img_ishere_ = false;
  bool rs_camr_color_img_ishere_ = false;
  bool rs_camb_color_img_ishere_ = false;
  bool rs_camc_bb_img_ishere_ = false;
  bool rs_caml_bb_img_ishere_ = false;
  bool rs_camr_bb_img_ishere_ = false;
  bool rs_camb_bb_img_ishere_ = false;
  bool item_target_ishere_ = false;
  bool item_picked_ishere_ = false;
  bool status_msg_ishere_ = false;
  bool status_state_ishere_ = false;
  bool status_run_ishere_ = false;
  bool status_task_ishere_ = false;

  bool round_started_ = false;

  bool status_run_ = false;
  cv::Mat yolo_color_img_;
  cv::Mat yolo_depth_img_;
  cv::Mat rs_camc_color_img_;
  cv::Mat rs_caml_color_img_;
  cv::Mat rs_camr_color_img_;
  cv::Mat rs_camb_color_img_;
  cv::Mat rs_camc_bb_img_;
  cv::Mat rs_caml_bb_img_;
  cv::Mat rs_camr_bb_img_;
  cv::Mat rs_camb_bb_img_;
  std::string status_msg_;
  std::string state_now_;
  std::string state_prev_1_ = "Initial state";
  std::string state_prev_2_ = "Initial state";
  std::string state_prev_3_ = "Initial state";
  std::string state_prev_4_ = "Initial state";
  std::vector<std::string> state_next_ = {"", "", "", ""};
  std::vector<std::string> status_task_ = {"Task pending"};

  enum NodeState {
    SHOW_REC_SPACE_CAM,
    SHOW_REC_SPACE_BB
  };
  NodeState node_state_ = SHOW_REC_SPACE_CAM;

  long round_start_ = 0;
  long state_start_ = 0;
  long state_time_0_ = 0;
  long state_time_1_ = 0;
  long state_time_2_ = 0;
  long state_time_3_ = 0;
  long state_time_4_ = 0;
  long disp_start_ = 0;
  long disp_time_ = 10000;

  const int font_face_ = cv::FONT_HERSHEY_DUPLEX;
  const double font_scale_ = 0.6;
  const int font_thick_ = 1;
  const int font_ltype_ = CV_AA;

  cv::Mat monitor_;
  cv::Size tile_size_;
  cv::Size thumb_size_;
  const int tile_width_ = 640;
  const int tile_height_ = 480;
  const int padding_left_ = 35;
  const int padding_top_ = 70;
  int log_height_;

  void yoloColorCallback(const sensor_msgs::ImageConstPtr &msg);
  void yoloDepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamCColorCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamLColorCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamRColorCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamBColorCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamCBbCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamLBbCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamRBbCallback(const sensor_msgs::ImageConstPtr &msg);
  void rsCamBBbCallback(const sensor_msgs::ImageConstPtr &msg);
  void itemTargetCallback(const std_msgs::StringConstPtr &msg);
  void itemPickedCallback(const std_msgs::StringConstPtr &msg);
  void statusMessageCallback(const std_msgs::StringConstPtr &msg);
  void statusStateCallback(const tnp_msgs::StateInfo &msg);
  void statusRunCallback(const std_msgs::BoolConstPtr &msg);
  void statusTaskCallback(const tnp_msgs::TaskList &msg);

  void monitorPanelCell();
  void monitorPanelDisp();
};

static cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &msg, const std::string type);

#endif /* SRC_TNP_MONITOR_H_ */
