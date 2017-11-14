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

#include "tnp_monitor.h"

using namespace std;

/*
 * Converts ros images to openCV images, copyies the images
 * type as sensor_msgs::image_encodings::BGR8|TYPE_16UC1.
 */
cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &img, const std::string type)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, type);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("Image converter, cv_bridge exception: %s", e.what());
  }
  return cv_ptr;
}

Monitor::Monitor(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
  sub_status_msg_ = nh_.subscribe("/tnp_simple_state_machine/status_message", 1, &Monitor::statusMessageCallback, this);
  sub_status_state_ = nh_.subscribe("/tnp_task_manager/state_machine_info", 1, &Monitor::statusStateCallback, this);
  sub_status_run_ = nh_.subscribe("/tnp_task_manager/running_status", 1, &Monitor::statusRunCallback, this);
  sub_status_task_ = nh_.subscribe("/tnp_task_manager/task_msgs", 1, &Monitor::statusTaskCallback, this);
  sub_yolo_color_img_ = it_.subscribe("/tnp_deep_vision/color_img", 1, &Monitor::yoloColorCallback, this);
  sub_yolo_depth_img_ = it_.subscribe("/tnp_deep_vision/depth_img", 1, &Monitor::yoloDepthCallback, this);
  sub_rs_camc_color_img_ = it_.subscribe("/camera_C/color/image_raw", 1, &Monitor::rsCamCColorCallback, this);
  sub_rs_caml_color_img_ = it_.subscribe("/camera_L/color/image_raw", 1, &Monitor::rsCamLColorCallback, this);
  sub_rs_camr_color_img_ = it_.subscribe("/camera_r/color/image_raw", 1, &Monitor::rsCamRColorCallback, this);
  sub_rs_camb_color_img_ = it_.subscribe("/camera_B/color/image_raw", 1, &Monitor::rsCamBColorCallback, this);
  sub_rs_camc_bb_img_ = it_.subscribe("/tnp_cloud_matching/bounding_box_C", 1, &Monitor::rsCamCBbCallback, this);
  sub_rs_caml_bb_img_ = it_.subscribe("/tnp_cloud_matching/bounding_box_L", 1, &Monitor::rsCamLBbCallback, this);
  sub_rs_camr_bb_img_ = it_.subscribe("/tnp_cloud_matching/bounding_box_R", 1, &Monitor::rsCamRBbCallback, this);
  sub_rs_camb_bb_img_ = it_.subscribe("/tnp_cloud_matching/bounding_box_B", 1, &Monitor::rsCamBBbCallback, this);
}

void Monitor::monitorPanelCell()
{
  // Draw tile borders.
  cv::line(monitor_, cv::Point(tile_width_ * 0, tile_height_ * 1 - 1), cv::Point(tile_width_ * 2, tile_height_ * 1 - 1), cv::Scalar(255, 255, 255), 2);
  cv::line(monitor_, cv::Point(tile_width_ * 0, tile_height_ * 2 - 1), cv::Point(tile_width_ * 3, tile_height_ * 2 - 1), cv::Scalar(255, 255, 255), 2);
  cv::line(monitor_, cv::Point(tile_width_ * 1 - 1, tile_height_ * 0), cv::Point(tile_width_ * 1 - 1, tile_height_ * 2), cv::Scalar(255, 255, 255), 2);
  cv::line(monitor_, cv::Point(tile_width_ * 2 - 1, tile_height_ * 0), cv::Point(tile_width_ * 2 - 1, tile_height_ * 2), cv::Scalar(255, 255, 255), 2);

  // Draw recognition space borders.
  cv::line(monitor_, cv::Point(tile_width_ * 1, tile_height_ * 0.5 - 1), cv::Point(tile_width_ * 2, tile_height_ * 0.5 - 1), cv::Scalar(255, 255, 255), 2);
  cv::line(monitor_, cv::Point(tile_width_ * 1.5 - 1, tile_height_ * 0), cv::Point(tile_width_ * 1.5 - 1, tile_height_ * 1), cv::Scalar(255, 255, 255), 2);

  // Draw timer/status border.
  cv::line(monitor_, cv::Point(tile_width_ * 0.3 - 1, tile_height_ * 2), cv::Point(tile_width_ * 0.3 - 1, tile_height_ * 2 + log_height_), cv::Scalar(255, 255, 255), 2);
}

void Monitor::monitorPanelInit()
{
  // Create the monitor panel layout.
  tile_size_.width = tile_width_;
  tile_size_.height = tile_height_;
  thumb_size_.width = tile_width_ / 2;
  thumb_size_.height = tile_height_ / 2;
  log_height_ = tile_height_ * 0.25;
  monitor_ = cv::Mat(tile_height_ * 2 + log_height_, tile_width_ * 3, CV_8UC3, cv::Scalar(255, 0, 0));

  // Display tile information.
  cv::putText(monitor_, "EE CamE C", cv::Point(tile_width_ * 0 + padding_left_, tile_height_ * 0 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  cv::putText(monitor_, "EE CamE D", cv::Point(tile_width_ * 0 + padding_left_, tile_height_ * 1 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

  cv::putText(monitor_, "RS CamC C", cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 0 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  cv::putText(monitor_, "RS CamB C", cv::Point(tile_width_ * 1.5 + padding_left_, tile_height_ * 0 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  cv::putText(monitor_, "RS CamL C", cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 0.5 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  cv::putText(monitor_, "RS CamR C", cv::Point(tile_width_ * 1.5 + padding_left_, tile_height_ * 0.5 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

  cv::putText(monitor_, "State Machine", cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

  cv::putText(monitor_, "Democratic Vote", cv::Point(tile_width_ * 2 + padding_left_, tile_height_ * 0 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

  cv::putText(monitor_, "Time", cv::Point(tile_width_ * 0 + padding_left_, tile_height_ * 2 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  cv::putText(monitor_, "Status", cv::Point(tile_width_ * 0.3 + padding_left_, tile_height_ * 2 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

  // Overlay inner borders.
  monitorPanelCell();

  // Display the monitor panel.
  cv::imshow("Monitor", monitor_);
  cv::waitKey(1);
}

void Monitor::monitorPanelDisp()
{
  cv::Rect rect;
  long round_time;
  char txt[255];

  // Update current time.
  long now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
  if (status_run_) {
    if (!round_started_) {
      round_started_ = true;
      round_start_ = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
    }
  }
  else {
    round_started_ = false;
  }

  // Update state machine infomation.
  if (status_state_ishere_ && (state_now_ != state_prev_1_)) {
    cv::rectangle(monitor_, cv::Point(tile_width_ * 1, tile_height_ * 1), cv::Point(tile_width_ * 2, tile_height_ * 2), cv::Scalar(0, 0, 0), CV_FILLED);

    // Display current state.
    cv::putText(monitor_, state_now_, cv::Point(tile_width_ * 1.25 + padding_left_, tile_height_ * 1.4 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

    // Display previous state.
    sprintf(txt, "P4: ");
    cv::putText(monitor_, txt + state_prev_4_, cv::Point(tile_width_ * 1.25 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 0), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    sprintf(txt, "P3: ");
    cv::putText(monitor_, txt + state_prev_3_, cv::Point(tile_width_ * 1.25 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 1), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    sprintf(txt, "P2: ");
    cv::putText(monitor_, txt + state_prev_2_, cv::Point(tile_width_ * 1.25 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 2), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    sprintf(txt, "P1: ");
    cv::putText(monitor_, txt + state_prev_1_, cv::Point(tile_width_ * 1.25 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 3), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    if (round_started_) {
      state_time_4_ = state_time_3_;
      sprintf(txt, "%.1lfs", (float)state_time_4_ / 1000);
      cv::putText(monitor_, txt, cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 0), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
      state_time_3_ = state_time_2_;
      sprintf(txt, "%.1lfs", (float)state_time_3_ / 1000);
      cv::putText(monitor_, txt, cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 1), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
      state_time_2_ = state_time_1_;
      sprintf(txt, "%.1lfs", (float)state_time_2_ / 1000);
      cv::putText(monitor_, txt, cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 2), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
      state_time_1_ = state_time_0_;
      sprintf(txt, "%.1lfs", (float)state_time_1_ / 1000);
      cv::putText(monitor_, txt, cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1 + padding_top_ + (padding_top_ / 2) * 3), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    }

    // Display up to 4 potential next states.
    std::vector<std::string> state_next_4 = {"N1-1:", "N1-2:", "N1-3:"};
    for (std::size_t i = 0; i < 3 && i < state_next_.size(); i++) {
      state_next_4[i] = state_next_[i];
      sprintf(txt, "N1-%d: ", int(i + 1));
      cv::putText(monitor_, txt + state_next_4[i], cv::Point(tile_width_ * 1.25 + padding_left_ * 0, tile_height_ * 1.6 + padding_top_ + (padding_top_ / 2) * i), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    }

    // Register initial state time.
    state_start_ = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

    // Update previous state.
    state_prev_4_ = state_prev_3_;
    state_prev_3_ = state_prev_2_;
    state_prev_2_ = state_prev_1_;
    state_prev_1_ = state_now_;
  }

  // Display timers.
  if (round_started_) {
    round_time = now - round_start_;
    sprintf(txt, "%.1lfs", (float)round_time / 1000);
    cv::rectangle(monitor_, cv::Point(tile_width_ * 0, tile_height_ * 2), cv::Point(tile_width_ * 0.3, tile_height_ * 2 + log_height_), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(monitor_, txt, cv::Point(tile_width_ * 0 + padding_left_, tile_height_ * 2 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);

    state_time_0_ = now - state_start_;
    sprintf(txt, "%.1lfs", (float)state_time_0_ / 1000);
    cv::rectangle(monitor_, cv::Point(tile_width_ * 1, tile_height_ * 1.4), cv::Point(tile_width_ * 1.25, tile_height_ * 1.6), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(monitor_, txt, cv::Point(tile_width_ * 1 + padding_left_, tile_height_ * 1.4 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  }

  // Update target item infomation.
  if (status_task_ishere_) {
    cv::rectangle(monitor_, cv::Point(tile_width_ * 2, tile_height_ * 0), cv::Point(tile_width_ * 3, tile_height_ * 2), cv::Scalar(0, 0, 0), CV_FILLED);
    for (std::size_t i = 0; i < status_task_.size(); i++) {
      cv::putText(monitor_, status_task_[i], cv::Point(tile_width_ * 2 + padding_left_, tile_height_ * 0 + padding_top_ + (padding_top_ / 2) * i), font_face_, 1.5 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
    }
  }

  // Display status messages from task manager.
  if (status_msg_ishere_) {
    cv::rectangle(monitor_, cv::Point(tile_width_ * 0.3, tile_height_ * 2), cv::Point(tile_width_ * 3, tile_height_ * 2 + log_height_), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(monitor_, status_msg_, cv::Point(tile_width_ * 0.3 + padding_left_, tile_height_ * 2 + padding_top_), font_face_, 2 * font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
  }

  // Display YOLO color image.
  if (yolo_color_img_ishere_) {
    rect = cv::Rect(tile_width_ * 0, tile_height_ * 0, yolo_color_img_.cols, yolo_color_img_.rows);
    yolo_color_img_.copyTo(monitor_(rect));
  }

  // Display YOLO depth image.
  if (yolo_depth_img_ishere_) {
    rect = cv::Rect(tile_width_ * 0, tile_height_ * 1, yolo_depth_img_.cols, yolo_depth_img_.rows);
    yolo_depth_img_.copyTo(monitor_(rect));
  }

  // Display views from recognition space dynamically.
  cv::Mat rs_thumb;

  if (node_state_ == SHOW_REC_SPACE_CAM) {
    if (rs_camc_bb_img_ishere_ || rs_caml_bb_img_ishere_ || rs_camr_bb_img_ishere_ || rs_camb_bb_img_ishere_) {
      node_state_ = SHOW_REC_SPACE_BB;
      disp_start_ = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

      // Reset tile to blue.
      cv::rectangle(monitor_, cv::Point(tile_width_ * 2, tile_height_ * 1), cv::Point(tile_width_ * 3, tile_height_ * 2), cv::Scalar(255, 0, 0), CV_FILLED);
    }
    else {
      // Display center camera color image.
      if (rs_camc_color_img_ishere_) {
        cv::resize(rs_camc_color_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::flip(rs_thumb, rs_thumb, 0);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "C", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1, tile_height_ * 0, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display base camera color image.
      if (rs_camb_color_img_ishere_) {
        cv::resize(rs_camb_color_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "B", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1.5, tile_height_ * 0, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display left camera color image.
      if (rs_caml_color_img_ishere_) {
        cv::resize(rs_caml_color_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "L", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1, tile_height_ * 0.5, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display right camera color image.
      if (rs_camr_color_img_ishere_) {
        cv::resize(rs_camr_color_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "R", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1.5, tile_height_ * 0.5, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }
    }
  }

  if (node_state_ = SHOW_REC_SPACE_BB) {
    if (now - disp_start_ < disp_time_) {
      // Display center camera bounding box.
      if (rs_camc_bb_img_ishere_) {
        cv::resize(rs_camc_bb_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::flip(rs_thumb, rs_thumb, 0);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "C", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1, tile_height_ * 0, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display base camera bounding box.
      if (rs_camb_bb_img_ishere_) {
        cv::resize(rs_camb_bb_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        //cv::flip(rs_thumb, rs_thumb, 0);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "B", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1.5, tile_height_ * 0, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display left camera bounding box.
      if (rs_caml_bb_img_ishere_) {
        cv::resize(rs_caml_bb_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "L", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1, tile_height_ * 0.5, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      // Display right camera bounding box.
      if (rs_camr_bb_img_ishere_) {
        cv::resize(rs_camr_bb_img_, rs_thumb, thumb_size_, 0, 0, CV_INTER_AREA);
        cv::rectangle(rs_thumb, cv::Point(0, 0), cv::Point(30, 30), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::putText(rs_thumb, "R", cv::Point(8, 20), font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thick_, font_ltype_);
        rect = cv::Rect(tile_width_ * 1.5, tile_height_ * 0.5, rs_thumb.cols, rs_thumb.rows);
        rs_thumb.copyTo(monitor_(rect));
      }

      rs_camc_bb_img_ishere_ = false;
      rs_caml_bb_img_ishere_ = false;
      rs_camr_bb_img_ishere_ = false;
      rs_camb_bb_img_ishere_ = false;
    }
    else {
      node_state_ = SHOW_REC_SPACE_CAM;
    }
  }

  // Overlay inner borders.
  monitorPanelCell();

  // Display the monitor panel.
  cv::imshow("Monitor", monitor_);
  cv::waitKey(1);
}

void Monitor::yoloColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  yolo_color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  yolo_color_img_ishere_ = true;
  monitorPanelDisp();
}

void Monitor::yoloDepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  yolo_depth_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  yolo_depth_img_ishere_ = true;
}

void Monitor::rsCamCColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camc_color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camc_color_img_ishere_ = true;
}

void Monitor::rsCamLColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_caml_color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_caml_color_img_ishere_ = true;
}

void Monitor::rsCamRColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camr_color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camr_color_img_ishere_ = true;
}

void Monitor::rsCamBColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camb_color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camb_color_img_ishere_ = true;
}

void Monitor::rsCamCBbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camc_bb_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camc_bb_img_ishere_ = true;
}

void Monitor::rsCamLBbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_caml_bb_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_caml_bb_img_ishere_ = true;
}

void Monitor::rsCamRBbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camr_bb_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camr_bb_img_ishere_ = true;
}

void Monitor::rsCamBBbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  rs_camb_bb_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  rs_camb_bb_img_ishere_ = true;
}

void Monitor::statusMessageCallback(const std_msgs::StringConstPtr &msg)
{
  status_msg_ = msg->data;
  status_msg_ishere_ = true;
}

void Monitor::statusStateCallback(const tnp_msgs::StateInfo &msg)
{
  state_now_ = msg.current_state;
  state_next_ = msg.next_states;
  status_state_ishere_ = true;
}

void Monitor::statusRunCallback(const std_msgs::BoolConstPtr &msg)
{
  status_run_ = msg->data;
  status_run_ishere_ = true;
}

void Monitor::statusTaskCallback(const tnp_msgs::TaskList &msg)
{
  status_task_ = msg.info;
  status_task_ishere_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_monitor");
  ros::NodeHandle nh;

  Monitor mp = Monitor(nh);

  cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
  cv::startWindowThread();
  mp.monitorPanelInit();

  ros::spin();

  cv::destroyWindow("Monitor");

  return 0;
}
