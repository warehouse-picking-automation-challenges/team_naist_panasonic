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

#ifndef SRC_TNP_RECORD_RGBD_FRAME_H_
#define SRC_TNP_RECORD_RGBD_FRAME_H_

#include "RGBDFrame.h"
#include "tnp_feat_vision/cloud_matching.h"
#include "helpers.h"
#include "UtilCvPclRs.h"

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#include <ctime>
#include <pcl/features/integral_image_normal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

class RecordRgbdFrameNode
{
public:
  RecordRgbdFrameNode(ros::NodeHandle &nh);
  virtual ~RecordRgbdFrameNode();
  bool recordRgbdFramesCallback(tnp_feat_vision::cloud_matching::Request &req,
                                tnp_feat_vision::cloud_matching::Response &res);

private:
  ros::NodeHandle nh_;

  ros::ServiceServer srv_rgbd_frame_record_;
  time_t t_start_;
  int call_counter_;

  cv::Mat rgbPreparation(const sensor_msgs::Image rgb);
  cv::Mat depthPreparation(const sensor_msgs::Image depth);
  std::vector<cv::Mat> rgbInfoPreparation(const sensor_msgs::CameraInfo camInfo);
  std::vector<cv::Mat> depthInfoPreparation(const sensor_msgs::CameraInfo camInfo);
  int matchCamToInt(std::string camID);

};

#endif /* SRC_TNP_RECORD_RGBD_FRAME_H_ */
