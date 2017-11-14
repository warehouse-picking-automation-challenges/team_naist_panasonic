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

#ifndef SRC_TNP_CLOUD_MATCHING_H_
#define SRC_TNP_RECORD_RGBD_FRAME_H_

#include "RGBDFrame.h"
#include "tnp_feat_vision/set_items_info.h"
#include "tnp_feat_vision/set_items_pics.h"
#include "tnp_feat_vision/cloud_matching.h"
#include "tnp_feat_vision/set_rs_background.h"
#include "tnp_feat_vision/get_bounding_box.h"
#include "tnp_feat_vision/remove_background.h"
#include "BackgroundSubtraction.h"
#include "helpers.h"

#include "UtilCvPclRs.h"
#include "BBoxExtractor.h"

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <pcl/features/integral_image_normal.h>
#include <opencv2/opencv.hpp>

class CloudMatchingNode
{
public:
  CloudMatchingNode(ros::NodeHandle &nh);
  virtual ~CloudMatchingNode();
  bool cloudMatchingCallback(tnp_feat_vision::cloud_matching::Request &req,
                             tnp_feat_vision::cloud_matching::Response &res);

  bool setRSBackgroundCallback(tnp_feat_vision::set_rs_background::Request &req,
                               tnp_feat_vision::set_rs_background::Response &res);
  bool getBoundingBoxCallback(tnp_feat_vision::get_bounding_box::Request &req,
                              tnp_feat_vision::get_bounding_box::Response &res);

  bool backgroundSubtraction(tnp_feat_vision::remove_background::Request &req,
                             tnp_feat_vision::remove_background::Response &res);

  void depthPreparation(cv_bridge::CvImagePtr depth);

  // Service created to receive items information data from task manager node
  bool setItemsInfoCallback(tnp_feat_vision::set_items_info::Request &req,
                            tnp_feat_vision::set_items_info::Response &res);
  bool setItemsPicsCallback(tnp_feat_vision::set_items_pics::Request &req,
                            tnp_feat_vision::set_items_pics::Response &res);

private:
  ros::NodeHandle nh_;
  BBoxExtractor bbx_extractor_;

  enum NodeState
  {
    INIT, READY
  };
  NodeState nodeState_ = INIT;
  std::string resourceFolder_;

  ros::ServiceServer srv_cloud_matching_;
  ros::ServiceServer srv_set_items_info_;
  ros::ServiceServer srv_set_items_pics_;
  ros::ServiceServer srv_set_rs_background_;
  ros::ServiceServer srv_get_bounding_box_;
  ros::ServiceServer srv_remove_background_;

  ros::Publisher pub_bounding_box_L_;
  ros::Publisher pub_bounding_box_C_;
  ros::Publisher pub_bounding_box_R_;
  ros::Publisher pub_bounding_box_B_;

  cv::Ptr<cv::AKAZE> akaze_;
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::Mat> listResultImg0_;
  std::vector<std::string> itemIDs_;
  std::vector<std::string> loadedItemIDs_;
  std::vector<cv::Mat> listFeaturesDescFull_;
  std::vector<std::vector<cv::Point3f> > listFeaturesPos3d_;
  std::vector<std::vector<cv::Point3f> > listFeaturesNormal3d_;
  std::vector<std::vector<RgbdFrame> > background_;
  void init();

  RgbdFrame generatePointCloudOrganized2(cv::Mat colorK, cv::Mat colorD, cv::Mat colorImg, cv::Mat depthK,
                                         cv::Mat depthD, cv::Mat depthImg, cv::Mat Extrinsic);
  RgbdFrame generatePointCloudOrganized3(RgbdFrame& frame);

  cv::Mat findPoseWithDepthCue(const std::vector<cv::Point3f> &obj, const std::vector<cv::Point3f> &objNormal,
                               const std::vector<cv::Point2f> &scene, RgbdFrame frame, cv::Mat &mask);

  cv::Mat findPoseWith2PointsAndNormal(cv::Point3f p1, cv::Point3f n1, cv::Point3f p2, cv::Point3f origP1,
                                       cv::Point3f origN1, cv::Point3f origP2);
  bool loadFeatures(const char *filename, std::vector<cv::Point3f>& featuresPos3d,
                    std::vector<cv::Point3f>& featuresNormal3d, cv::Mat& featuresDesc);

  // Variables to store the items information data provided by task manager node
  std::vector<std_msgs::String> items_ids_information;
  std::vector<std_msgs::Float64[3]> items_dimensions_information;
};

constexpr unsigned int str2int(const char* str, int h = 0);

#endif /* SRC_TNP_CLOUD_MATCHING_H_ */
