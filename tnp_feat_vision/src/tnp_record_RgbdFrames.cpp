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

#include "tnp_record_RgbdFrames.h"

using namespace std;

RecordRgbdFrameNode::RecordRgbdFrameNode(ros::NodeHandle &nh) :
    nh_(nh)
{
  srv_rgbd_frame_record_ = nh_.advertiseService("/tnp_record_rgbd_frames/save",
                                                &RecordRgbdFrameNode::recordRgbdFramesCallback, this);
  t_start_ = std::time(0);
  call_counter_ = 0;

}

RecordRgbdFrameNode::~RecordRgbdFrameNode()
{
  // TODO Auto-generated destructor stub
}

bool RecordRgbdFrameNode::recordRgbdFramesCallback(tnp_feat_vision::cloud_matching::Request &req,
                                                   tnp_feat_vision::cloud_matching::Response &res)
{
  string save_path, item_name;
  if (req.target_id.data.empty())
  {
    save_path = "/root/share/rgbdFrame_backup_" + to_string(t_start_);
    item_name = "capture";
  }
  else
  {
    save_path = "/root/share/item_images_" + to_string(t_start_) + "/" + req.target_id.data;
    item_name = req.target_id.data;
    ROS_INFO("Saving item data %s", item_name.c_str());
  }

  vector<RgbdFrame> scenery = Helpers::rosToRGBDFrame(req.cam_id, req.rgb_data, req.depth_data, req.rgb_info_data,
                                                      req.depth_info_data);
  ROS_DEBUG("Received %i RGBDFrames", scenery.size());

  // check whether directory exists already, otherwise create.
  struct stat st = {0};
  if (stat(save_path.c_str(), &st) == -1)
  {
    Helpers::_mkdir(save_path.c_str(), 0755);
    ROS_DEBUG("Created new save folder %s", save_path.c_str());
  }

  for (int i = 0; i < scenery.size(); i++)
  {
    int camNumber = matchCamToInt(scenery[i].camSerial);
    string filename = item_name + to_string(call_counter_) + "_cam" + to_string(camNumber);
    scenery[i].save(save_path, filename);
    ROS_DEBUG("Writing RGBDFrame to disk: %s", filename.c_str());
  }
  call_counter_++;

  return true;
}

/**
 *  Translates the provided cam location into an ID
 */
int RecordRgbdFrameNode::matchCamToInt(string camID)
{
  if (camID.compare("L") == 0)
  {
    return 0;
  }
  else if (camID.compare("C") == 0)
  {
    return 1;
  }
  else if (camID.compare("R") == 0)
  {
    return 2;
  }
  else if (camID.compare("B") == 0)
  {
    return 3;
  }
  else if (camID.compare("E") == 0)
  {
    return 4;
  }
  else if (camID.compare("T") == 0)
  {
    return 5;
  }
  else
  {
    return -1;
  }
}

/**
 * All calculations on rgb data before cloud matching
 */
cv::Mat RecordRgbdFrameNode::rgbPreparation(const sensor_msgs::Image rosRgbImg)
{
  return Helpers::convert2OpenCV(rosRgbImg, sensor_msgs::image_encodings::BGR8)->image;
}

/**
 * All calculations on depth data before cloud matching
 */
cv::Mat RecordRgbdFrameNode::depthPreparation(const sensor_msgs::Image rosDepthImg)
{
  return Helpers::convert2OpenCV(rosDepthImg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
}

/**
 * All calculations on rgbInfo before cloud matching
 */
vector<cv::Mat> RecordRgbdFrameNode::rgbInfoPreparation(const sensor_msgs::CameraInfo camInfo)
{
  vector<cv::Mat> rgbMats;
  rgbMats.push_back(cv::Mat(3, 3, CV_64F, (void *)&camInfo.K[0], cv::Mat::AUTO_STEP).clone());
  rgbMats.push_back(cv::Mat(1, 5, CV_64F, (void *)&camInfo.D[0], cv::Mat::AUTO_STEP).clone()); // depth_D
  return rgbMats;
}

/**
 * All calculations on depthInfo before cloud matching
 */
vector<cv::Mat> RecordRgbdFrameNode::depthInfoPreparation(const sensor_msgs::CameraInfo camInfo)
{
  vector<cv::Mat> depthMats;

  cv::Mat depth_to_color_extrinsics;

  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

  depth_to_color_extrinsics = cv::Mat::eye(4, 4, CV_64F);
  depth_to_color_extrinsics.at<double>(0, 3) = camInfo.P[3];
  depth_to_color_extrinsics.at<double>(1, 3) = camInfo.P[7];
  depth_to_color_extrinsics.at<double>(2, 3) = camInfo.P[11];
  R.copyTo(depth_to_color_extrinsics(cv::Rect(0, 0, 3, 3)));

  depthMats.push_back(cv::Mat(3, 3, CV_64F, (void *)&camInfo.K[0], cv::Mat::AUTO_STEP).clone()); // depth_K
  depthMats.push_back(depth_to_color_extrinsics);
  depthMats.push_back(cv::Mat(1, 5, CV_64F, (void *)&camInfo.D[0], cv::Mat::AUTO_STEP).clone()); // depth_D

  return depthMats;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tnp_cloud_matching");
  ros::NodeHandle nh;

  RecordRgbdFrameNode recordRgbdFrameNode(nh);

  ros::spin();
}
