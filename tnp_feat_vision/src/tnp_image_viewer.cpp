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

// Reference: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
boost::array<double, 12> P;
cv::Mat cv_P;

void cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  P = msg.P;
  cv_P = cv::Mat(3, 4, CV_64F, &P[0]);
  ROS_INFO_STREAM(cv_P);
}

void colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::imshow("color/image_raw", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::imshow("depth/image_raw", cv_bridge::toCvShare(msg, "16UC1")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tnp_image_viewer");
  ros::NodeHandle nh;

  ros::Subscriber sub_camera_info;
  sub_camera_info = nh.subscribe("/camera/depth_registered/sw_registered/camera_info", 1, cameraInfoCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_color, sub_depth;

  cv::namedWindow("color/image_raw");
  cv::startWindowThread();
  sub_color = it.subscribe("/tnp_image_converter/color/image_raw", 1, colorCallback);

  cv::namedWindow("depth/image_raw");
  cv::startWindowThread();
  sub_depth = it.subscribe("/tnp_image_converter/depth/image_raw", 1, depthCallback);

  ros::spin();

  cv::destroyWindow("color/image_raw");
  cv::destroyWindow("depth/image_raw");

  return 0;
}
