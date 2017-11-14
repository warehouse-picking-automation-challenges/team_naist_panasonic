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

#include "tnp_simple_capture.h"

/*
 * Converts ros images to openCV images, copyies the images
 * type as sensor_msgs::image_encodings::BGR8|TYPE_16UC1.
 */
cv_bridge::CvImagePtr convert2OpenCV(const sensor_msgs::Image &img, const std::string type)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, type);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Image converter, cv_bridge exception: %s", e.what());
  }
  return cv_ptr;
}

static const std::string CV_WIN_NAME = "Image";
cv::Scalar colors[6] = {cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 0),
                        cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255),
                        cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 255)};

//// Public functions
SimpleCap::SimpleCap(ros::NodeHandle &nh)
  : nh_(nh), it_(nh)
  , save_count_color_(0)
  , save_count_depth_(0)
  , save_count_depth_reg_(0)
  , save_count_ir_(0)
{
  setupROSConfig();
}

SimpleCap::~SimpleCap()
{
  cv::destroyWindow(CV_WIN_NAME);
}

bool SimpleCap::setupROSConfig()
{
  sub_camera_info_ = nh_.subscribe("/camera/depth_registered/sw_registered/camera_info", 1, &SimpleCap::cameraInfoCallback, this);
  sub_color_ = it_.subscribe("/camera/rgb/image_raw", 1, &SimpleCap::colorCallback, this);
  sub_depth_ = it_.subscribe("/camera/depth/image_raw", 1, &SimpleCap::depthCallback, this);
  sub_depth_reg_ = it_.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 1, &SimpleCap::depthRegCallback, this);
  sub_ir_ = it_.subscribe("/camera/ir/image_raw", 1, &SimpleCap::irCallback, this);

  capture_service_ = nh_.advertiseService("simple_capture/capture", &SimpleCap::captureCallback, this);

  return true;
}

std::string SimpleCap::getImgSaveDir()
{
  return img_save_dir_;
}

void SimpleCap::setImgSaveDir(std::string save_dir)
{
  img_save_dir_ = save_dir;
}

//// Private functions
void SimpleCap::allThere()
{
  if (color_img_ishere_ and depth_img_ishere_ and depth_reg_img_ishere_ and ir_img_ishere_ and cam_info_ishere_)
  {
    color_img_ishere_ = false;
    depth_img_ishere_ = false;
    depth_reg_img_ishere_ = false;
    ir_img_ishere_ = false;
    cam_info_ishere_ = false;

    //// Get min/max value in depth image (ignore zero value).
    cv::Mat depth_8U, depth_nonzero_mask;
    depth_img_.convertTo(depth_8U, CV_8UC1);
    cv::threshold(depth_8U, depth_nonzero_mask, 1, 255, cv::THRESH_BINARY);
    double min, max;
    cv::minMaxLoc(depth_img_, &min, &max, NULL, NULL, depth_nonzero_mask);
    depth_max_vals_.push_back(max);
    if(depth_max_vals_.size() > 100)
    {
      depth_max_vals_.erase(depth_max_vals_.begin());
    }
    max = std::accumulate(depth_max_vals_.begin(), depth_max_vals_.end(), 0.0) / depth_max_vals_.size();
    //// Draw min/max in depth image.
    cv::Mat depth_show;
    depth_img_.convertTo(depth_show, CV_8UC1, 255 / max);
    depth_show.copyTo(depth_8u_save_);

    //// Get min/max value in depth_registered image (from realsense_camera) (ignore zero value).
    cv::Mat depth_reg_8U, depth_reg_nonzero_mask;
    depth_reg_img_.convertTo(depth_reg_8U, CV_8UC1);
    cv::threshold(depth_reg_8U, depth_reg_nonzero_mask, 1, 255, cv::THRESH_BINARY);
    double min_reg, max_reg;
    cv::minMaxLoc(depth_reg_img_, &min_reg, &max_reg, NULL, NULL, depth_reg_nonzero_mask);
    depth_reg_max_vals_.push_back(max_reg);
    if(depth_reg_max_vals_.size() > 100)
    {
      depth_reg_max_vals_.erase(depth_reg_max_vals_.begin());
    }
    max_reg = std::accumulate(depth_reg_max_vals_.begin(), depth_reg_max_vals_.end(), 0.0) / depth_reg_max_vals_.size();

    //// Draw min/max in depth_reg image.
    cv::Mat depth_reg_show;
    depth_reg_img_.convertTo(depth_reg_show, CV_8UC1, 255 / max_reg);
    cv::cvtColor(depth_reg_show, depth_reg_show, CV_GRAY2BGR);
    depth_reg_show.copyTo(depth_reg_8u_save_);
    std::stringstream ss;
    ss << "Min: " << min_reg * DEPTH_SCALE << "m, Max: " << max_reg * DEPTH_SCALE << "m";
    cv::putText(depth_reg_show, ss.str().c_str(), cv::Point(0, depth_reg_show.rows - 5), font_face, font_scale * 1.2,
                cv::Scalar(255, 0, 0), font_thick, font_ltype);

    //// Draw grid
    //// depth_registered
    int grid_size = 50;
    for(int i = 0; i < depth_reg_show.cols / 50.0; ++i)
    {
      cv::line(depth_reg_show, cv::Point(grid_size * (i + 1), 0), cv::Point(grid_size * (i + 1), depth_reg_show.rows - 1), colors[i%6]);
    }
    for(int j = 0; j < depth_reg_show.rows / 50.0; ++j)
    {
      cv::line(depth_reg_show, cv::Point(0, grid_size * (j + 1)), cv::Point(depth_reg_show.cols - 1, grid_size * (j + 1)), colors[j%6]);
    }
    //// color
    cv::Mat color_show;
    color_img_.copyTo(color_show);
    for(int i = 0; i < color_show.cols / 50.0; ++i)
    {
      cv::line(color_show, cv::Point(grid_size * (i + 1), 0), cv::Point(grid_size * (i + 1), color_show.rows - 1), colors[i%6]);
    }
    for(int j = 0; j < color_show.rows / 50.0; ++j)
    {
      cv::line(color_show, cv::Point(0, grid_size * (j + 1)), cv::Point(color_show.cols - 1, grid_size * (j + 1)), colors[j%6]);
    }
    
    cv::Mat img_show(cv::max(color_show.rows, depth_reg_show.rows), color_show.cols + depth_reg_show.cols, CV_8UC3);
    cv::Rect rect(0, 0, color_show.cols, color_show.rows);
    color_show.copyTo(img_show(rect));
    rect = cv::Rect(color_show.cols, 0, depth_reg_show.cols, depth_reg_show.rows);
    depth_reg_show.copyTo(img_show(rect));
    cv::imshow(CV_WIN_NAME, img_show);
    cv::waitKey(1);
  }
}

void SimpleCap::colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  color_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::BGR8)->image;
  color_img_ishere_ = true;
  allThere();
}

void SimpleCap::depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  depth_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  depth_img_ishere_ = true;
  allThere();
}

void SimpleCap::depthRegCallback(const sensor_msgs::ImageConstPtr &msg)
{
  depth_reg_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  depth_reg_img_ishere_ = true;
  allThere();
}

void SimpleCap::irCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ir_img_ = convert2OpenCV(*msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  ir_img_ishere_ = true;
  allThere();
}

void SimpleCap::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  cam_info_ = msg;
  cam_info_ishere_ = true;
  allThere();
}

bool SimpleCap::captureCallback(tnp_simple_capture::capture::Request &req,
                                tnp_simple_capture::capture::Response &res)
{
  ROS_INFO("captureCallback was called");
  std::stringstream ss;
  ss << img_save_dir_ << req.saveFileName << "_" << req.typeToSave;
  if(req.typeToSave == "color")
  {
    ss << "_" << save_count_color_ << ".jpg";
    cv::imwrite(ss.str(), color_img_);
    ++save_count_color_;
  }
  else if(req.typeToSave == "depth")
  {
    ss << "_" << save_count_depth_ << ".jpg";
    cv::imwrite(ss.str(), depth_8u_save_);
    ++save_count_depth_;
  }
  else if(req.typeToSave == "depth_reg")
  {
    ss << "_" << save_count_depth_reg_ << ".jpg";
    cv::imwrite(ss.str(), depth_reg_8u_save_);
    ++save_count_depth_reg_;
  }
  else if(req.typeToSave == "ir")
  {
    ss << "_" << save_count_ir_ << ".jpg";
    cv::imwrite(ss.str(), ir_img_);
    ++save_count_ir_;
  }
  else
  {
    ROS_ERROR("Check service request typeToSave: color, depth, depth_reg, ir");
    res.savedFilePath = "save_error";
    return false;
  }
  res.savedFilePath = ss.str();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_simple_capture");

  ros::NodeHandle nh;
  SimpleCap cap(nh);

  mkdir(cap.getImgSaveDir().c_str(), 0664);
  cv::namedWindow(CV_WIN_NAME, cv::WINDOW_NORMAL);

  ros::spin();

  return 0;
}
