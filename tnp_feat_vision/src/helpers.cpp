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

#include "helpers.h"

using namespace std;

/**
 * Returns the current time in milliseconds
 */
long Helpers::timeNow()
{
  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * recursive mkdir
 * @param dir path
 * @param a_rights access rights
 */
void Helpers::_mkdir(const char *dir, int a_rights)
{
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", dir);
  len = strlen(tmp);
  if (tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for (p = tmp + 1; *p; p++)
    if (*p == '/')
    {
      *p = 0;
      mkdir(tmp, a_rights);
      *p = '/';
    }
  mkdir(tmp, a_rights);
}

/**
 *  Returns files in the directory *
 * @param dir directory string
 * @param file and folder names in given dir path
 * @param fileType fileType filter [default "", returns all files]
 * @return success of folder access
 */
bool Helpers::getDir(string dir, vector<string> &files, string fileType /* = ""*/)
{
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(dir.c_str())) == NULL)
  {
    return false;
  }

  while ((dirp = readdir(dp)) != NULL)
  {
    if (hasEnding(dirp->d_name, fileType) || fileType.compare("") == 0)
      files.push_back(string(dirp->d_name));
  }

  closedir(dp);
  return true;
}

bool Helpers::file_exists(const std::string& name)
{
  if (FILE *file = fopen(name.c_str(), "r"))
  {
    fclose(file);
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * File name has a particular ending and is not a hidden file.
 * @param fullString fileName
 * @param ending fileType
 * @return hasEnding
 */
bool Helpers::hasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending)
        && 0 != fullString.compare(0, 1, "."));
  }
  else
  {
    return false;
  }
}

/**
 * Sorts the file structure descending, smallest first
 * @param a
 * @param b
 * @return a > b
 */
bool Helpers::asc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b)
{
  return a.first < b.first;
}

/**
 * Sorts the file structure descending, largest first
 * @param a
 * @param b
 * @return a < b
 */
bool Helpers::desc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b)
{
  return a.first > b.first;
}

void Helpers::printFirstN(int n, std::vector<std::pair<double, std::pair<int, int> > > data_item,
                          vector<string> itemNames)
{
  ROS_DEBUG("printFirstN        ");
  for (int id = 0; id < n && id < data_item.size(); id++)
  {
    int j = data_item[id].second.first; // item
    int k = data_item[id].second.second; // picture
    ROS_INFO("Similarity to %s - Training data %i | Distance %f", itemNames[j].c_str(), k, data_item[id].first);
  }
}

/*
 * Converts ros images to openCV images, copyies the images
 * type as sensor_msgs::image_encodings::TYPE_16UC1  // sensor_msgs::image_encodings::BGR8
 */
cv_bridge::CvImagePtr Helpers::convert2OpenCV(const sensor_msgs::Image &img, const std::string type)
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

/**
 *   transforming ROS data package to input data which fits the cloud matching algorithm
 */
vector<RgbdFrame> Helpers::rosToRGBDFrame(vector<std_msgs::String> &camIds, vector<sensor_msgs::Image> &rgbImgs,
                                           vector<sensor_msgs::Image> &depthImgs,
                                           vector<sensor_msgs::CameraInfo> &rgbInfos,
                                           vector<sensor_msgs::CameraInfo> &depthInfos)
{
  ROS_DEBUG("Transforming images to RgbdFrames...");
  vector < RgbdFrame > scenery;

  for (int i = 0; i < camIds.size(); i++)
  {
    RgbdFrame aFrame;

    cv::Mat rgb = rgbPreparation(rgbImgs[i]);
    vector < cv::Mat > colorData = rgbInfoPreparation(rgbInfos[i]);
    cv::Mat color_K = colorData[0];
    cv::Mat color_D = colorData[1];

    cv::Mat depth, depth_K, depth_D, depth_to_color_extrinsics;
    if (!depthImgs.empty() && !depthInfos.empty())
    {
      depth = depthPreparation(depthImgs[i]);
      vector < cv::Mat > depthData = depthInfoPreparation(depthInfos[i]);
      depth_K = depthData[0];
      depth_to_color_extrinsics = depthData[1];
      depth_D = depthData[2];
    }

    aFrame = RgbdFrame::extractDataFromRos(color_K, color_D, rgb, depth_K, depth_D, depth, depth_to_color_extrinsics);
    aFrame.camSerial = camIds[i].data;

    scenery.push_back(aFrame); // can be also an empty frame, when a camera failed.
  }

  return scenery;
}

/**
 * All calculations on rgb data before cloud matching
 */
cv::Mat Helpers::rgbPreparation(const sensor_msgs::Image rosRgbImg)
{
  return Helpers::convert2OpenCV(rosRgbImg, sensor_msgs::image_encodings::BGR8)->image;
}

/**
 * All calculations on depth data before cloud matching
 */
cv::Mat Helpers::depthPreparation(const sensor_msgs::Image rosDepthImg)
{
  return Helpers::convert2OpenCV(rosDepthImg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
}

/**
 * All calculations on rgbInfo before cloud matching
 */
vector<cv::Mat> Helpers::rgbInfoPreparation(const sensor_msgs::CameraInfo camInfo)
{
  vector < cv::Mat > rgbMats;
  rgbMats.push_back(cv::Mat(3, 3, CV_64F, (void *)&camInfo.K[0], cv::Mat::AUTO_STEP).clone());
  rgbMats.push_back(cv::Mat(1, 5, CV_64F, (void *)&camInfo.D[0], cv::Mat::AUTO_STEP).clone()); // depth_D
  return rgbMats;
}

/**
 * All calculations on depthInfo before cloud matching
 */
vector<cv::Mat> Helpers::depthInfoPreparation(const sensor_msgs::CameraInfo camInfo)
{
  vector < cv::Mat > depthMats;

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
