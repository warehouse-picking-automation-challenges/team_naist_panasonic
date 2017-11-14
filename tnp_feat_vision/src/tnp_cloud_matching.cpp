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

#include "tnp_cloud_matching.h"

using namespace std;

CloudMatchingNode::CloudMatchingNode(ros::NodeHandle &nh) :
    nh_(nh), bbx_extractor_(false)
{
  resourceFolder_ = "/root/share/tnp_feature_vision/cloud_matching/";

  srv_cloud_matching_ = nh_.advertiseService("/tnp_cloud_matching/identify_item",
                                             &CloudMatchingNode::cloudMatchingCallback, this);
  srv_set_items_info_ = nh_.advertiseService("/tnp_cloud_matching/set_items_info",
                                             &CloudMatchingNode::setItemsInfoCallback, this);
  srv_set_items_pics_ = nh_.advertiseService("/tnp_cloud_matching/set_items_pics",
                                             &CloudMatchingNode::setItemsPicsCallback, this);
  srv_set_rs_background_ = nh_.advertiseService("/tnp_cloud_matching/set_rs_background",
                                                &CloudMatchingNode::setRSBackgroundCallback, this);
  srv_get_bounding_box_ = nh_.advertiseService("/tnp_cloud_matching/get_bounding_box",
                                               &CloudMatchingNode::getBoundingBoxCallback, this);
  srv_remove_background_ = nh_.advertiseService("/tnp_cloud_matching/remove_background",
                                                &CloudMatchingNode::backgroundSubtraction, this);

  pub_bounding_box_L_ = nh_.advertise<sensor_msgs::Image>("/tnp_cloud_matching/bounding_box_L", 1);
  pub_bounding_box_C_ = nh_.advertise<sensor_msgs::Image>("/tnp_cloud_matching/bounding_box_C", 1);
  pub_bounding_box_R_ = nh_.advertise<sensor_msgs::Image>("/tnp_cloud_matching/bounding_box_R", 1);
  pub_bounding_box_B_ = nh_.advertise<sensor_msgs::Image>("/tnp_cloud_matching/bounding_box_B", 1);

  akaze_ = cv::AKAZE::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

  itemIDs_ = vector<string>( {"burts_bees_baby_wipes", "crayons", "tissue_box", "pie_plates", "reynolds_wrap",
                              "robots_dvd", "epsom_salts", "bath_sponge", "table_cloth", "windex"});
  background_ = vector<vector<RgbdFrame>>();

  init();

  ROS_INFO("Cloud matching constructor left successfully.");

}

CloudMatchingNode::~CloudMatchingNode()
{
  // TODO Auto-generated destructor stub
}

/*
 * Initialize CloudmatchingNode, loads all feature descriptors from disk.
 */
void CloudMatchingNode::init()
{
  ROS_INFO("Initialize cloud matching algorithm");
  long t_start = Helpers::timeNow();

  for (int i = 0; i < itemIDs_.size(); i++)
  {
    // read in features from disk
    char filename[255];
    sprintf(filename, string(resourceFolder_ + "cloud/cloud_result/features_%s.yml").c_str(), itemIDs_[i].c_str());

    ROS_INFO("Load item data from %s", filename);
    vector<cv::Point3f> featuresPos3d;
    vector<cv::Point3f> featuresNormal3d;
    cv::Mat featuresDescFull;
    if (loadFeatures(filename, featuresPos3d, featuresNormal3d, featuresDescFull))
    {
      cv::Mat resultImg0;
      sprintf(filename, "%s/capture%d_cam%d_color.png", itemIDs_[i].c_str(), 0, 0);
      resultImg0 = cv::imread(filename);
      listResultImg0_.push_back(resultImg0);
      loadedItemIDs_.push_back(itemIDs_[i]);
      listFeaturesPos3d_.push_back(featuresPos3d);
      listFeaturesNormal3d_.push_back(featuresNormal3d);
      listFeaturesDescFull_.push_back(featuresDescFull);
      ROS_INFO("Loaded feature file contains %lu keypoints, %lu normals, %i x %i desc. features.",
               listFeaturesPos3d_[i].size(), listFeaturesNormal3d_[i].size(), listFeaturesDescFull_[i].cols,
               listFeaturesDescFull_[i].rows);
    }
  }

  if (loadedItemIDs_.size() > 0)
  {
    ROS_INFO("%lu out of %lu item`s features loaded.", loadedItemIDs_.size(), itemIDs_.size());
    nodeState_ = READY;
  }
  else
    ROS_ERROR("ERROR, no item features loaded!");

  // Background substraction init
  std::vector<cv::Mat> cameraK, cameraDist, planeRt, cameraPoses, newCameraPoses, objectRt;
  vector<string> cameraSerial;
  if (!loadCalib(string(resourceFolder_ + "calibMulti.yml").c_str(), cameraSerial, cameraK, cameraDist, newCameraPoses,
                 planeRt))
  {
    ROS_ERROR("setRSBackgroundCallback, loadCalib failed");
    nodeState_ = INIT;
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> boardTop;
  cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1;
  cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2;
  cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1;
  cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2;
  cv::Ptr<cv::aruco::CharucoBoard> additionalBoard;
  float squareLength = (0.138 / 4);
  float markerLength = squareLength * 0.025 / 0.035;
  printf("multiboard square %f marker %f\n", squareLength, markerLength);

  createArucoData(squareLength, markerLength, dictionary, boardTop, boardSmallSide1, boardSmallSide2, boardLongSide1,
                  boardLongSide2, additionalBoard, false);

  bbx_extractor_.init(cameraSerial, cameraK, cameraDist, newCameraPoses, dictionary, additionalBoard->ids[0],
                      additionalBoard->ids[additionalBoard->ids.size() - 1]);
  bbx_extractor_.loadConfig(string(resourceFolder_ + "recoSpace.yml").c_str());

  ROS_DEBUG("CloudMatching init time: %0.2lf sec", ((double )Helpers::timeNow() - t_start) / 1000);
}

/**
 * Reads item descriptor file from disk
 */
bool CloudMatchingNode::loadFeatures(const char *filename, vector<cv::Point3f>& featuresPos3d,
                                     vector<cv::Point3f>& featuresNormal3d, cv::Mat& featuresDesc)
{
  if (access(filename, F_OK) != -1) // file exists
  {
    ROS_INFO("Reading feature file %s...", filename);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    //printf("Read keypoints\n");
    featuresPos3d = vector<cv::Point3f>(fs["keypoints"].size());
    for (int i = 0; i < fs["keypoints"].size(); i++)
      fs["keypoints"][i] >> featuresPos3d[i];
    // printf("Read normals\n");
    featuresNormal3d = vector<cv::Point3f>(fs["normals"].size());
    for (int i = 0; i < fs["normals"].size(); i++)
      fs["normals"][i] >> featuresNormal3d[i];
    //printf("Read features\n");
    fs["features"] >> featuresDesc;
    fs.release();

    return true;
  }
  else
  {
    ROS_WARN("Feature file %s not found.", filename);
    return false;
  }
}

RgbdFrame CloudMatchingNode::generatePointCloudOrganized3(RgbdFrame& frame)
{
  frame.imgDepth = cv::Mat(frame.img.rows, frame.img.cols, CV_32F);
  frame.imgDepth.setTo(cv::Scalar(0.0));
  frame.imgPoints->clear();
  frame.imgPoints->resize(frame.img.rows * frame.img.cols);
  frame.imgPoints->is_dense = false;
  frame.imgPoints->width = frame.img.cols;
  frame.imgPoints->height = frame.img.rows;
  for (int i = 0; i < frame.imgPoints->height; i++)
    for (int j = 0; j < frame.imgPoints->width; j++)
    {
      int id = i * frame.imgPoints->width + j;
      frame.imgPoints->points[id].x = 0.0;
      frame.imgPoints->points[id].y = 0.0;
      frame.imgPoints->points[id].z = 0.0;
    }
  // Depth dimension helpers.
  int dw = 0;
  int dh = 0;
  int dwh = 0;
  dw = frame.imgDepth.cols;
  dh = frame.imgDepth.rows;
  dwh = dw * dh;
  // Set up the cloud to be used.
  frame.rs_cloud_ptr->clear();
  frame.rs_cloud_ptr->resize(dwh);
  frame.rs_cloud_ptr->is_dense = false;
  frame.rs_cloud_ptr->width = dw;
  frame.rs_cloud_ptr->height = dh;
  static const float nan = numeric_limits<float>::quiet_NaN();
  // Iterate the data space.
  // First, iterate across columns:
  for (int dy = 0; dy < dh; dy++)
  {
    // Second, iterate across rows:
    for (int dx = 0; dx < dw; dx++)
    {
      uint i = dy * dw + dx;
      uint16_t depth_value = frame.depth.at<uint16_t>(dy, dx);
      if (depth_value == 0)
      {
        frame.rs_cloud_ptr->points[i].x = (float)(nan);
        frame.rs_cloud_ptr->points[i].y = (float)(nan);
        frame.rs_cloud_ptr->points[i].z = (float)(nan);
        continue;
      }
      cv::Point2f depth_pixel((float)(dx), (float)(dy));
      float depth_in_meters = depth_value * frame.scale;
      cv::Point3f depth_point = frame.depthCameraDeproject(depth_pixel, depth_in_meters);
      cv::Point3f color_point = frame.depthCameraToColorCamera(depth_point);
      cv::Point2f color_pixel = frame.colorCameraProject(color_point);
      const int cx = (int)(round(color_pixel.x));
      const int cy = (int)(round(color_pixel.y));
      // Set up logic to remove bad points.
      bool depth_fail = true;
      bool color_fail = true;
      depth_fail = (depth_point.z > NOISY);
      color_fail = (cx < 0 || cy < 0 || cx > frame.img.cols || cy > frame.img.rows);
      // Cloud input pointers.
      // XYZ input access to cloud.
      float* dp_x;
      float* dp_y;
      float* dp_z;
      dp_x = &(frame.rs_cloud_ptr->points[i].x);
      dp_y = &(frame.rs_cloud_ptr->points[i].y);
      dp_z = &(frame.rs_cloud_ptr->points[i].z);
      // RGB input access to cloud.
      uint8_t* cp_r;
      uint8_t* cp_g;
      uint8_t* cp_b;
      cp_r = &(frame.rs_cloud_ptr->points[i].r);
      cp_g = &(frame.rs_cloud_ptr->points[i].g);
      cp_b = &(frame.rs_cloud_ptr->points[i].b);
      // Cloud input data.
      // Set up depth point data.
      float real_x = 0;
      float real_y = 0;
      float real_z = 0;
      float adjusted_x = 0;
      float adjusted_y = 0;
      float adjusted_z = 0;
      real_x = depth_point.x;
      real_y = depth_point.y;
      real_z = depth_point.z;
      // Adjust point to coordinates.
      adjusted_x = -1 * real_x;
      adjusted_y = -1 * real_y;
      adjusted_z = real_z;
      // Set up color point data.
      uint8_t* offset = frame.img.ptr<uint8_t>(cy) + cx * 3;
      uint8_t raw_r = 0;
      uint8_t raw_g = 0;
      uint8_t raw_b = 0;
      uint8_t adjusted_r = 0;
      uint8_t adjusted_g = 0;
      uint8_t adjusted_b = 0;
      // Cloud point evaluation.
      // If bad point, remove and skip.
      if (depth_fail || color_fail)
      {
        *dp_x = *dp_y = *dp_z = (float)(nan);
        *cp_r = *cp_g = *cp_b = 255;
        continue;
      }
      else // If valid point, add data to cloud.
      {
        raw_r = raw_r = *(offset);
        raw_g = raw_g = *(offset + 1);
        raw_b = raw_b = *(offset + 2);
        // Adjust color arbitrarily.
        adjusted_r = raw_r;
        adjusted_g = raw_g;
        adjusted_b = raw_b;
        // Fill in cloud depth.
        *dp_x = adjusted_x;
        *dp_y = adjusted_y;
        *dp_z = adjusted_z;
        // Fill in cloud color.
        *cp_r = adjusted_r;
        *cp_g = adjusted_g;
        *cp_b = adjusted_b;
      }
    }
  }
  for (int i = 1; i < frame.depth.rows; i++)
  {
    for (int j = 1; j < frame.depth.cols; j++)
    {
      int x[4] = {j - 1, j, j, j - 1}, y[4] = {i - 1, i - 1, i, i};
      uint16_t depth_value[4];
      float depth_in_meters[4];
      cv::Point3f color_point[4];
      cv::Point2f color_pixel[4];
      bool valid = true;
      for (int k = 0; k < 4; k++)
      {
        depth_value[k] = frame.depth.at<uint16_t>(y[k], x[k]);
        depth_in_meters[k] = depth_value[k] * frame.scale;
        cv::Point2f depth_pixel((float)(x[k]), (float)(y[k]));
        cv::Point3f depth_point = frame.depthCameraDeproject(depth_pixel, depth_in_meters[k]);
        color_point[k] = frame.depthCameraToColorCamera(depth_point);
        color_pixel[k] = frame.colorCameraProject(color_point[k]);
        if (depth_value[k] == 0
            || (color_pixel[k].x < 0 || color_pixel[k].y < 0 || color_pixel[k].x > frame.img.cols
                || color_pixel[k].y > frame.img.rows))
          valid = false;
      }
      if (!valid)
        continue;

      for (int k = 0; k < 2; k++)
      {
        cv::Point3f color_pointTri[3] = {color_point[0], color_point[2], (k == 0 ? color_point[1] : color_point[3])};
        cv::Point2f color_pixelTri[3] = {color_pixel[0], color_pixel[2], (k == 0 ? color_pixel[1] : color_pixel[3])};
        cv::Point2f pixMin = {min(color_pixelTri[0].x, min(color_pixelTri[1].x, color_pixelTri[2].x)), min(
            color_pixelTri[0].y, min(color_pixelTri[1].y, color_pixelTri[2].y))};
        cv::Point2f pixMax = {max(color_pixelTri[0].x, max(color_pixelTri[1].x, color_pixelTri[2].x)), max(
            color_pixelTri[0].y, max(color_pixelTri[1].y, color_pixelTri[2].y))};
        for (int x = (int)(floor(pixMin.x)); x < (int)(ceil(pixMax.x)); x++)
          for (int y = (int)(floor(pixMin.y)); y < (int)(ceil(pixMax.y)); y++)
          {
            if (x < 0 || y < 0 || x > frame.img.cols || y > frame.img.rows)
              continue;

            rs::float2 p = {float(x), float(y)};
            float u, v, w;
            Barycentric(rs2cv(p), color_pixelTri[0], color_pixelTri[1], color_pixelTri[2], u, v, w);
            if (u < 0 || u > 1 || v < 0 || v > 1)
              continue;

            cv::Point3f res = u * color_pointTri[0] + v * color_pointTri[1] + w * color_pointTri[2];
            frame.imgDepth.ptr<float>(y)[x] = sqrt(res.dot(res));
            int id = y * frame.imgPoints->width + x;
            frame.imgPoints->points[id].x = res.x;
            frame.imgPoints->points[id].y = res.y;
            frame.imgPoints->points[id].z = res.z;
          }
      }
    }
  }
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(frame.imgPoints);
  ne.compute(*(frame.imgNormals));

  return frame;
}

// Get the raw data and build the current frames cloud.
RgbdFrame CloudMatchingNode::generatePointCloudOrganized2(cv::Mat colorK, cv::Mat colorD, cv::Mat colorImg,
                                                          cv::Mat depthK, cv::Mat depthD, cv::Mat depthImg,
                                                          cv::Mat Extrinsic)
{

  RgbdFrame frame = RgbdFrame::extractDataFromRos(colorK, colorD, colorImg, depthK, depthD, depthImg, Extrinsic);
  RgbdFrame optimFrame = generatePointCloudOrganized3(frame);

  return optimFrame;
}

cv::Mat CloudMatchingNode::findPoseWith2PointsAndNormal(cv::Point3f p1, cv::Point3f n1, cv::Point3f p2,
                                                        cv::Point3f origP1, cv::Point3f origN1, cv::Point3f origP2)
{
  cv::Point3f x = n1;
  cv::Point3f y = p2 - p1;
  y -= y.dot(x) * x;
  y /= sqrt(y.dot(y));
  cv::Point3f z = x.cross(y);

  cv::Point3f origX = origN1;
  cv::Point3f origY = origP2 - origP1;
  origY -= origY.dot(origX) * origX;
  origY /= sqrt(origY.dot(origY));
  cv::Point3f origZ = origX.cross(origY);

  cv::Point3f center = p1 - origP1.dot(origX) * x - origP1.dot(origY) * y - origP1.dot(origZ) * z;
  cv::Mat poseMat = cv::Mat::eye(4, 4, CV_32F);
  poseMat.at<float>(0, 3) = center.x;
  poseMat.at<float>(1, 3) = center.y;
  poseMat.at<float>(2, 3) = center.z;

  cv::Mat R1(3, 3, CV_32F);
  R1.at<float>(0, 0) = x.x;
  R1.at<float>(0, 1) = y.x;
  R1.at<float>(0, 2) = z.x;
  R1.at<float>(1, 0) = x.y;
  R1.at<float>(1, 1) = y.y;
  R1.at<float>(1, 2) = z.y;
  R1.at<float>(2, 0) = x.z;
  R1.at<float>(2, 1) = y.z;
  R1.at<float>(2, 2) = z.z;

  cv::Mat R2(3, 3, CV_32F);
  R2.at<float>(0, 0) = origX.x;
  R2.at<float>(0, 1) = origY.x;
  R2.at<float>(0, 2) = origZ.x;
  R2.at<float>(1, 0) = origX.y;
  R2.at<float>(1, 1) = origY.y;
  R2.at<float>(1, 2) = origZ.y;
  R2.at<float>(2, 0) = origX.z;
  R2.at<float>(2, 1) = origY.z;
  R2.at<float>(2, 2) = origZ.z;

  cv::Mat R = R1 * R2.inv();
  R.copyTo(poseMat(cv::Rect(0, 0, 3, 3)));

  return poseMat;
}

cv::Mat CloudMatchingNode::findPoseWithDepthCue(const vector<cv::Point3f> &obj, const vector<cv::Point3f> &objNormal,
                                                const vector<cv::Point2f> &scene, RgbdFrame frame, cv::Mat &mask)
{
  bool planar_test = false;
  vector<cv::Point3f> newObj;
  vector<cv::Point3f> newObjNormal;
  vector<cv::Point2f> newScene;
  vector<cv::Point3f> newScenePoints;
  vector<cv::Point3f> newSceneNormal;
  mask = cv::Mat(obj.size(), 1, CV_8UC1);
  mask.setTo(cv::Scalar(0));
  vector<int> selectedId;
  for (int i = 0; i < scene.size(); i++)
  {
    int id = int(scene[i].y) * frame.imgPoints->width + int(scene[i].x);
    pcl::PointXYZ p1 = frame.imgPoints->points[id];
    cv::Point3f p(p1.x, p1.y, p1.z);
    if (sqrt(p.dot(p)) > 0)
    {
      newObj.push_back(obj[i]);
      newObjNormal.push_back(objNormal[i]);
      newScene.push_back(scene[i]);
      newScenePoints.push_back(p);
      newSceneNormal.push_back(pcl2cv(frame.imgNormals->points[id]));
      selectedId.push_back(i);
    }
  }
  if (newObj.size() < 4)
    return cv::Mat();
  vector<vector<int>> goodPairs(newObj.size());
  vector<vector<bool>> goodPairsTable(newObj.size());
  for (int i = 0; i < newObj.size(); i++)
  {
    goodPairsTable[i] = vector<bool>(newObj.size());
    for (int j = 0; j < newObj.size(); j++)
      goodPairsTable[i][j] = false;
  }
  for (int i = 0; i < newObj.size(); i++)
  {
    for (int j = i + 1; j < newObj.size(); j++)
    {
      float angleDiff = fabs(acos(newSceneNormal[i].dot(newSceneNormal[j])));
      float dist = sqrt((newObj[i] - newObj[j]).dot(newObj[i] - newObj[j]));
      float dist2 = sqrt((newScenePoints[i] - newScenePoints[j]).dot(newScenePoints[i] - newScenePoints[j]));
      if (dist > 0.95 * dist2 && dist < 1.05 * dist2 && (!planar_test || angleDiff < 30 * CV_PI / 180))
      {
        goodPairs[i].push_back(j);
        goodPairs[j].push_back(i);
        goodPairsTable[i][j] = true;
        goodPairsTable[j][i] = true;
      }
    }
  }
  cv::Mat bestPose;
  vector<int> bestValidPoints;
  for (int iter = 0; iter < 10000; iter++)
  {

    int id1 = rand() % newObj.size();
    if (goodPairs[id1].size() < 3)
      continue;
    int id2 = goodPairs[id1][rand() % goodPairs[id1].size()];

    cv::Mat pose = findPoseWith2PointsAndNormal(newScenePoints[id1], newSceneNormal[id1], newScenePoints[id2],
                                                newObj[id1], newObjNormal[id1], newObj[id2]);

    vector<int> validPoints;
    for (int i = 0; i < newObj.size(); i++)
    {
      cv::Point3f delta = newScenePoints[i] - multMatVec(pose, newObj[i]);
      float dist = delta.dot(delta);
      if (dist < 0.005 * 0.005)
        validPoints.push_back(i);
    }
    if (validPoints.size() > bestValidPoints.size())
    {
      bestPose = pose;
      bestValidPoints = validPoints;
    }
  }
  for (int i = 0; i < bestValidPoints.size(); i++)
  {
    mask.at<unsigned char>(selectedId[bestValidPoints[i]], 0) = 1;
  }
  return bestPose;
}

bool CloudMatchingNode::cloudMatchingCallback(tnp_feat_vision::cloud_matching::Request &req,
                                              tnp_feat_vision::cloud_matching::Response &res)
{
  if (nodeState_ == INIT || req.target_id.data.compare("INIT") == 0)
  {
    ROS_WARN("Node in INIT state - try to initialize now.");
    init();
  }

  if (nodeState_ == READY)
  {

    vector<RgbdFrame> scenery = Helpers::rosToRGBDFrame(req.cam_id, req.rgb_data, req.depth_data, req.rgb_info_data,
                                                        req.depth_info_data);

    int selectedObj = 0;
    for (int i = 0; i < loadedItemIDs_.size(); i++)
    {
      if (loadedItemIDs_[i] == req.target_id.data)
        selectedObj = i;
    }

    if (selectedObj != -1)
    {
      ROS_INFO("Found matching database for %s, using descriptors from internal ID %i", req.target_id.data.c_str(),
               selectedObj);

      for (int frameId = 0; frameId < scenery.size(); frameId++)
      {
        RgbdFrame rawFrame = scenery[frameId];

        RgbdFrame optimizedFrame = generatePointCloudOrganized3(rawFrame);

        cv::Mat frameImg = optimizedFrame.img.clone();
        cv::Mat result = frameImg.clone();

        cv::imshow("depth", 10 * optimizedFrame.imgDepth);

        vector<cv::KeyPoint> keypoints2;
        cv::Mat descriptors2;

        akaze_->detectAndCompute(frameImg, cv::Mat(), keypoints2, descriptors2);

        if (keypoints2.size() == 0)
          return true;

        std_msgs::Float64 conf[5];
        vector<cv::DMatch> matches;
        vector<cv::DMatch> good_matches;
        matcher_->match(listFeaturesDescFull_[selectedObj], descriptors2, matches);

        double max_dist = 0;
        double min_dist = 100;
        // Quick calculation of max and min distances between key points.
        ROS_INFO("matches.size(): %lu", matches.size());

        for (int i = 0; i < matches.size(); i++)
        {
          double dist = matches[i].distance;
          if (dist < min_dist)
            min_dist = dist;
          if (dist > max_dist)
            max_dist = dist;
        }

        for (int i = 0; i < matches.size(); i++)
        {
          if (matches[i].distance < 3 * min_dist)
            good_matches.push_back(matches[i]);
        }

        vector<cv::Point3f> obj;
        vector<cv::Point3f> normal;
        vector<cv::Point2f> scene;

        ROS_INFO("good_matches.size(): %lu", good_matches.size());

        for (int i = 0; i < good_matches.size(); i++)
        {
          obj.push_back(listFeaturesPos3d_[selectedObj][good_matches[i].queryIdx]);
          normal.push_back(listFeaturesNormal3d_[selectedObj][good_matches[i].queryIdx]);
          scene.push_back(keypoints2[good_matches[i].trainIdx].pt);
          cv::circle(result, keypoints2[good_matches[i].trainIdx].pt, 3, cv::Scalar(255, 0, 0));
        }
        cv::imshow("result", result);

        cv::Mat mask;
        cv::Mat P = findPoseWithDepthCue(obj, normal, scene, optimizedFrame, mask);
        vector<cv::Point> listPoints;

        matches = good_matches;
        good_matches.clear();

        for (int i = 0; i < matches.size(); i++)
        {
          if (mask.at<unsigned char>(i, 0) > 0)
            good_matches.push_back(matches[i]);
        }

        ROS_INFO("good_matches.size() 2: %lu", good_matches.size());

        if (P.cols > 0)
        {
          //Extracting item pose
          cv::Mat R = getRVecFromMat(P);
          cv::Mat T = getTVecFromMat(P);
          geometry_msgs::PoseStamped itemPose;

          itemPose.pose.position.x = T.at<double>(0, 0);
          itemPose.pose.position.y = T.at<double>(1, 0);
          itemPose.pose.position.z = T.at<double>(2, 0);
          double x = R.at<double>(0, 0);
          double y = R.at<double>(1, 0);
          double z = R.at<double>(2, 0);
          double w = sqrt(x * x + y * y + z * z);
          itemPose.pose.orientation.x = x / w;
          itemPose.pose.orientation.y = y / w;
          itemPose.pose.orientation.z = z / w;
          itemPose.pose.orientation.w = w;

          res.poses.push_back(itemPose);

          conf[0].data = 1.0;
          cv::Scalar color(0, 255, 0);

          vector<cv::Point> listPoints, listModelPoints;

          for (int i = 0; i < good_matches.size(); i++)
          {
            listPoints.push_back(keypoints2[good_matches[i].trainIdx].pt);
            cv::circle(frameImg, keypoints2[good_matches[i].trainIdx].pt, 3, cv::Scalar(255, 0, 0));
          }
          for (int i = 0; i < listFeaturesPos3d_[selectedObj].size(); i++)
          {
            cv::Point3f p = multMatVec(P, listFeaturesPos3d_[selectedObj][i]);
            p /= p.z;
            cv::Point3f uv = multMatVec(RgbdFrame::getKMat(optimizedFrame.color_intrin), p);
            listModelPoints.push_back(cv::Point(uv.x, uv.y));
          }
          std::vector<std::vector<cv::Point>> hulls(2);
          cv::convexHull(cv::Mat(listPoints), hulls[0], false);
          cv::convexHull(cv::Mat(listModelPoints), hulls[1], false);
          cv::drawContours(frameImg, hulls, 0, cv::Scalar(255, 255, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
          double area = hulls[0].size() >= 3 ? cv::contourArea(hulls[0]) : 0;
          double modelArea = hulls[1].size() >= 3 ? cv::contourArea(hulls[1]) : frameImg.cols * frameImg.rows;
          //printf("Area: %lf px, ratio %lf\n", area, area/(frameImg.cols * frameImg.rows));

          double matchScore = 0, areaScore = 0;
          if (good_matches.size() < 5)
            matchScore = 0.33 * good_matches.size() / 5;
          else if (good_matches.size() < 15)
            matchScore = 0.33 + 0.33 * (good_matches.size() - 5) / 10;
          else
            matchScore = std::min(1.0, 0.66 + 0.34 * (good_matches.size() - 15) / 35);

          if (area < modelArea / 10)
            areaScore = 0.33 * area / (modelArea / 10);
          else if (area < modelArea / 5)
            areaScore = 0.33 + 0.33 * (area - modelArea / 10) / (modelArea / 10);
          else
            areaScore = std::min(1.0, 0.66 + 0.34 * (area - modelArea / 5) / (modelArea - modelArea / 5));

          double finalScore = (sqrt(matchScore * areaScore) + (matchScore + areaScore) / 2) / 2;

          conf[0].data = finalScore;
          if (finalScore < 0.33)
            color = cv::Scalar(255 * finalScore / 0.33, 0, 255 * (0.33 - finalScore) / 0.33);
          else if (finalScore < 0.66)
            color = cv::Scalar(255 * (0.33 - (finalScore - 0.33)) / 0.33, 255 * (finalScore - 0.33) / 0.33,
                               255 * (finalScore - 0.33) / 0.33);
          else
            color = cv::Scalar(0, 255, 255 * (0.33 - (finalScore - 0.66)) / 0.33);

          for (int i = 0; i < listModelPoints.size(); i++)
          {
            cv::circle(frameImg, listModelPoints[i], 3, color);
          }

          char txt[255];
          sprintf(txt, "matches %lu, score %f", good_matches.size(), finalScore);
          cv::putText(frameImg, txt, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
          //printf("COM in camera reference frame: %s\n", txt);
        }

        // Show detected matches.
        char filename[255];
        sprintf(filename, "Good Matches & Object detection %d", frameId);
        cv::imshow(filename, frameImg);

        cv::waitKey(1);

        res.items_ids.push_back(req.target_id);
        res.confidences.push_back(conf[0]);
      }

      return true;

    }
    else
    {
      ROS_ERROR("No matching database found for %s", req.target_id.data.c_str());

      std_msgs::String nothing;
      nothing.data = "";
      std_msgs::Float64 nullo;
      nullo.data = 0.0;

      res.items_ids.push_back(nothing);
      res.confidences.push_back(nullo);
      return true;
    }
  }
  else
  {
    ROS_ERROR("Node still in INIT state - Can`t work like this.");
    return false;
  }
}

/**
 * Service created to receive the items information data from the task manager node
 */
bool CloudMatchingNode::setItemsInfoCallback(tnp_feat_vision::set_items_info::Request &req,
                                             tnp_feat_vision::set_items_info::Response &res)
{
  return true;
}

bool CloudMatchingNode::setItemsPicsCallback(tnp_feat_vision::set_items_pics::Request &req,
                                             tnp_feat_vision::set_items_pics::Response &res)
{
  return true;
}

bool CloudMatchingNode::setRSBackgroundCallback(tnp_feat_vision::set_rs_background::Request &req,
                                                tnp_feat_vision::set_rs_background::Response &res)
{
  ROS_INFO("SetBackgroundCallback triggered.");

  vector<sensor_msgs::Image> emptyImg;
  vector<sensor_msgs::CameraInfo> emptyInfo;

  vector<RgbdFrame> scenery = Helpers::rosToRGBDFrame(req.cam_id, req.rgb_data, emptyImg, req.rgb_info_data, emptyInfo);
  // transform 25 pic into 5*5 array

  int id = 0;
  for (int i = 0; i < req.num_camera_per_scene.size(); i++)
  {
    vector<RgbdFrame> tmp;
    for (int j = 0; j < req.num_camera_per_scene[i].data; j++)
    {
      tmp.push_back(scenery[id]);
      id++;
    }
    background_.push_back(tmp);
   //bbx_extractor_.updateBackgroundFrame(tmp); // not needed in bbx_extractor
  }

  return true;
}

/**
 * weak string hashing to a number, collision for more complex strings
 * like str2int("WS") == str2int("tP") and str2int("5g") == str2int("sa")
 * @param str string
 * @param h
 * @return a weak hash in int
 */
constexpr unsigned int str2int(const char* str, int h /*= 0*/)
{
  return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

/**
 * Calculates the bounding box of an unknown item in the recognition space
 *
 * @param req a set of rgbd pictures from the recognition space
 * @param res pose of the center of the bound box and its and dimensions.  bb_width rel. bb_pose.x, bb_length rel. bb_pose.y,  bb_height rel. bb_pose.z
 * @return bounding box pose and dimensions
 */
bool CloudMatchingNode::getBoundingBoxCallback(tnp_feat_vision::get_bounding_box::Request &req,
                                               tnp_feat_vision::get_bounding_box::Response &res)
{
  vector<RgbdFrame> scenery = Helpers::rosToRGBDFrame(req.cam_id, req.rgb_data, req.depth_data, req.rgb_info_data,
                                                      req.depth_info_data);
  vector<cv::Mat> boundingBoxes;

  bbx_extractor_.compute(scenery);

// draw bounding boxes into images and publish them
  for (int i = 0; i < scenery.size(); i++)
  {
    cv::Mat abbx = bbx_extractor_.generateView(scenery[i], bbx_extractor_._recoBBOX, bbx_extractor_._OBB,
                                               bbx_extractor_._AABB);

    switch (str2int(scenery[i].camSerial.c_str()))
    {
      case str2int("L"):
        pub_bounding_box_L_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", abbx).toImageMsg());
        break;
      case str2int("C"):
        pub_bounding_box_C_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", abbx).toImageMsg());
        break;
      case str2int("R"):
        pub_bounding_box_R_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", abbx).toImageMsg());
        break;
      case str2int("B"):
        pub_bounding_box_B_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", abbx).toImageMsg());
        break;
      default:
        ROS_ERROR("Dafuq, camID '%s'not matching. Can`t publish that pic.", scenery[i].camSerial.c_str());
    }
  }

// retrieve bounding box results
  if (bbx_extractor_._OBB.size() > 0)
  {
    cv::Point3f obb_center = extractCenterFromOBB(bbx_extractor_._OBB);
    cv::Point3f obb_size = extractSizeFromOBB(bbx_extractor_._OBB);
    cv::Mat obb_rot = extractRotationFromOBB(bbx_extractor_._OBB);

    // transform them into ROS messages
    tf::Matrix3x3 m = tf::Matrix3x3(obb_rot.at<double>(0, 0), obb_rot.at<double>(0, 1), obb_rot.at<double>(0, 2),
                                    obb_rot.at<double>(1, 0), obb_rot.at<double>(1, 1), obb_rot.at<double>(1, 2),
                                    obb_rot.at<double>(2, 0), obb_rot.at<double>(2, 1), obb_rot.at<double>(2, 2));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    geometry_msgs::Point tmp_center;
    tmp_center.x = obb_center.x;
    tmp_center.y = obb_center.y;
    tmp_center.z = obb_center.z;

    geometry_msgs::PoseStamped poseStmpd;
    poseStmpd.pose.orientation.x = q.x();
    poseStmpd.pose.orientation.y = q.y();
    poseStmpd.pose.orientation.z = q.z();
    poseStmpd.pose.orientation.w = q.w();
    poseStmpd.pose.position = tmp_center;

    std_msgs::Float64 size_x, size_y, size_z;

    size_x.data = obb_size.x;
    size_y.data = obb_size.y;
    size_z.data = obb_size.z;

    res.bb_pose = poseStmpd;
    res.bb_width = size_x;
    res.bb_length = size_y;
    res.bb_height = size_z;
  }

  return true;
}

bool CloudMatchingNode::backgroundSubtraction(tnp_feat_vision::remove_background::Request &req,
                                              tnp_feat_vision::remove_background::Response &res)
{
  vector<RgbdFrame> scenery = Helpers::rosToRGBDFrame(req.cam_id, req.rgb_data, req.depth_data, req.rgb_info_data,
                                                      req.depth_info_data);

  for(int i = 0; i < scenery.size(); i++)
    cv::cvtColor(scenery[i].img, scenery[i].img, CV_RGB2BGR); // TODO: Hotfix

  vector<cv::Mat> br_objects = recognitionSpaceBackgroundSubtractor(bbx_extractor_, background_, scenery,
                                                                     bbx_extractor_.cameraSerial,
                                                                     bbx_extractor_.cameraK, bbx_extractor_.cameraDist,
                                                                     bbx_extractor_.cameraPoses, false);
  cv::imshow("background substracted result", br_objects[0]);
  res.cam_id = req.cam_id;

  for (int i = 0; i < br_objects.size(); i++)
  {
    // cv::cvtColor(br_objects[i], br_objects[i], CV_BGR2RGB); // TODO: Bugfix
    res.rgb_data.push_back(*cv_bridge::CvImage(std_msgs::Header(), "bgr8", br_objects[i]).toImageMsg());
  }

  if (res.rgb_data.size() > 0)
    return true;
  else
    return false;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tnp_cloud_matching");
  ros::NodeHandle nh;

  CloudMatchingNode cloudMatchingN(nh);

  ros::spin();
}
