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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"

#include <Eigen/Core>

#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <librealsense/rs.h>
#include <librealsense/rs.hpp>

#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#define NOISY 3.5

cv::Mat color_image;
cv::Mat depth_image;

bool color_image_ishere;
bool color_info_ishere;
bool depth_image_ishere;
bool depth_info_ishere;

float color_info_fx;
float color_info_fy;
float color_info_ppx;
float color_info_ppy;
float color_info_tx;
float color_info_ty;
float color_info_tz;

float depth_info_fx;
float depth_info_fy;
float depth_info_ppx;
float depth_info_ppy;
float depth_info_tx;
float depth_info_ty;
float depth_info_tz;

cv::Mat depth_to_color_extrinsics;
cv::Mat color_K;
cv::Mat depth_K;
bool USE_HARDCODED_ROTATION = true;

std::vector<std::vector<cv::Mat>> listImg;
std::vector<std::vector<cv::Mat>> listRectifiedImg;
std::vector<std::vector<std::vector<cv::Point3f>>>listCorners;
std::vector<cv::Point3f> listSize;

// Variables for item pose and confidence
geometry_msgs::PoseStamped itemPose;
std_msgs::Float64 conf[5];

std::vector<std::vector<std::vector<cv::KeyPoint>>>keypoints1;
std::vector<std::vector<cv::Mat>> descriptors1;
cv::Ptr<cv::DescriptorMatcher> matcher;
cv::Ptr<cv::AKAZE> akaze;
cv::Ptr<cv::ORB> orb;

ros::Publisher pub_item_pose, pub_confidence, pub_image_id;
ros::ServiceServer getItemPosService;

int w, h, depth;
float scale;

int selectedImg = 0;

cv::Point3f multMatVec(cv::Mat mat, cv::Point3f p)
{
  cv::Point3f p2;
  cv::Mat vec(mat.cols, 1, mat.type());

  if (vec.type() == CV_32F)
  {
    vec.at<float>(0, 0) = p.x;
    vec.at<float>(1, 0) = p.y;
    vec.at<float>(2, 0) = p.z;
    if (mat.cols == 4)
      vec.at<float>(3, 0) = 1.0;
  }
  else
  {
    vec.at<double>(0, 0) = p.x;
    vec.at<double>(1, 0) = p.y;
    vec.at<double>(2, 0) = p.z;
    if (mat.cols == 4)
      vec.at<double>(3, 0) = 1.0;
  }

  cv::Mat res = mat * vec;
  if (res.rows == 4)
  {
    if (res.type() == CV_32F)
      res = res / res.at<float>(3, 0);
    else
      res = res / res.at<double>(3, 0);
  }

  if (res.type() == CV_32F)
  {
    p2.x = res.at<float>(0, 0);
    p2.y = res.at<float>(1, 0);
    p2.z = res.at<float>(2, 0);
  }
  else
  {
    p2.x = res.at<double>(0, 0);
    p2.y = res.at<double>(1, 0);
    p2.z = res.at<double>(2, 0);
  }
  return p2;
}

cv::Mat point2Mat(cv::Point3d p)
{
  cv::Mat m(3, 1, CV_64F);
  m.at<double>(0, 0) = p.x;
  m.at<double>(1, 0) = p.y;
  m.at<double>(2, 0) = p.z;
  return m;
}

cv::Mat RTVec2Mat(cv::Mat rvec, cv::Mat tvec)
{
  if (rvec.empty() || tvec.empty())
  {
    printf("Error RTVec2Mat empty mat\n");
    return cv::Mat::eye(4, 4, CV_64F);
  }
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Mat Rmat = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat Tmat = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(Rmat(cv::Rect(0, 0, 3, 3)));
  tvec.copyTo(Tmat(cv::Rect(3, 0, 1, 3)));

  return Tmat * Rmat;
}

cv::Mat getRVecFromMat(cv::Mat M)
{
  cv::Mat rvec;
  cv::Rodrigues(M(cv::Rect(0, 0, 3, 3)), rvec);
  return rvec;
}

cv::Mat getTVecFromMat(cv::Mat M)
{
  return M(cv::Rect(3, 0, 1, 3)).clone();
}

cv::Point3f pcl2cv(pcl::Normal n)
{
  return cv::Point3f(n.normal_x, n.normal_y, n.normal_z);
}

cv::Point2f rs2cv(rs::float2 a)
{
  return cv::Point2f(a.x, a.y);
}

cv::Point3f rs2cv(rs::float3 a)
{
  return cv::Point3f(a.x, a.y, a.z);
}

rs::float2 cv2rs(cv::Point2f a)
{
  return
  { a.x, a.y};
}

rs::float3 cv2rs(cv::Point3f a)
{
  return
  { a.x, a.y, a.z};
}

cv::Point3f transformByMat44(cv::Mat M, cv::Point3f p)
{
  cv::Point3f p2;
  float w = M.at<float>(3, 0) * p.x + M.at<float>(3, 1) * p.y + M.at<float>(3, 2) * p.z + M.at<float>(3, 3);
  p2.x = (M.at<float>(0, 0) * p.x + M.at<float>(0, 1) * p.y + M.at<float>(0, 2) * p.z + M.at<float>(0, 3)) / w;
  p2.y = (M.at<float>(1, 0) * p.x + M.at<float>(1, 1) * p.y + M.at<float>(1, 2) * p.z + M.at<float>(1, 3)) / w;
  p2.z = (M.at<float>(2, 0) * p.x + M.at<float>(2, 1) * p.y + M.at<float>(2, 2) * p.z + M.at<float>(2, 3)) / w;
  return p2;
}

cv::Mat findPoseWith2PointsAndNormal(cv::Point3f p1, cv::Point3f n1, cv::Point3f p2, cv::Point3f origP1,
                                     cv::Point3f origN1, cv::Point3f origP2)
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

class RgbdFrame
{
public:
  cv::Mat depth; // Depth camera, depth value CV_16UC1.
  cv::Mat img; // Color camera, RGB value CV_8UC3.
  cv::Mat imgDepth; // Color camera, depth value CV_32F.
  pcl::PointCloud<pcl::PointXYZ>::Ptr imgPoints; // Color camera, point cloud.
  pcl::PointCloud<pcl::Normal>::Ptr imgNormals; // Color camera, normals.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rs_cloud_ptr; // Depth camera, point cloud (with RGB from projection on color camera).

  rs::intrinsics depth_intrin;
  rs::extrinsics depth_to_color;
  rs::intrinsics color_intrin;
  float scale;

  RgbdFrame()
  {
    rs_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    imgPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    imgNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  // Project the 3D point to the 2D color image.
  cv::Point2f colorCameraProject(cv::Point3f p)
  {
    return rs2cv(color_intrin.project(cv2rs(p)));
  }

  cv::Point2f depthCameraProject(cv::Point3f p)
  {
    return rs2cv(depth_intrin.project(cv2rs(p)));
  }

  cv::Point3f depthCameraToColorCamera(cv::Point3f p)
  {
    return rs2cv(depth_to_color.transform(cv2rs(p)));
  }

  cv::Point3f depthCameraDeproject(cv::Point2f p, float distance_in_meter)
  {
    return rs2cv(depth_intrin.deproject(cv2rs(p), distance_in_meter));
  }
};

cv::Mat findPoseWithDepthCue(const std::vector<cv::Point3f> &obj, const std::vector<cv::Point3f> &objNormal,
                             const std::vector<cv::Point2f> &scene, RgbdFrame frame, cv::Mat &mask)
{
  bool planar_test = true;
  std::vector<cv::Point3f> newObj;
  std::vector<cv::Point3f> newObjNormal;
  std::vector<cv::Point2f> newScene;
  std::vector<cv::Point3f> newScenePoints;
  std::vector<cv::Point3f> newSceneNormal;
  mask = cv::Mat(obj.size(), 1, CV_8UC1);
  mask.setTo(cv::Scalar(0));
  std::vector<int> selectedId;
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
  std::vector<std::vector<int>> goodPairs(newObj.size());
  std::vector<std::vector<bool>> goodPairsTable(newObj.size());
  for (int i = 0; i < newObj.size(); i++)
  {
    goodPairsTable[i] = std::vector<bool>(newObj.size());
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
  std::vector<int> bestValidPoints;
  for (int iter = 0; iter < 10000; iter++)
  {

    int id1 = rand() % newObj.size();
    if (goodPairs[id1].size() < 3)
      continue;
    int id2 = goodPairs[id1][rand() % goodPairs[id1].size()];

    cv::Mat pose = findPoseWith2PointsAndNormal(newScenePoints[id1], newSceneNormal[id1], newScenePoints[id2],
                                                newObj[id1], newObjNormal[id1], newObj[id2]);

    std::vector<int> validPoints;
    for (int i = 0; i < newObj.size(); i++)
    {
      cv::Point3f delta = newScenePoints[i] - transformByMat44(pose, newObj[i]);
      float dist = delta.dot(delta);
      if (dist < 0.001 * 0.001)
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

RgbdFrame extractDataFromRos(cv::Mat colorK, cv::Mat colorImg, cv::Mat depthK, cv::Mat depthImg, cv::Mat Extrinsic)
{
  RgbdFrame frame;

  frame.depth_intrin = rs::intrinsics();
  frame.depth_intrin.width = depthImg.cols;
  frame.depth_intrin.height = depthImg.rows;
  frame.depth_intrin.ppx = depthK.at<double>(0, 2);
  frame.depth_intrin.ppy = depthK.at<double>(1, 2);
  frame.depth_intrin.fx = depthK.at<double>(0, 0);
  frame.depth_intrin.fy = depthK.at<double>(1, 1);
  ((rs_intrinsics *)&frame.depth_intrin)->model = (rs_distortion)rs::distortion::inverse_brown_conrady;
  for (int i = 0; i < 5; i++)
    frame.depth_intrin.coeffs[i] = 0; // Approximated to 0 for now.

  frame.color_intrin = rs::intrinsics();
  frame.color_intrin.width = colorImg.cols;
  frame.color_intrin.height = colorImg.rows;
  frame.color_intrin.ppx = colorK.at<double>(0, 2);
  frame.color_intrin.ppy = colorK.at<double>(1, 2);
  frame.color_intrin.fx = colorK.at<double>(0, 0);
  frame.color_intrin.fy = colorK.at<double>(1, 1);
  ((rs_intrinsics *)&frame.color_intrin)->model = (rs_distortion)rs::distortion::modified_brown_conrady;
  for (int i = 0; i < 5; i++)
    frame.color_intrin.coeffs[i] = 0; // Approximated to 0 for now.

  for (int i = 0; i < 3; i++)
  {
    frame.depth_to_color.translation[i] = Extrinsic.at<double>(i, 3);
    for (int j = 0; j < 3; j++)
      frame.depth_to_color.rotation[j * 3 + i] = Extrinsic.at<double>(i, j);
  }
  frame.scale = 0.001;

  frame.depth = depthImg.clone();
  frame.img = colorImg.clone();

  return frame;
}

bool rayTriangleIntersect(const cv::Point3f &orig, const cv::Point3f &dir, const cv::Point3f &v0, const cv::Point3f &v1,
                          const cv::Point3f &v2, float &t)
{
  // Compute plane's normal.
  cv::Point3f v0v1 = v1 - v0;
  cv::Point3f v0v2 = v2 - v0;
  // No need to normalize.
  cv::Point3f N = v0v1.cross(v0v2);
  float area2 = sqrt(N.dot(N));

  // Step 1: Finding P.

  // Check if the ray and the plane are parallel.
  float NdotRayDirection = N.dot(dir);
  if (fabs(NdotRayDirection) < 0.00001) // Almost 0.
    return false; // They are parallel so they don't intersect.

  // Compute d parameter using equation 2.
  float d = N.dot(v0);

  // Compute t using equation 3.
  t = (N.dot(orig) + d) / NdotRayDirection;
  // Check if the triangle is in behind the ray.
  if (t < 0)
    return false; // The triangle is behind.

  // Compute the intersection point using equation 1.
  cv::Point3f P = orig + t * dir;

  // Step 2: Inside-outside test.

  cv::Point3f C; // Vector perpendicular to the triangle's plane.

  // Edge 0:
  cv::Point3f edge0 = v1 - v0;
  cv::Point3f vp0 = P - v0;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0)
    return false; // P is on the right side.

  // Edge 1:
  cv::Point3f edge1 = v2 - v1;
  cv::Point3f vp1 = P - v1;
  C = edge1.cross(vp1);
  if (N.dot(C) < 0)
    return false; // P is on the right side.

  // Edge 2:
  cv::Point3f edge2 = v0 - v2;
  cv::Point3f vp2 = P - v2;
  C = edge2.cross(vp2);
  if (N.dot(C) < 0)
    return false; // P is on the right side.

  return true; // This ray hits the triangle.
}

void Barycentric(cv::Point2f p, cv::Point2f a, cv::Point2f b, cv::Point2f c, float &u, float &v, float &w)
{
  cv::Point2f v0 = b - a, v1 = c - a, v2 = p - a;
  float d00 = v0.dot(v0);
  float d01 = v0.dot(v1);
  float d11 = v1.dot(v1);
  float d20 = v2.dot(v0);
  float d21 = v2.dot(v1);
  float denom = d00 * d11 - d01 * d01;
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;
}

// Get the raw data and build the current frames cloud.
RgbdFrame generatePointCloudOrganized2(cv::Mat colorK, cv::Mat colorImg, cv::Mat depthK, cv::Mat depthImg,
                                       cv::Mat Extrinsic)
{

  RgbdFrame frame = extractDataFromRos(colorK, colorImg, depthK, depthImg, Extrinsic);
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

  static const float nan = std::numeric_limits<float>::quiet_NaN();

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
        frame.rs_cloud_ptr->points[i].x = (float)nan;
        frame.rs_cloud_ptr->points[i].y = (float)nan;
        frame.rs_cloud_ptr->points[i].z = (float)nan;
        continue;
      }

      cv::Point2f depth_pixel((float)dx, (float)dy);
      float depth_in_meters = depth_value * frame.scale;

      cv::Point3f depth_point = frame.depthCameraDeproject(depth_pixel, depth_in_meters);
      cv::Point3f color_point = frame.depthCameraToColorCamera(depth_point);
      cv::Point2f color_pixel = frame.colorCameraProject(color_point);

      const int cx = (int)std::round(color_pixel.x);
      const int cy = (int)std::round(color_pixel.y);

      // Set up logic to remove bad points.
      bool depth_fail = true;
      bool color_fail = true;

      depth_fail = (depth_point.z > NOISY);
      color_fail = (cx < 0 || cy < 0 || cx > frame.img.cols || cy > frame.img.rows);

      // Cloud input pointers.

      // XYZ input access to cloud.
      float *dp_x;
      float *dp_y;
      float *dp_z;

      dp_x = &(frame.rs_cloud_ptr->points[i].x);
      dp_y = &(frame.rs_cloud_ptr->points[i].y);
      dp_z = &(frame.rs_cloud_ptr->points[i].z);

      // RGB input access to cloud.
      uint8_t *cp_r;
      uint8_t *cp_g;
      uint8_t *cp_b;

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
      uint8_t *offset = frame.img.ptr<uint8_t>(cy) + cx * 3;

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
        *dp_x = *dp_y = *dp_z = (float)nan;
        *cp_r = *cp_g = *cp_b = 255;
        continue;
      }

      // If valid point, add data to cloud.
      else
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
        cv::Point2f depth_pixel((float)x[k], (float)y[k]);
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
        cv::Point2f pixMin = {std::min(color_pixelTri[0].x, std::min(color_pixelTri[1].x, color_pixelTri[2].x)),
                              std::min(color_pixelTri[0].y, std::min(color_pixelTri[1].y, color_pixelTri[2].y))};
        cv::Point2f pixMax = {std::max(color_pixelTri[0].x, std::max(color_pixelTri[1].x, color_pixelTri[2].x)),
                              std::max(color_pixelTri[0].y, std::max(color_pixelTri[1].y, color_pixelTri[2].y))};
        for (int x = (int)floor(pixMin.x); x < (int)ceil(pixMax.x); x++)
          for (int y = (int)floor(pixMin.y); y < (int)ceil(pixMax.y); y++)
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

bool allThere()
{
  if (color_image_ishere and color_info_ishere and depth_image_ishere and depth_info_ishere)
  {
    RgbdFrame frame = generatePointCloudOrganized2(color_K, color_image, depth_K, depth_image,
                                                   depth_to_color_extrinsics);

    cv::Mat frameImg;
    frameImg = frame.img.clone();

    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    akaze->detectAndCompute(frameImg, cv::Mat(), keypoints2, descriptors2);

    if (keypoints2.size() == 0)
      return true;

    std::vector<std::vector<cv::DMatch>> matches(listImg[selectedImg].size());
    std::vector<std::vector<cv::DMatch>> good_matches(listImg[selectedImg].size());
    for (int face = 0; face < listImg[selectedImg].size(); face++)
    {
      matcher->match(descriptors1[selectedImg][face], descriptors2, matches[face]);

      double max_dist = 0;
      double min_dist = 100;
      // Quick calculation of max and min distances between keypoints.
      for (int i = 0; i < matches[face].size(); i++)
      {
        double dist = matches[face][i].distance;
        if (dist < min_dist)
          min_dist = dist;
        if (dist > max_dist)
          max_dist = dist;
      }

      //printf("Max dist: %f \n", max_dist );
      //printf("Min dist: %f \n", min_dist );

      // Draw only good matches (whose distance is less than 3*min_dist)
      for (int i = 0; i < matches[face].size(); i++)
      {
        if (matches[face][i].distance < 3 * min_dist)
          good_matches[face].push_back(matches[face][i]);
      }
    }

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point3f> normal;
    std::vector<cv::Point2f> scene;
    for (int face = 0; face < listImg[selectedImg].size(); face++)
      for (size_t i = 0; i < good_matches[face].size(); i++)
      {
        // Get the keypoints from the good matches.
        cv::Point2f p = keypoints1[selectedImg][face][good_matches[face][i].queryIdx].pt;
        if (listCorners[selectedImg][face][0].z == 1)
        {
          obj.push_back((0.001 / scale) * cv::Point3f(p.x, p.y, scale * listSize[selectedImg].z));
          normal.push_back(cv::Point3f(0, 0, 1));
        }
        else
        {
          obj.push_back((0.001 / scale) * cv::Point3f(p.x, p.y, 0));
          normal.push_back(cv::Point3f(0, 0, -1));
        }
        scene.push_back(keypoints2[good_matches[face][i].trainIdx].pt);
      }
    cv::Mat mask;
    cv::Mat P = findPoseWithDepthCue(obj, normal, scene, frame, mask);
    int startId = 0;
    for (int face = 0; face < listImg[selectedImg].size(); face++)
    {
      matches[face] = good_matches[face];
      good_matches[face].clear();
      for (int i = startId; i < startId + matches[face].size(); i++)
      {
        if (mask.at<unsigned char>(i, 0) > 0)
        {
          good_matches[face].push_back(matches[face][i - startId]);
        }
      }
      obj.clear();
      scene.clear();
      for (size_t i = 0; i < good_matches[face].size(); i++)
      {
        // Get the keypoints from the good matches.
        cv::Point2f p = keypoints1[selectedImg][face][good_matches[face][i].queryIdx].pt;
        obj.push_back((0.001 / scale) * cv::Point3f(p.x, p.y, 0));
        scene.push_back(keypoints2[good_matches[face][i].trainIdx].pt);
      }
      startId += matches[face].size();
    }
    std::vector<cv::Mat> img_matches(listImg[selectedImg].size());
    for (int face = 0; face < listImg[selectedImg].size(); face++)
      cv::drawMatches(listRectifiedImg[selectedImg][face], keypoints1[selectedImg][face], frameImg, keypoints2,
                      good_matches[face], img_matches[face]);

    for (int face = 0; face < listImg[selectedImg].size(); face++)
      if (P.cols > 0)
      {
        cv::Point3f tl = transformByMat44(P, (0.001 / scale) * cv::Point3f(0, 0, 0));
        cv::Point3f tr = transformByMat44(
            P, (0.001 / scale) * cv::Point3f(listRectifiedImg[selectedImg][face].cols, 0, 0));
        cv::Point3f br = transformByMat44(
            P,
            (0.001 / scale)
                * cv::Point3f(listRectifiedImg[selectedImg][face].cols, listRectifiedImg[selectedImg][face].rows, 0));
        cv::Point3f bl = transformByMat44(
            P, (0.001 / scale) * cv::Point3f(0, listRectifiedImg[selectedImg][face].rows, 0));

        cv::Point3f tlb = transformByMat44(P, (0.001 / scale) * cv::Point3f(0, 0, scale * listSize[selectedImg].z));
        cv::Point3f trb = transformByMat44(
            P,
            (0.001 / scale)
                * cv::Point3f(listRectifiedImg[selectedImg][face].cols, 0, scale * listSize[selectedImg].z));
        cv::Point3f brb = transformByMat44(
            P,
            (0.001 / scale)
                * cv::Point3f(listRectifiedImg[selectedImg][face].cols, listRectifiedImg[selectedImg][face].rows,
                              scale * listSize[selectedImg].z));
        cv::Point3f blb = transformByMat44(
            P,
            (0.001 / scale)
                * cv::Point3f(0, listRectifiedImg[selectedImg][face].rows, scale * listSize[selectedImg].z));

        cv::Point3f com = (tl + tr + br + bl + tlb + trb + brb + blb) / 8;

        // transform to camera coordinate system
        const cv::Mat ImgToCamera = RTVec2Mat(point2Mat(cv::Point3d(0, 0, -CV_PI / 2)),
                                              point2Mat(cv::Point3d(0, 0, 0))); // this may not be necessary as the camera is publishing also its frames
        com = multMatVec(ImgToCamera, com);

        //Publishing item pose into topic
        itemPose.pose.position.x = com.x;
        itemPose.pose.position.y = com.y;
        itemPose.pose.position.z = com.z;
        itemPose.pose.orientation.x = 0.0;
        itemPose.pose.orientation.y = 0.0;
        itemPose.pose.orientation.z = 0.0;
        itemPose.pose.orientation.w = 1.0;
        itemPose.header.stamp = ros::Time::now();
        itemPose.header.frame_id = "tnp_ee_camera_frame";
        pub_item_pose.publish(itemPose);

        cv::Point2f tl2 = rs2cv(frame.color_intrin.project(cv2rs(tl)));
        cv::Point2f tr2 = rs2cv(frame.color_intrin.project(cv2rs(tr)));
        cv::Point2f br2 = rs2cv(frame.color_intrin.project(cv2rs(br)));
        cv::Point2f bl2 = rs2cv(frame.color_intrin.project(cv2rs(bl)));

        cv::Point2f tl2b = rs2cv(frame.color_intrin.project(cv2rs(tlb)));
        cv::Point2f tr2b = rs2cv(frame.color_intrin.project(cv2rs(trb)));
        cv::Point2f br2b = rs2cv(frame.color_intrin.project(cv2rs(brb)));
        cv::Point2f bl2b = rs2cv(frame.color_intrin.project(cv2rs(blb)));

        conf[face].data = 1.0;
        cv::Scalar color(0, 255, 0);
        std::vector<cv::Point> listPoints;
        std::vector<std::vector<cv::Point>> hulls(1);
        for (int i = 0; i < good_matches[face].size(); i++)
        {
          listPoints.push_back(keypoints2[good_matches[face][i].trainIdx].pt);
        }
        cv::convexHull(cv::Mat(listPoints), hulls[0], false);
        cv::drawContours(
            img_matches[face](cv::Rect(listRectifiedImg[selectedImg][face].cols, 0, frameImg.cols, frameImg.rows)),
            hulls, 0, cv::Scalar(255, 255, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        double area = hulls[0].size() >= 3 ? cv::contourArea(hulls[0]) : 0;
        //printf("Area: %lf px, ratio %lf\n", area, area/(frameImg.cols * frameImg.rows));
        if (good_matches[face].size() < 5 || area < frameImg.cols * frameImg.rows / 400)
        {
          color = cv::Scalar(0, 0, 255);
          conf[face].data = 0.33;
        }
        else if (good_matches[face].size() < 10 || area < frameImg.cols * frameImg.rows / 20)
        {
          color = cv::Scalar(0, 255, 255);
          conf[face].data = 0.66;
        }

        char txt[255];
        sprintf(txt, "%lf, %lf, %lf", com.x, com.y, com.z);
        cv::putText(img_matches[face], txt, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        //printf("COM in camera reference frame: %s\n", txt);

        cv::line(img_matches[face], tl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tr2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], tr2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 br2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], br2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 bl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], bl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);

        cv::line(img_matches[face], tl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], tr2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tr2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], br2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 br2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], bl2 + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 bl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);

        cv::line(img_matches[face], tl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tr2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], tr2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 br2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], br2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 bl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
        cv::line(img_matches[face], bl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0),
                 tl2b + cv::Point2f(listRectifiedImg[selectedImg][face].cols, 0), color, 4);
      }

    // Show detected matches.
    char filename[255];
    for (int face = 0; face < listImg[selectedImg].size(); face++)
    {
      sprintf(filename, "Good Matches & Object detection %d", face);
      cv::imshow(filename, img_matches[face]);
      if (face == 0)
      {
        conf[4].data = conf[face].data;
      }
      else
      {
        if (conf[4].data < conf[face].data)
        {
          conf[4].data = conf[face].data;
        }
      }
    }

    //Publish the highest confidence among the different faces
    pub_confidence.publish(conf[4]);

    cv::waitKey(1);

    color_image_ishere = false;
    depth_image_ishere = false;
  }
}

void colorImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  color_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  color_image_ishere = true;
  allThere();
}

void colorInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  color_K = cv::Mat(3, 3, CV_64F, (void *)&msg->K[0], cv::Mat::AUTO_STEP).clone();

  color_info_fx = msg->K[0];
  color_info_fy = msg->K[4];
  color_info_ppx = msg->K[2];
  color_info_ppy = msg->K[5];
  color_info_tx = msg->P[3];
  color_info_ty = msg->P[7];
  color_info_tz = msg->P[11];
  color_info_ishere = true;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  depth_image = cv_bridge::toCvShare(msg, "16UC1")->image.clone();
  depth_image_ishere = true;
  allThere();
}

void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  if (USE_HARDCODED_ROTATION)
  {
    R.at<double>(0, 0) = 0.999987;
    R.at<double>(0, 1) = 0.004618;
    R.at<double>(0, 2) = 0.002142;
    R.at<double>(1, 0) = -0.004655;
    R.at<double>(1, 1) = 0.999834;
    R.at<double>(1, 2) = 0.017596;
    R.at<double>(2, 0) = -0.002060;
    R.at<double>(2, 1) = -0.017605;
    R.at<double>(2, 2) = 0.999843;
    R = R.t(); // Transpose because RS uses column-major, and OpenCV uses row-major.
  }
  depth_to_color_extrinsics = cv::Mat::eye(4, 4, CV_64F);
  depth_to_color_extrinsics.at<double>(0, 3) = msg->P[3];
  depth_to_color_extrinsics.at<double>(1, 3) = msg->P[7];
  depth_to_color_extrinsics.at<double>(2, 3) = msg->P[11];
  R.copyTo(depth_to_color_extrinsics(cv::Rect(0, 0, 3, 3)));

  depth_K = cv::Mat(3, 3, CV_64F, (void *)&msg->K[0], cv::Mat::AUTO_STEP).clone();

  depth_info_fx = msg->K[0];
  depth_info_fy = msg->K[4];
  depth_info_ppx = msg->K[2];
  depth_info_ppy = msg->K[5];
  depth_info_tx = msg->P[3];
  depth_info_ty = msg->P[7];
  depth_info_tz = msg->P[11];
  depth_info_ishere = true;
}

void itemIdCallback(const std_msgs::Int16::ConstPtr &msg)
{
  char filename[255];
  if (selectedImg != msg->data && msg->data <= listImg.size() - 1)
  {
    for (int face = 0; face < listImg[selectedImg].size(); face++)
    {
      sprintf(filename, "Good Matches & Object detection %d", face);
      cv::destroyWindow(filename);
    }

    selectedImg = msg->data;
  }
}


int main(int argc, char **argv)
{

  scale = 2.0;

  cv::Mat image;
  image = cv::imread("crayola.jpg");

  if (!image.data)
  {
    ROS_ERROR("NO IMAGES WHERE FOUND ON /ROOT/.ROS DIRECTORY THE PROCESS CANNOT CONTINUE");
    return 0;
  }

  std::vector<cv::Mat> crayolaImg;
  std::vector<std::vector<cv::Point3f>> crayolaCorners(1);
  crayolaImg.push_back(cv::imread("crayola.jpg"));
  crayolaCorners[0].push_back(cv::Point3f(389, 115, 0));
  crayolaCorners[0].push_back(cv::Point3f(1018, 117, 0));
  crayolaCorners[0].push_back(cv::Point3f(978, 661, 0));
  crayolaCorners[0].push_back(cv::Point3f(395, 616, 0));
  listImg.push_back(crayolaImg);
  listCorners.push_back(crayolaCorners);
  listSize.push_back(cv::Point3f(145, 125, 37));

  std::vector<cv::Mat> expoImg;
  std::vector<std::vector<cv::Point3f>> expoCorners(2);
  expoImg.push_back(cv::imread("expo1.jpg"));
  expoCorners[0].push_back(cv::Point3f(3308, 2173, 0));
  expoCorners[0].push_back(cv::Point3f(1059, 2178, 0));
  expoCorners[0].push_back(cv::Point3f(994, 1264, 0));
  expoCorners[0].push_back(cv::Point3f(3383, 1239, 0));
  expoImg.push_back(cv::imread("expo2.jpg"));
  expoCorners[1].push_back(cv::Point3f(843, 2256, 1));
  expoCorners[1].push_back(cv::Point3f(3544, 2122, 1));
  expoCorners[1].push_back(cv::Point3f(3534, 950, 1));
  expoCorners[1].push_back(cv::Point3f(692, 1160, 1));
  listImg.push_back(expoImg);
  listCorners.push_back(expoCorners);
  listSize.push_back(cv::Point3f(130, 53, 34));

  std::vector<cv::Mat> robotsImg;
  std::vector<std::vector<cv::Point3f>> robotsCorners(2);
  robotsImg.push_back(cv::imread("robots1.jpg"));
  robotsCorners[0].push_back(cv::Point3f(246, 998, 0));
  robotsCorners[0].push_back(cv::Point3f(2644, 928, 0));
  robotsCorners[0].push_back(cv::Point3f(2728, 3960, 0));
  robotsCorners[0].push_back(cv::Point3f(240, 3934, 0));
  robotsImg.push_back(cv::imread("robots2.jpg"));
  if (true)
  {
    robotsCorners[1].push_back(cv::Point3f(2880, 400, 1));
    robotsCorners[1].push_back(cv::Point3f(84, 516, 1));
    robotsCorners[1].push_back(cv::Point3f(99, 3907, 1));
    robotsCorners[1].push_back(cv::Point3f(2982, 3934, 1));
  }
  else
  {
    robotsCorners[1].push_back(cv::Point3f(84, 516, 0));
    robotsCorners[1].push_back(cv::Point3f(2880, 400, 0));
    robotsCorners[1].push_back(cv::Point3f(2982, 3934, 0));
    robotsCorners[1].push_back(cv::Point3f(99, 3907, 0));
  }
  listImg.push_back(robotsImg);
  listCorners.push_back(robotsCorners);
  listSize.push_back(cv::Point3f(166, 201, 4));

  std::vector<cv::Mat> pencilImg;
  std::vector<std::vector<cv::Point3f>> pencilCorners(2);
  pencilImg.push_back(cv::imread("pencil1.jpg"));
  pencilCorners[0].push_back(cv::Point3f(3400, 772, 0));
  pencilCorners[0].push_back(cv::Point3f(3396, 2256, 0));
  pencilCorners[0].push_back(cv::Point3f(88, 2240, 0));
  pencilCorners[0].push_back(cv::Point3f(132, 644, 0));
  pencilImg.push_back(cv::imread("pencil2.jpg"));
  pencilCorners[1].push_back(cv::Point3f(532, 1068, 1));
  pencilCorners[1].push_back(cv::Point3f(556, 2612, 1));
  pencilCorners[1].push_back(cv::Point3f(3920, 2648, 1));
  pencilCorners[1].push_back(cv::Point3f(3964, 1048, 1));
  listImg.push_back(pencilImg);
  listCorners.push_back(pencilCorners);
  listSize.push_back(cv::Point3f(91, 193, 9));

  std::vector<cv::Mat> kleenexImg;
  std::vector<std::vector<cv::Point3f>> kleenexCorners(1);
  kleenexImg.push_back(cv::imread("kleenex.jpg"));
  kleenexCorners[0].push_back(cv::Point3f(3564, 156, 0));
  kleenexCorners[0].push_back(cv::Point3f(3636, 2928, 0));
  kleenexCorners[0].push_back(cv::Point3f(864, 2964, 0));
  kleenexCorners[0].push_back(cv::Point3f(872, 132, 0));
  listImg.push_back(kleenexImg);
  listCorners.push_back(kleenexCorners);
  listSize.push_back(cv::Point3f(111, 111, 127));

  cv::Mat img = cv::imread("robots1.jpg");

  for (int i = 0; i < listImg.size(); i++)
  {
    std::vector<cv::Mat> img2(listImg[i].size());
    for (int j = 0; j < listImg[i].size(); j++)
    {
      std::vector<cv::Point2f> corners(4);
      for (int k = 0; k < 4; k++)
        corners[k] = cv::Point2f(listCorners[i][j][k].x, listCorners[i][j][k].y);
      std::vector<cv::Point2f> corners2(4);
      corners2[0] = scale * cv::Point2f(0, 0);
      corners2[1] = scale * cv::Point2f(listSize[i].x, 0);
      corners2[2] = scale * cv::Point2f(listSize[i].x, listSize[i].y);
      corners2[3] = scale * cv::Point2f(0, listSize[i].y);

      cv::Mat H = cv::findHomography(corners, corners2);
      cv::warpPerspective(listImg[i][j], img2[j], H,
                          cv::Size((int)(scale * (listSize[i].x)), (int)(scale * (listSize[i].y))));
      cv::waitKey(500);
    }
    listRectifiedImg.push_back(img2);
  }

  akaze = cv::AKAZE::create();

  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  for (int i = 0; i < listImg.size(); i++)
  {
    keypoints1.push_back(std::vector<std::vector<cv::KeyPoint>>(listRectifiedImg[i].size()));
    descriptors1.push_back(std::vector<cv::Mat>(listRectifiedImg[i].size()));
    for (int j = 0; j < listImg[i].size(); j++)
    {
      akaze->detectAndCompute(listRectifiedImg[i][j], cv::Mat(), keypoints1[i][j], descriptors1[i][j]);
    }
  }

  ros::init(argc, argv, "tnp_image_viewer");
  ros::init(argc, argv, "tnp_plane_matching");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_color_image, sub_depth_image;
  ros::Subscriber sub_color_info, sub_depth_info, sub_image_id;

  sub_color_image = it.subscribe("/tnp_image_converter/color/image_raw", 1, colorImageCallback);
  sub_color_info = nh.subscribe("/camera/color/camera_info", 1, colorInfoCallback);

  sub_depth_image = it.subscribe("/tnp_image_converter/depth/image_raw", 1, depthImageCallback);
  sub_depth_info = nh.subscribe("/camera/depth/camera_info", 1, depthInfoCallback);

  //Subscriber to selectedImg for changing between different objects
  sub_image_id = nh.subscribe("/tnp_vision/feature_based/target_item_id", 1, itemIdCallback);

  //Publishers for Item Pose and Confidence
  pub_item_pose = nh.advertise<geometry_msgs::PoseStamped>("/tnp_vision/feature_based/item_pose", 1);
  pub_confidence = nh.advertise<std_msgs::Float64>("/tnp_vision/feature_based/confidence", 1);
  pub_image_id = nh.advertise<std_msgs::Int16>("/tnp_vision/feature_based/target_item_id", 1);

  ros::spin();

  return 0;
}
