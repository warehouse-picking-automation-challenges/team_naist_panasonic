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

#include "FeaturesMatcher.h"
#include "UtilCvPclRs.h"

cv::Mat findPoseWith2PointsAndNormal(cv::Point3f p1, cv::Point3f n1, cv::Point3f p2, cv::Point3f origP1, cv::Point3f origN1, cv::Point3f origP2)
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

cv::Mat findPoseWithDepthCue(const std::vector<cv::Point3f> &obj, const std::vector<cv::Point3f> &objNormal, const std::vector<cv::Point2f> &scene, RgbdFrame frame, cv::Mat &mask)
{
  bool planar_test = false;
  std::vector<cv::Point3f> newObj;
  std::vector<cv::Point3f> newObjNormal;
  std::vector<cv::Point2f> newScene;
  std::vector<cv::Point3f> newScenePoints;
  std::vector<cv::Point3f> newSceneNormal;
  mask = cv::Mat(obj.size(), 1, CV_8UC1);
  mask.setTo(cv::Scalar(0));
  std::vector<int> selectedId;
  for (int i = 0; i < scene.size(); i++) {
    int id = int(scene[i].y) * frame.imgPoints->width + int(scene[i].x);
    pcl::PointXYZ p1 = frame.imgPoints->points[id];
    cv::Point3f p(p1.x, p1.y, p1.z);
    if (sqrt(p.dot(p)) > 0) {
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
  for (int i = 0; i < newObj.size(); i++) {
    goodPairsTable[i] = std::vector<bool>(newObj.size());
    for (int j = 0; j < newObj.size(); j++)
      goodPairsTable[i][j] = false;
  }
  for (int i = 0; i < newObj.size(); i++) {
    for (int j = i + 1; j < newObj.size(); j++) {
      float angleDiff = fabs(acos(newSceneNormal[i].dot(newSceneNormal[j])));
      float dist = sqrt((newObj[i] - newObj[j]).dot(newObj[i] - newObj[j]));
      float dist2 = sqrt((newScenePoints[i] - newScenePoints[j]).dot(newScenePoints[i] - newScenePoints[j]));
      if (dist > 0.95 * dist2 && dist < 1.05 * dist2 && (!planar_test || angleDiff < 30 * CV_PI / 180)) {
        goodPairs[i].push_back(j);
        goodPairs[j].push_back(i);
        goodPairsTable[i][j] = true;
        goodPairsTable[j][i] = true;
      }
    }
  }
  cv::Mat bestPose;
  std::vector<int> bestValidPoints;
  for (int iter = 0; iter < 10000; iter++) {

    int id1 = rand() % newObj.size();
    if (goodPairs[id1].size() < 3)
      continue;
    int id2 = goodPairs[id1][rand() % goodPairs[id1].size()];

    cv::Mat pose = findPoseWith2PointsAndNormal(newScenePoints[id1], newSceneNormal[id1], newScenePoints[id2], newObj[id1], newObjNormal[id1], newObj[id2]);

    std::vector<int> validPoints;
    for (int i = 0; i < newObj.size(); i++) {
      cv::Point3f delta = newScenePoints[i] - multMatVec(pose, newObj[i]);
      float dist = delta.dot(delta);
      if (dist < 0.005 * 0.005)
        validPoints.push_back(i);
    }
    if (validPoints.size() > bestValidPoints.size()) {

      bestPose = pose;
      bestValidPoints = validPoints;
    }
  }
  for (int i = 0; i < bestValidPoints.size(); i++) {
    mask.at<unsigned char>(selectedId[bestValidPoints[i]], 0) = 1;
  }
  return bestPose;
}
