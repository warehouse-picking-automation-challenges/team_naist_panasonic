#ifndef FEATURESMATCHER_H
#define FEATURESMATCHER_H

#include <opencv2/opencv.hpp>
#include "RGBDFrame.h"

cv::Mat findPoseWith2PointsAndNormal(cv::Point3f p1, cv::Point3f n1, cv::Point3f p2, cv::Point3f origP1, cv::Point3f origN1, cv::Point3f origP2);
cv::Mat findPoseWithDepthCue(const std::vector<cv::Point3f> &obj, const std::vector<cv::Point3f> &objNormal, const std::vector<cv::Point2f> &scene, RgbdFrame frame, cv::Mat &mask);

#endif // FEATURESMATCHER_H
