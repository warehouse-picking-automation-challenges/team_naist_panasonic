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

#ifndef ENDEFFECTORQRDETECTOR_H
#define ENDEFFECTORQRDETECTOR_H

#include <opencv2/opencv.hpp>
#include "RGBDFrame.h"
#include "UtilCvPclRs.h"

//collision detection between ray and a cylinder aligned with X axis
bool rayCylinderXalignedIntersection(cv::Point3d p0, cv::Point3d dir, cv::Point3d cylinderCenter, double height, double radius, cv::Point3f& result);

//find the position in X axis of the QR codes of the end effector
//input
//suctionCylinderCenter, suctionCylinderHeight, suctionCylinderRadius : calibration position of the end effector (when no suction)
//listFrames : one set of frames from reco space
//serialId, cameraK, cameraDist, cameraPoses : calibration parameters
//dictionary, minId, maxId : QR code dictionary and valid range or the id)
//output :
//camOrig : position of the cameras in the reco space
//cylinderIntersections : intersection of the QR code ray with the calibrated suction cylinder(first : cam pos, second : intersection)
//qrCodeVec : all the projected vectors of the borders of qr code
//qrCodePoints : list of 4-points sets of corners of detected QR code
//height : detected height (in X axis) of the QR code
//return : true if valid position found, false otherwise
bool findEndEffectorQRPosX(cv::Point3f suctionCylinderCenter, float suctionCylinderHeight, float suctionCylinderRadius, const std::vector<RgbdFrame>& listFrames, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, cv::Ptr<cv::aruco::Dictionary> dictionary, int minId, int maxId, std::vector<cv::Point3f>& camOrig, std::vector<std::pair<cv::Point3f, cv::Point3f> >& cylinderIntersections, std::vector<std::vector<std::vector<cv::Point3f> > >& qrCodeVec, std::vector<std::vector<cv::Point3f> >& qrCodePoints, float& height);

#endif
