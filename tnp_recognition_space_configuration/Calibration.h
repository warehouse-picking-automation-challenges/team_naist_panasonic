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

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "RGBDFrame.h"
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

void calibrate(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, const std::vector<std::vector<RgbdFrame> >& listFrames, std::vector<cv::Mat>& cameraKOut, std::vector<cv::Mat>& cameraDistOut, std::vector<cv::Mat>& cameraPosesOut, int nbInfraredCam = 0);
void calibrateMultiboard(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<std::vector<RgbdFrame> >& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, std::vector<cv::Mat>& newCameraPoses, std::vector<cv::Mat>& objectRt, std::vector<cv::Mat>& planeRt, bool optimizePlaneRt = true, bool useDepth = true);
cv::Mat detectMultiboardPose(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<RgbdFrame>& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, const std::vector<cv::Mat>& planeRt);
void getCameraPoseFromMultiboard(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<std::vector<RgbdFrame> >& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& planeRt, std::vector<cv::Mat>& cameraPoses);

#endif // CALIBRATION_H
