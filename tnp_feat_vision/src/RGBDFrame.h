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

#ifndef RGBDFRAME_H
#define RGBDFRAME_H

#include <opencv2/opencv.hpp>
#include <librealsense/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define NOISY       3.5		// Remove points past NOISY meters
#define FPS_MILLI   500		// Update fps every 0.5 seconds

class RgbdFrame
{
public:
    bool useDepth, useIR, useIR2;
    std::string camSerial;
    cv::Mat depth;//depth camera, depth value CV_16UC1
    cv::Mat img;//color camera, rgb value CV_8UC3
    cv::Mat mask;//mask of img, CV_8UC1
    cv::Mat imgIR, imgIR2;
    cv::Mat imgDepth;//color camera, depth value CV_32F
    pcl::PointCloud<pcl::PointXYZ>::Ptr imgPoints;//color camera point cloud
    pcl::PointCloud<pcl::Normal>::Ptr imgNormals;//color camera normals
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rs_cloud_ptr;//depth camera, point cloud (with rgb from projection on color camera)
    pcl::PointCloud<pcl::Normal>::Ptr rs_cloud_normal_ptr;//depth camera, normal

    //realsense intrinsics
    rs::intrinsics depth_intrin;
    rs::extrinsics depth_to_color;
    rs::intrinsics color_intrin;

    rs::intrinsics ir_intrin;
    rs::intrinsics ir2_intrin;
    rs::extrinsics ir_to_color;
    rs::extrinsics color_to_ir;
    rs::extrinsics ir2_to_color;

    float scale;

    RgbdFrame();

    //project the 3d point to 2d color image
    cv::Point2f colorCameraProject(cv::Point3f p);

    cv::Point2f depthCameraProject(cv::Point3f p);

    cv::Point3f depthCameraToColorCamera(cv::Point3f p);

    cv::Point3f depthCameraDeproject(cv::Point2f p, float distance_in_meter);//undistort and deproject the point to 3d

    //undistort the depth point, should be applied after dividing by K matrix, not on pixel coordinated
    cv::Point2f undistortDepth(cv::Point2f p);
    cv::Point3f undistortDepth(cv::Point3f p);

    static cv::Mat getKMat(const rs::intrinsics intrin);

    static void extractDataFromKMat(cv::Mat K, rs::intrinsics& intrin);

    static cv::Mat getDistMat(const rs::intrinsics intrin);

    static void extractDataFromDistMat(cv::Mat dist, rs::intrinsics& intrin);

    static cv::Mat getRtMat(const rs::extrinsics extrin);

    static void extractDataFromRtMat(cv::Mat Rt, rs::extrinsics& extrin);

    static RgbdFrame extractDataFromRos(cv::Mat colorK, cv::Mat colorD, cv::Mat colorImg, cv::Mat depthK, cv::Mat depthD, cv::Mat depthImg, cv::Mat Extrinsic);

    void storeIntrinsic(cv::FileStorage& fs, std::string base, rs::intrinsics intrin);

    void readIntrinsic(cv::FileStorage& fs, std::string base, rs::intrinsics& intrin);

    void save(std::string folder, std::string name);

    bool load(std::string folder, std::string name);
};

RgbdFrame extractDataFromRs( rs::device *dev, bool useDepth = true, bool useIR = false, bool useIR2 = false);
void generatePointCloudOrganized2( RgbdFrame& frame, bool computeDepthAligned = true, bool computeNormal = true, bool computeNormalAligned = true);
RgbdFrame generatePointCloudOrganized2( rs::device *dev);

//stream = color:0, if useIR : ir1:1, ir2:2, else depth : 1
cv::Mat getCameraMatrix(const RgbdFrame& frame, int stream);

void sortBySerial(std::vector<RgbdFrame>& frames);

#ifdef USE_CERES_FOR_DISTORTIONS
//stream = color:0, if useIR : ir1:1, ir2:2, else depth : 1
cv::Mat getCameraDistMatrix(const RgbdFrame& frame, cv::Size size, int stream);
#endif


#endif // RGBDFRAME_H
