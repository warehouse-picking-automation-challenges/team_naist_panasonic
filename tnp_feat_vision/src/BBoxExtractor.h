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

#ifndef BBOXEXTRACTOR_H
#define BBOXEXTRACTOR_H

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/aruco.hpp>
#include "RGBDFrame.h"

#ifdef USE_GL
#include "GLRenderer.h"
#include "GLRendererDrawingPrimitive.h"
#endif

class BBoxExtractor
{
public:
    BBoxExtractor(bool drawResult = true);

    int getIdBySerial(std::string serial) const;

    void init(const std::vector<std::string>& cameraSerial, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, cv::Ptr<cv::aruco::Dictionary> qrDictionary, int minQrId, int maxQrId);
    void loadConfig(const char *filename);

    void updateBackgroundFrame(const std::vector<RgbdFrame>& listFrames);
    void updateBackgroundFrame(const RgbdFrame& frame);

    void compute(std::vector<RgbdFrame> listFrames);
    void processFrame(RgbdFrame frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB, float suctionCylinderPosX);
    bool testPointInRecognitionVolume(cv::Point3f p, float suctionCylinderPosX);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getConnectedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Point3f startPoint, float maxDistInit = 0.10, float maxDistCluster = 0.01);
    void getConnectedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr kdtree_cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, std::vector<int>& toEvaluate, std::vector<bool>& alreadyUsed, float maxDist);

    void drawSuctionCylinder(cv::Mat img, cv::Mat KRGB, cv::Mat pose);
    std::vector<cv::Point3f> getRecognitionBBOX();

    void handleKey(int key);

    cv::Mat generateView(const RgbdFrame &frame, const std::vector<cv::Point3f>& recoBBOX, const std::vector<cv::Point3f>& OBB, const std::vector<cv::Point3f>& AABB);



    std::vector<cv::Ptr<cv::BackgroundSubtractorMOG2> > backgroundSub;
    std::vector<bool> backgroundInit;
    std::vector<std::string> cameraSerial;
    std::vector<cv::Mat> cameraK;
    std::vector<cv::Mat> cameraDist;
    std::vector<cv::Mat> cameraPoses;
    cv::Ptr<cv::aruco::Dictionary> qrDictionary;
    int minQrId, maxQrId;

    cv::Point3f recognitionCenter, recognitionSize;
    cv::Point3f suctionCylinderCenter, suctionCylinderMainAxis, suctionCylinderRadiusAxis1, suctionCylinderRadiusAxis2;
    float suctionCylinderHeight, suctionCylinderRadius;
    int editMode;
    cv::Point3f recoMoveDir;
    bool drawResult;

    float realCylinderPosX;
    std::vector<cv::Point3f> _recoBBOX;
    std::vector<cv::Point3f> _OBB;
    std::vector<cv::Point3f> _AABB;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud;

    #ifdef USE_GL

    void drawSuctionCylinder(std::shared_ptr<GLMeshData> meshData);

    //rendering
    bool GLWin;
    std::shared_ptr<GLRenderer> renderer;
    std::shared_ptr<GLMeshData> meshData;
    std::shared_ptr<GLMesh> mesh;
    std::shared_ptr<GLMeshData> bboxMeshData;
    std::shared_ptr<GLMesh> bboxMesh;
    #endif
};

#endif // BBOXEXTRACTOR_H
