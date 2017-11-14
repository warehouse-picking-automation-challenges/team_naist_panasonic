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

#ifndef UTILCVPCLRS_H
#define UTILCVPCLRS_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <librealsense/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include "RGBDFrame.h"

std::string matType2str(cv::Mat mat);
std::string mat2str(cv::Mat mat, bool printMatContent = true);
bool isR200(rs::device *rsCamera);
bool isSR300(rs::device *rsCamera);
int printRSContextInfo( rs::context *c );
int configureRSStreams( rs::device *rsCamera, bool useDepth = true, bool useIR = false, bool useIR2 = false );

template<typename MatT, int MatRows, int MatCols>
cv::Point3f multMatVecT(cv::Mat mat, cv::Point3f p)
{
    cv::Point3f result;
    if(MatCols == 3)
    {
        MatT *line = mat.ptr<MatT>(0);
        result.x = line[0]*p.x+line[1]*p.y+line[2]*p.z;
        line = mat.ptr<MatT>(1);
        result.y = line[0]*p.x+line[1]*p.y+line[2]*p.z;
        line = mat.ptr<MatT>(2);
        result.z = line[0]*p.x+line[1]*p.y+line[2]*p.z;
    }
    else
    {
        MatT *line = mat.ptr<MatT>(0);
        result.x = line[0]*p.x+line[1]*p.y+line[2]*p.z+line[3];
        line = mat.ptr<MatT>(1);
        result.y = line[0]*p.x+line[1]*p.y+line[2]*p.z+line[3];
        line = mat.ptr<MatT>(2);
        result.z = line[0]*p.x+line[1]*p.y+line[2]*p.z+line[3];
    }
    if(MatRows == 3)
        return result;
    else
    {
        MatT *line = mat.ptr<MatT>(3);
        return result / (line[0]*p.x+line[1]*p.y+line[2]*p.z+line[3]);
    }
}

cv::Point3f multMatVec(cv::Mat mat, cv::Point3f p);
cv::Point2f multMatVec(cv::Mat mat, cv::Point2f p);
cv::Point2f min(cv::Point2f p1, cv::Point2f p2);
cv::Point2f max(cv::Point2f p1, cv::Point2f p2);
void BarycentricPrecompute(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f& v0, cv::Point2f& v1, float& d00, float& d01, float& d11, float& invDenom);

inline void BarycentricWithPrecomputedVal(cv::Point2f p, cv::Point2f a, cv::Point2f v0, cv::Point2f v1, float d00, float d01, float d11, float invDenom, float &u, float &v, float &w)
{
    cv::Point2f v2 = p - a;
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    v = (d11 * d20 - d01 * d21) * invDenom;
    w = (d00 * d21 - d01 * d20) * invDenom;
    u = 1.0f - v - w;
}

void Barycentric(cv::Point2f p, cv::Point2f a, cv::Point2f b, cv::Point2f c, float &u, float &v, float &w);
cv::Mat RTVec2Mat(cv::Mat rvec, cv::Mat tvec);
cv::Mat getRVecFromMat(cv::Mat M);
cv::Mat getTVecFromMat(cv::Mat M);
cv::Mat point2Mat(cv::Point3d p);

cv::Point3f getMinBoard(cv::Ptr<cv::aruco::CharucoBoard> board);
cv::Point3f getMaxBoard(cv::Ptr<cv::aruco::CharucoBoard> board);


void detectMarker(cv::Mat img, cv::Ptr<cv::aruco::Dictionary> dictionary, std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f> >& markerCorners, cv::Mat result);
bool detectCornersAndPose(const std::vector<int>& markerIds, const std::vector<std::vector<cv::Point2f> >& markerCorners, cv::Mat img, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat K, cv::Mat dist, std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds, cv::Mat& rvec, cv::Mat& tvec, cv::Mat result);
bool detectMarker(cv::Mat img, cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat K, cv::Mat dist, std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f> >& markerCorners, std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds, cv::Mat& rvec, cv::Mat& tvec, cv::Mat result = cv::Mat());
bool loadCalib(const char *filename, std::vector<std::string>& serialId, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses);
bool loadCalib(const char *filename, std::vector<std::string>& serialId, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses, std::vector<cv::Mat>& planeRt);
void saveCalib(const char *filename, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPose, const std::vector<cv::Mat>& planeRt = std::vector<cv::Mat>());
bool loadRecoSpaceConfig(const char *filename, cv::Point3f& recognitionCenter, cv::Point3f& recognitionSize, cv::Point3f& suctionCylinderCenter, cv::Point3f& suctionCylinderMainAxis, float& suctionCylinderHeight, float& suctionCylinderRadius);
void saveRecoSpaceConfig(const char *filename, cv::Point3f recognitionCenter, cv::Point3f recognitionSize, cv::Point3f suctionCylinderCenter, cv::Point3f suctionCylinderMainAxis, float suctionCylinderHeight, float suctionCylinderRadius);


std::vector<std::string> loadFramesIdSerial(const char *seq_folder, int nbCameras, int frameId, const char *prefix = "capture");
std::vector<RgbdFrame> loadFramesId(const char *seq_folder, int nbCameras, int frameId, const char *prefix = "capture");
std::vector<std::vector<RgbdFrame> > loadFrames(const char *seq_folder, int nbCameras, int nbFrames, int firstFrameId = 0, const char *prefix = "capture");

int getIdBySerial(const std::vector<std::string>& listSerial, std::string serial);

std::vector<cv::Point2f> project2d(cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& data);
void drawCylinder(cv::Mat result, cv::Mat KRGB, cv::Mat pose, cv::Point3f baseCenter, cv::Point3f top, float radius, cv::Scalar color, int nbHeightSegment = 10, int nbRadiusSegment = 20);
void drawBox(cv::Mat result, cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& box, cv::Scalar color);
void drawFilledBox(cv::Mat result, cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& box, cv::Scalar color);
bool pointInBox(cv::Point3f p, cv::Point3f center, cv::Point3f size);
bool pointInCylinder(cv::Point3f p, cv::Point3f cylinderCenter, cv::Point3f cylinderMainAxis, float height, float radius);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterStatisticalOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int meanK = 50, float stddevMulThresh = 1.0);


std::vector<cv::Point3f> getBoxPoints(cv::Point3f center, cv::Point3f size);

inline cv::Point2f rs2cv(rs::float2 a)
{
    return cv::Point2f(a.x, a.y);
}

inline cv::Point3f rs2cv(rs::float3 a)
{
    return cv::Point3f(a.x, a.y, a.z);
}

inline rs::float2 cv2rs(cv::Point2f a)
{
    return {a.x, a.y};
}

inline rs::float3 cv2rs(cv::Point3f a)
{
    return {a.x, a.y, a.z};
}

inline cv::Point3f pcl2cv(pcl::Normal n)
{
  return cv::Point3f(n.normal_x, n.normal_y, n.normal_z);
}

inline cv::Point3f pcl2cv(pcl::PointXYZ p)
{
  return cv::Point3f(p.x, p.y, p.z);
}

inline cv::Point3f pcl2cv(pcl::PointXYZRGB p)
{
  return cv::Point3f(p.x, p.y, p.z);
}

template<int nbDistVal, typename T>
void applyDistortion(const T* p, const T* const cameraDist, T* res, bool modified = false)
{
    if(nbDistVal == 0)
    {
        res[0] = p[0];
        res[1] = p[1];
    }
    else
    {
        T r2 = p[0]*p[0] + p[1]*p[1];
        T rad = T(1) + cameraDist[0] * r2;
        if(nbDistVal >= 2)
            rad += cameraDist[1] * r2 * r2;
        if(nbDistVal >= 5)
            rad += cameraDist[4] * r2 * r2 * r2;
        res[0] = p[0] * rad;
        res[1] = p[1] * rad;
        T x,y;
        if(modified)
        {
            x = res[0];
            y = res[1];
        }
        else
        {
            x = p[0];
            y = p[1];
        }
        if(nbDistVal >= 3)
        {
            res[0] += T(2)*cameraDist[2]*x*y;
            res[1] += cameraDist[2]*(r2+T(2)*y*y);
        }
        if(nbDistVal >= 4)
        {
            res[0] += cameraDist[3]*(r2+T(2)*x*x);
            res[1] += T(2)*cameraDist[3]*x*y;
        }
    }
}

template<int nbDistVal>
cv::Point2d applyDistortion(cv::Point2d p, cv::Mat cameraDist, bool modified = false)
{
    double vec[2] = {p.x, p.y};
    double res[2];
    applyDistortion<nbDistVal, double>(&vec[0], cameraDist.ptr<double>(0), &res[0], modified);
    return cv::Point2d(res[0], res[1]);
}

template<typename T>
std::vector<T> mat2vec(cv::Mat mat)
{
    std::vector<T> vec(mat.cols*mat.rows);
    for(int i = 0; i < mat.rows; i++)
    {
        T *ptr = mat.ptr<T>(i);
        T *dst = &vec[i*mat.cols];
        for(int j = 0; j < mat.cols; j++)
            dst[j] = ptr[j];
    }
    return vec;
}

template<typename T>
cv::Mat vec2mat(const std::vector<T>& vec, int cols, int rows, int type)
{
    cv::Mat mat(rows, cols, type);
    for(int i = 0; i < mat.rows; i++)
    {
        const T *ptr = &vec[i*mat.cols];
        T *dst = mat.ptr<T>(i);
        for(int j = 0; j < mat.cols; j++)
            dst[j] = ptr[j];
    }
    return mat;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::vector<cv::Point3f> computeAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented);
std::vector<cv::Point3f> computeAABB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented);
std::vector<cv::Point3f> computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, bool useConvexHull = true, float samplingDistance = CV_PI/180, float minTheta = 0.0, float maxTheta = CV_PI/2, float minPhi = 0, float maxPhi = 2*CV_PI);
std::vector<cv::Point3f> computeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented, bool useConvexHull = true, float samplingDistance = CV_PI/180, float minTheta = 0.0, float maxTheta = CV_PI/2, float minPhi = 0, float maxPhi = 2*CV_PI);
std::vector<cv::Point3f> estimateOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented);
std::vector<cv::Point3f> estimateBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented);

cv::Point3f extractCenterFromOBB(const std::vector<cv::Point3f>& OBB);
cv::Point3f extractSizeFromOBB(const std::vector<cv::Point3f>& OBB);

//return Rotation matrix R such that R * (OBB-center) = (AABB-center)
cv::Mat extractRotationFromOBB(const std::vector<cv::Point3f>& OBB);

#endif // CVPCLRSUTIL_H
