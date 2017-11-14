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

#include "UtilCvPclRs.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>

std::string matType2str(cv::Mat mat)
{
    char tmp[255];
    if(mat.type()==CV_32F)
        sprintf(tmp, "CV_32F");
    else if(mat.type()==CV_64F)
        sprintf(tmp, "CV_64F");
    else if(mat.type()==CV_8UC1)
        sprintf(tmp, "CV_8UC1");
    else if(mat.type()==CV_8UC2)
        sprintf(tmp, "CV_8UC2");
    else if(mat.type()==CV_8UC3)
        sprintf(tmp, "CV_8UC3");
    else if(mat.type()==CV_16UC1)
        sprintf(tmp, "CV_16UC1");
    else if(mat.type()==CV_32S)
        sprintf(tmp, "CV_32S");
    else sprintf(tmp, "unknown type (%d)", mat.type());
    return tmp;
}

std::string mat2str(cv::Mat mat, bool printMatContent)
{
    std::string str;
    char tmp[255];
    sprintf(tmp, "%dx%d :", mat.rows, mat.cols);
    str += tmp;
    str += matType2str(mat);
    str += "\n";
    if(printMatContent)
    {
        for(int i = 0; i < mat.rows; i++)
        {
            for(int j = 0; j < mat.cols; j++)
            {
                if(mat.type()==CV_32F)
                    sprintf(tmp, "%f", mat.at<float>(i, j));
                else if(mat.type()==CV_64F)
                    sprintf(tmp, "%lf", mat.at<double>(i, j));
                else if(mat.type()==CV_8UC1)
                    sprintf(tmp, "%d", mat.at<unsigned char>(i, j));
                else if(mat.type()==CV_8UC3)
                    sprintf(tmp, "(%d,%d,%d)", (int)mat.at<unsigned char>(i, j*3), (int)mat.at<unsigned char>(i, j*3+1), (int)mat.at<unsigned char>(i, j*3+2));
                else if(mat.type()==CV_16UC1)
                    sprintf(tmp, "%d", (int)mat.at<unsigned short>(i, j));
                else if(mat.type()==CV_32S)
                    sprintf(tmp, "%d", (int)mat.at<int>(i, j));
                str += tmp;
                if(j < mat.cols-1)
                    str += ", ";
                else str += "\n";
            }
        }
    }
    return str;
}


int printRSContextInfo( rs::context *c )
{
    printf( "There are %d connected RealSense devices.\n", c->get_device_count( ) );
    if( c->get_device_count( ) == 0 )
        throw std::runtime_error( "No device detected. Is it plugged in?" );

    return EXIT_SUCCESS;
}

bool isR200(rs::device *rsCamera)
{
    return std::string(rsCamera->get_name()).find("R200") != std::string::npos;
}

bool isSR300(rs::device *rsCamera)
{
    return std::string(rsCamera->get_name()).find("SR300") != std::string::npos;
}


int configureRSStreams( rs::device *rsCamera, bool useDepth, bool useIR, bool useIR2)
{
    std::cout << "Configuring RS streaming " << rsCamera->get_name( ) << "... \n";
    std::cout << "portId : " << rsCamera->get_usb_port_id() << " serial " << rsCamera->get_serial() << " ... \n";
    std::cout << "firmware : " << rsCamera->get_firmware_version() << "\n";
    std::cout << "isR200 : " << (isR200(rsCamera)?"true":"false") << "\n";
    std::cout << "useDepth : " << (useDepth?"true":"false") << " useIR " << (useIR?"true":"false") << " useIR2 " << (useIR2?"true":"false") << "\n";

    if(useDepth)
    {
        if(isR200(rsCamera))
            rsCamera->enable_stream(rs::stream::depth, 480, 360, rs::format::z16, 30);
        else rsCamera->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 10);
    }
    else rsCamera->disable_stream(rs::stream::depth);

    if(useIR)
        rsCamera->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
    else rsCamera->disable_stream(rs::stream::infrared);

    if(isR200(rsCamera))
    {
        if(useIR2)
            rsCamera->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 30);
        else rsCamera->disable_stream(rs::stream::infrared2);
    }
    if(isR200(rsCamera))
        rsCamera->enable_stream(rs::stream::color, 1280, 720, rs::format::rgb8, 15);
    else rsCamera->enable_stream(rs::stream::color, 1280, 720, rs::format::rgb8, 10);//rsCamera->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 10);

    std::cout << "Starting... \n";

    rsCamera->start( );

    std::cout << "RS streaming enabled and running.\n";

    return EXIT_SUCCESS;
}

cv::Point3f multMatVec(cv::Mat mat, cv::Point3f p)
{
    cv::Point3f p2;
    cv::Mat vec(mat.cols, 1, mat.type());

    if(vec.type() == CV_32F)
    {
        vec.at<float>(0,0) = p.x; vec.at<float>(1,0) = p.y; vec.at<float>(2,0) = p.z;
        if(mat.cols == 4)
            vec.at<float>(3,0) = 1.0;
    }
    else
    {
        vec.at<double>(0,0) = p.x; vec.at<double>(1,0) = p.y; vec.at<double>(2,0) = p.z;
        if(mat.cols == 4)
            vec.at<double>(3,0) = 1.0;
    }

    cv::Mat res = mat*vec;
    if(res.rows == 4)
    {
        if(res.type() == CV_32F)
            res = res / res.at<float>(3,0);
        else res = res / res.at<double>(3,0);
    }

    if(res.type() == CV_32F)
    {
        p2.x = res.at<float>(0,0); p2.y = res.at<float>(1,0); p2.z = res.at<float>(2,0);
    }
    else
    {
        p2.x = res.at<double>(0,0); p2.y = res.at<double>(1,0); p2.z = res.at<double>(2,0);
    }
    return p2;
}

cv::Point2f multMatVec(cv::Mat mat, cv::Point2f p)
{
    cv::Point2f p2;
    cv::Mat vec(mat.cols, 1, mat.type());

    if(vec.type() == CV_32F)
    {
        vec.at<float>(0,0) = p.x; vec.at<float>(1,0) = p.y;
        if(mat.cols == 3)
            vec.at<float>(2,0) = 1.0;
    }
    else
    {
        vec.at<double>(0,0) = p.x; vec.at<double>(1,0) = p.y;
        if(mat.cols == 3)
            vec.at<double>(2,0) = 1.0;
    }

    cv::Mat res = mat*vec;
    if(res.rows == 3)
    {
        if(res.type() == CV_32F)
            res = res / res.at<float>(2,0);
        else res = res / res.at<double>(2,0);
    }

    if(res.type() == CV_32F)
    {
        p2.x = res.at<float>(0,0); p2.y = res.at<float>(1,0);
    }
    else
    {
        p2.x = res.at<double>(0,0); p2.y = res.at<double>(1,0);
    }
    return p2;
}

cv::Point2f min(cv::Point2f p1, cv::Point2f p2)
{
    return cv::Point2f(p1.x<p2.x?p1.x:p2.x, p1.y<p2.y?p1.y:p2.y);
}

cv::Point2f max(cv::Point2f p1, cv::Point2f p2)
{
    return cv::Point2f(p1.x>p2.x?p1.x:p2.x, p1.y>p2.y?p1.y:p2.y);
}

void BarycentricPrecompute(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f& v0, cv::Point2f& v1, float& d00, float& d01, float& d11, float& invDenom)
{
    v0 = b - a;
    v1 = c - a;
    d00 = v0.dot(v0);
    d01 = v0.dot(v1);
    d11 = v1.dot(v1);
    invDenom = 1.0/(d00 * d11 - d01 * d01);
}

void Barycentric(cv::Point2f p, cv::Point2f a, cv::Point2f b, cv::Point2f c, float &u, float &v, float &w)
{
    cv::Point2f v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float invDenom = 1.0/(d00 * d11 - d01 * d01);
    v = (d11 * d20 - d01 * d21) * invDenom;
    w = (d00 * d21 - d01 * d20) * invDenom;
    u = 1.0f - v - w;
}

cv::Mat RTVec2Mat(cv::Mat rvec, cv::Mat tvec)
{
    if(rvec.empty() || tvec.empty())
    {
        printf("error RTVec2Mat empty mat\n");
        return cv::Mat::eye(4,4,CV_64F);
    }
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat Rmat = cv::Mat::eye(4,4,CV_64F);
    cv::Mat Tmat = cv::Mat::eye(4,4,CV_64F);
    R.copyTo(Rmat(cv::Rect(0,0,3,3)));
    tvec.copyTo(Tmat(cv::Rect(3,0,1,3)));

    return Tmat*Rmat;
}

cv::Mat getRVecFromMat(cv::Mat M)
{
    cv::Mat rvec;
    cv::Rodrigues(M(cv::Rect(0,0,3,3)), rvec);
    return rvec;
}

cv::Mat getTVecFromMat(cv::Mat M)
{
    return M(cv::Rect(3,0,1,3)).clone();
}

cv::Mat point2Mat(cv::Point3d p)
{
    cv::Mat m(3,1,CV_64F);
    m.at<double>(0, 0) = p.x;
    m.at<double>(1, 0) = p.y;
    m.at<double>(2, 0) = p.z;
    return m;
}

void detectMarker(cv::Mat img, cv::Ptr<cv::aruco::Dictionary> dictionary, std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f> >& markerCorners, cv::Mat result)
{
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds);
    if(!result.empty())
    {
        for(int x = 0; x < markerCorners.size(); x++)
            for(int y = 0; y < markerCorners[x].size(); y++)
                cv::circle(result, markerCorners[x][y],  3, cv::Scalar(0,255,0), 1);
    }
}

bool detectCornersAndPose(const std::vector<int>& markerIds, const std::vector<std::vector<cv::Point2f> >& markerCorners, cv::Mat img, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat K, cv::Mat dist, std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds, cv::Mat& rvec, cv::Mat& tvec, cv::Mat result)
{
    if(markerIds.size() > 0)
    {
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds, K, dist);

        for(int l = 0; l < charucoCorners.size(); l++)
            if(!result.empty())
            cv::circle(result, charucoCorners[l], 4, cv::Scalar(255,0,0), 2);

        cv::Mat rvec2, tvec2;
        bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, K, dist, rvec2, tvec2);
        rvec = rvec2;
        tvec = tvec2;
        if(valid)
        {
            if(!result.empty())
                cv::aruco::drawAxis(result, K, dist, rvec, tvec, 0.1);
            return true;
        }
    }
    return false;
}

bool detectMarker(cv::Mat img, cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, cv::Mat K, cv::Mat dist, std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f> >& markerCorners, std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds, cv::Mat& rvec, cv::Mat& tvec, cv::Mat result)
{
    detectMarker(img, dictionary, markerIds, markerCorners, result);
    return detectCornersAndPose(markerIds, markerCorners, img, board, K, dist, charucoCorners, charucoIds, rvec, tvec, result);
}

cv::Point3f getMinBoard(cv::Ptr<cv::aruco::CharucoBoard> board)
{
    cv::Point3f minBoard = board->chessboardCorners[0];
    for(int k = 0; k < board->chessboardCorners.size(); k++)
    {
        minBoard.x = std::min(minBoard.x, board->chessboardCorners[k].x);
        minBoard.y = std::min(minBoard.y, board->chessboardCorners[k].y);
    }
    minBoard.x -= board->getSquareLength();
    minBoard.y -= board->getSquareLength();
    return minBoard;
}

cv::Point3f getMaxBoard(cv::Ptr<cv::aruco::CharucoBoard> board)
{
    cv::Point3f maxBoard = board->chessboardCorners[0];
    for(int k = 0; k < board->chessboardCorners.size(); k++)
    {
        maxBoard.x = std::max(maxBoard.x, board->chessboardCorners[k].x);
        maxBoard.y = std::max(maxBoard.y, board->chessboardCorners[k].y);
    }
    maxBoard.x += board->getSquareLength();
    maxBoard.y += board->getSquareLength();
    return maxBoard;
}

template<typename T>
pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudXYZ(T cloudPtr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloudPtr, *cloud_xyz);
    return cloud_xyz;
}

std::vector<cv::Point3f> computeAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented)
{
    std::vector<cv::Point3f> AABB;
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudSegmented, minPoint, maxPoint);
    cv::Point3f min = pcl2cv(minPoint);
    cv::Point3f max = pcl2cv(maxPoint);
    AABB.push_back(cv::Point3f(min.x, min.y, min.z));
    AABB.push_back(cv::Point3f(max.x, min.y, min.z));
    AABB.push_back(cv::Point3f(min.x, max.y, min.z));
    AABB.push_back(cv::Point3f(max.x, max.y, min.z));
    AABB.push_back(cv::Point3f(min.x, min.y, max.z));
    AABB.push_back(cv::Point3f(max.x, min.y, max.z));
    AABB.push_back(cv::Point3f(min.x, max.y, max.z));
    AABB.push_back(cv::Point3f(max.x, max.y, max.z));

    return AABB;
}

std::vector<cv::Point3f> computeAABB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented)
{
    return computeAABB(toCloudXYZ(cloudSegmented));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull);
    return cloud_hull;
}

cv::Point3f computeOrthogonalVector(cv::Point3f vec)
{
    cv::Point3f vec2 = vec.cross(cv::Point3f(1,0,0));
    float norm2 = vec2.dot(vec2);
    if(norm2 < 0.001)
    {
        vec2 = vec.cross(cv::Point3f(0,1,0));
        norm2 = vec2.dot(vec2);
    }
    return vec2 / sqrt(norm2);
}

std::vector<cv::Point3f> computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, bool useConvexHull, float samplingDistance, float minTheta, float maxTheta, float minPhi, float maxPhi)
{
    int t0 = clock();
    if(useConvexHull)
        cloudSegmented = computeConvexHull(cloudSegmented);

    int nbSampleTheta = (int)cvCeil((maxTheta-minTheta)/samplingDistance);
    int nbSamplePhi = (int)cvCeil((maxPhi-minPhi)/samplingDistance);
    int nbSubSample = (int)cvCeil(CV_PI/samplingDistance);
    std::vector<cv::Point3f> cloud(cloudSegmented->size());
    for(int i = 0; i < cloud.size(); i++)
        cloud[i] = pcl2cv(cloudSegmented->points[i]);

    float bestVolume;
    cv::Point3f bestVec1;
    cv::Point3f bestVec2;
    cv::Point3f bestVec3;
    for(int i = 0; i < nbSampleTheta; i++)
    {
        float sinTheta = sin(minTheta + (maxTheta-minTheta)*i/(nbSampleTheta-1));
        float cosTheta = cos(minTheta + (maxTheta-minTheta)*i/(nbSampleTheta-1));
        int nbSamplePhi2 = std::max(2, (int)(sinTheta*nbSamplePhi));
        //printf("nb sample phi %d\n", nbSamplePhi2);
        for(int j = 0; j < nbSamplePhi2; j++)
        {
            float sinPhi = sin(minPhi + (maxPhi-minPhi)*j/(nbSamplePhi2-1));
            float cosPhi = cos(minPhi + (maxPhi-minPhi)*j/(nbSamplePhi2-1));
            cv::Point3f vec1(sinTheta*cosPhi, sinTheta*sinPhi, cosTheta);//(1,0,0)
            //printf("vec1 %f %f %f\n", vec1.x, vec1.y, vec1.z);
            cv::Point3f vec2 = computeOrthogonalVector(vec1);
            cv::Point3f vec3 = vec1.cross(vec2);

            std::vector<cv::Point2f> projectedCloud(cloud.size());
            float minX, maxX;
            for(int k = 0; k < cloud.size(); k++)
            {
                cv::Point3f p = cloud[k];
                projectedCloud[k] = cv::Point2f(p.dot(vec2), p.dot(vec3));
                float x = p.dot(vec1);
                if(k == 0)
                {
                    minX = x;
                    maxX = x;
                }
                else
                {
                    if(x < minX)
                        minX = x;
                    else if(x > maxX)
                        maxX = x;
                }
            }

            std::vector<cv::Point2f> convexHullCloud;
            if(projectedCloud.size() == 0)
                continue;
            cv::convexHull(projectedCloud, convexHullCloud, false);

            for(int k = 0; k < nbSubSample; k++)
            {
                float angle = CV_PI*k/(nbSubSample-1);
                cv::Point2f dir(cos(angle), sin(angle));
                cv::Point2f dir2(-dir.y, dir.x);

                float minY, maxY, minZ, maxZ;
                for(int l = 0; l < convexHullCloud.size(); l++)
                {
                    cv::Point2f p = convexHullCloud[l];
                    float y = p.dot(dir);
                    float z = p.dot(dir2);
                    if(l == 0)
                    {
                        minY = y;
                        maxY = y;
                        minZ = z;
                        maxZ = z;
                    }
                    else
                    {
                        if(y < minY)
                            minY = y;
                        else if(y > maxY)
                            maxY = y;

                        if(z < minZ)
                            minZ = z;
                        else if(z > maxZ)
                            maxZ = z;
                    }
                }

                float volume = (maxX-minX)*(maxY-minY)*(maxZ-minZ);
                if((i == 0 && j == 0 && k == 0) || volume < bestVolume)
                {
                    bestVolume = volume;
                    bestVec1 = vec1;
                    bestVec2 = dir.x*vec2+dir.y*vec3;
                    bestVec3 = bestVec1.cross(bestVec2);
                }
            }
       }
    }

    printf("%d clock\n", clock()-t0);//92321, 986820
    //hull 3012964

    printf("bestVolume : %f\n", bestVolume);

    float minX, maxX, minY, maxY, minZ, maxZ;
    for(int i = 0; i < cloud.size(); i++)
    {
        cv::Point3f p = cloud[i];
        cv::Point3f p2(p.dot(bestVec1), p.dot(bestVec2), p.dot(bestVec3));
        if(i == 0)
        {
            minX = p2.x;
            maxX = p2.x;
            minY = p2.y;
            maxY = p2.y;
            minZ = p2.z;
            maxZ = p2.z;
        }
        else
        {
            if(p2.x < minX)
                minX = p2.x;
            else if(p2.x > maxX)
                maxX = p2.x;

            if(p2.y < minY)
                minY = p2.y;
            else if(p2.y > maxY)
                maxY = p2.y;

            if(p2.z < minZ)
                minZ = p2.z;
            else if(p2.z > maxZ)
                maxZ = p2.z;
        }
    }
    float volume = (maxX-minX)*(maxY-minY)*(maxZ-minZ);
    printf("volume %f\n", volume);
    std::vector<cv::Point3f> OOBB;
    for(int i = 0; i < 8; i++)
    {
        cv::Point3f p(i%2==0?minX:maxX, (i%4)/2==0?minY:maxY, i/4==0?minZ:maxZ);
        OOBB.push_back(p.x*bestVec1+p.y*bestVec2+p.z*bestVec3);
    }
    return OOBB;
}

std::vector<cv::Point3f> computeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented, bool useConvexHull, float samplingDistance, float minTheta, float maxTheta, float minPhi, float maxPhi)
{
    return computeOBB(toCloudXYZ(cloudSegmented), useConvexHull, samplingDistance, minTheta, maxTheta, minPhi, maxPhi);
}

std::vector<cv::Point3f> estimateOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented)
{
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    std::vector<cv::Point3f> OOBB;
    cv::Point3f size(maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);
    for(int i = 0; i < 8; i++)
    {
        Eigen::Vector3f p(i%2==0?-size.x:size.x, (i%4)/2==0?-size.y:size.y, i/4==0?-size.z:size.z);
        p /= 2.0;
        Eigen::Vector3f result = bboxQuaternion*p+bboxTransform;
        OOBB.push_back(cv::Point3f(result[0], result[1], result[2]));
    }
    // This viewer has 4 windows, but is only showing images in one of them as written here.
   /* pcl::visualization::PCLVisualizer *visu;
    visu = new pcl::visualization::PCLVisualizer ("PlyViewer");
   *int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
    visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
    visu->addPointCloud(cloudSegmented);
    visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
    visu->spin();*/
    return OOBB;
}

std::vector<cv::Point3f> estimateOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented)
{
    return estimateOBB(toCloudXYZ(cloudSegmented));
}

cv::Point3f extractCenterFromOBB(const std::vector<cv::Point3f>& OBB)
{
    cv::Point3f center(0,0,0);
    for(int i = 0; i < OBB.size(); i++)
        center += OBB[i];
    return center/(float)OBB.size();
}

cv::Point3f extractSizeFromOBB(const std::vector<cv::Point3f>& OBB)
{
    float dist[3];
    for(int i = 0; i < 3; i++)
    {
        cv::Point3f dir = OBB[1<<i] - OBB[0];
        dist[i] = sqrt(dir.dot(dir));
    }
    return cv::Point3f(dist[0], dist[1], dist[2]);
}

cv::Mat extractRotationFromOBB(const std::vector<cv::Point3f>& OBB)
{
    std::vector<cv::Point3f> OBB2 = OBB;
    std::vector<cv::Point3f> model(8);
    cv::Point3f center = extractCenterFromOBB(OBB);
    cv::Point3f size = extractSizeFromOBB(OBB);
    for(int i = 0; i < 8; i++)
    {
        OBB2[i] -= center;
        model[i] = 0.5*cv::Point3f(i%2==0?-size.x:size.x, (i%4)/2==0?-size.y:size.y, i/4==0?-size.z:size.z);
        printf("%f %f %f -> %f %f %f\n", OBB2[i].x, OBB2[i].y, OBB2[i].z, model[i].x, model[i].y, model[i].z);
    }
    //x y z 0 0 0 0 0 0   *  X   =    x2 0 0
    //0 0 0 x y z 0 0 0               0  y2
    //0 0 0 0 0 0 x y z               0  0  z2
    cv::Mat src1 = cv::Mat::zeros(8*3,9,CV_64F);
    cv::Mat src2 = cv::Mat::zeros(8*3,1,CV_64F);
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            src1.at<double>(i*3+j, j*3) = OBB2[i].x;
            src1.at<double>(i*3+j, j*3+1) = OBB2[i].y;
            src1.at<double>(i*3+j, j*3+2) = OBB2[i].z;
        }
        src2.at<double>(i*3, 0) = model[i].x;
        src2.at<double>(i*3+1, 0) = model[i].y;
        src2.at<double>(i*3+2, 0) = model[i].z;
    }
    cv::Mat res;
    cv::solve(src1, src2, res, cv::DECOMP_SVD);
    cv::Mat mat(3,3,CV_64F);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        {
            mat.at<double>(i,j) = res.at<double>(i*3+j,0);
        }

    printf("R = %s\n", mat2str(mat).c_str());
    mat = mat(cv::Rect(0,0,3,3)).clone();
    mat.convertTo(mat, CV_64F);
    return mat;
}

bool loadCalib(const char *filename, std::vector<std::string>& serialId, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses, std::vector<cv::Mat>& planeRt)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
      printf("calib : can't load %s\n", filename);
      return false;
    }
    cv::FileNode serialNode = fs["serial"];
    serialId.resize(serialNode.size());
    for(int i = 0; i < serialNode.size(); i++)
    {
        serialNode[i] >> serialId[i];
        printf("%s\n\n", serialId[i].c_str());
    }
    cv::FileNode cameraKNode = fs["K"];
    cameraK.resize(cameraKNode.size());
    for(int i = 0; i < cameraKNode.size(); i++)
    {
        cameraKNode[i] >> cameraK[i];
        printf("%s\n\n", mat2str(cameraK[i]).c_str());
    }
    cv::FileNode cameraDistNode = fs["dist"];
    cameraDist.resize(cameraDistNode.size());
    for(int i = 0; i < cameraDistNode.size(); i++)
    {
        cameraDistNode[i] >> cameraDist[i];
        printf("%s\n\n", mat2str(cameraDist[i]).c_str());
    }
    cv::FileNode posesNode = fs["poses"];
    poses.resize(posesNode.size());
    for(int i = 0; i < posesNode.size(); i++)
    {
        posesNode[i] >> poses[i];
        printf("%s\n\n", mat2str(poses[i]).c_str());
    }
    cv::FileNode planeRtNode = fs["planeRt"];
    if(!planeRtNode.empty())
    {
        planeRt.resize(planeRtNode.size());
        for(int i = 0; i < planeRt.size(); i++)
        {
            planeRtNode[i] >> planeRt[i];
            printf("%s\n\n", mat2str(planeRt[i]).c_str());
        }
    }
    fs.release();
    return true;
}

bool loadCalib(const char *filename, std::vector<std::string>& serialId, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses)
{
    std::vector<cv::Mat> planeRt;
    return loadCalib(filename, serialId, cameraK, cameraDist, poses, planeRt);
}

void saveCalib(const char *filename, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPose, const std::vector<cv::Mat>& planeRt)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "serial" << "[";
    for(int i = 0; i < serialId.size(); i++)
        fs << serialId[i];
    fs << "]";
    fs << "K" << "[";
    for(int i = 0; i < cameraK.size(); i++)
        fs << cameraK[i];
    fs << "]";
    fs << "dist" << "[";
    for(int i = 0; i < cameraDist.size(); i++)
        fs << cameraDist[i];
    fs << "]";
    fs << "poses" << "[";
    for(int i = 0; i < cameraPose.size(); i++)
        fs << cameraPose[i];
    fs << "]";
    if(planeRt.size() > 0)
    {
        fs << "planeRt" << "[";
        for(int i = 0; i < planeRt.size(); i++)
            fs << planeRt[i];
        fs << "]";
    }
    fs.release();
}

bool loadRecoSpaceConfig(const char *filename, cv::Point3f& recognitionCenter, cv::Point3f& recognitionSize, cv::Point3f& suctionCylinderCenter, cv::Point3f& suctionCylinderMainAxis, float& suctionCylinderHeight, float& suctionCylinderRadius)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
      printf("RecoSpaceConfig : can't load %s\n", filename);
      return false;
    }
    fs["recognitionCenter"] >> recognitionCenter;
    fs["recognitionSize"] >> recognitionSize;
    fs["suctionCylinderCenter"] >> suctionCylinderCenter;
    fs["suctionCylinderMainAxis"] >> suctionCylinderMainAxis;
    fs["suctionCylinderHeight"] >> suctionCylinderHeight;
    fs["suctionCylinderRadius"] >> suctionCylinderRadius;
    fs.release();
}

void saveRecoSpaceConfig(const char *filename, cv::Point3f recognitionCenter, cv::Point3f recognitionSize, cv::Point3f suctionCylinderCenter, cv::Point3f suctionCylinderMainAxis, float suctionCylinderHeight, float suctionCylinderRadius)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened())
    {
      printf("RecoSpaceConfig : can't save %s\n", filename);
      return ;
    }
    fs << "recognitionCenter" << recognitionCenter;
    fs << "recognitionSize" << recognitionSize;
    fs << "suctionCylinderCenter" << suctionCylinderCenter;
    fs << "suctionCylinderMainAxis" << suctionCylinderMainAxis;
    fs << "suctionCylinderHeight" << suctionCylinderHeight;
    fs << "suctionCylinderRadius" << suctionCylinderRadius;
    fs.release();
}


std::vector<std::string> loadFramesIdSerial(const char *seq_folder, int nbCameras, int frameId, const char *prefix)
{
    std::vector<std::string> list;
    for(int j = 0; j < nbCameras; j++)
    {
        RgbdFrame frame;
        char name[255];
        sprintf(name, "%s%d_cam%d", prefix, frameId, j);
        if(frame.load(seq_folder, name))
            list.push_back(frame.camSerial);
    }
    return list;
}

int getIdBySerial(const std::vector<std::string>& listSerial, std::string serial)
{
    for(int i = 0; i < listSerial.size(); i++)
        if(listSerial[i] == serial)
            return i;
    return -1;
}

std::vector<RgbdFrame> loadFramesId(const char *seq_folder, int nbCameras, int frameId, const char *prefix)
{
    std::vector<RgbdFrame> list;
    for(int j = 0; j < nbCameras; j++)
    {
        RgbdFrame frame;
        char name[255];
        sprintf(name, "%s%d_cam%d", prefix, frameId, j);
        if(frame.load(seq_folder, name))
            list.push_back(frame);
    }
    return list;
}

std::vector<std::vector<RgbdFrame> > loadFrames(const char *seq_folder, int nbCameras, int nbFrames, int firstFrame, const char *prefix)
{
    std::vector<std::vector<RgbdFrame> > listFrames;
    for(int i = 0; i < nbFrames; i++)
        listFrames.push_back(loadFramesId(seq_folder, nbCameras, i+firstFrame, prefix));
    return listFrames;
}

std::vector<cv::Point2f> project2d(cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& data)
{
    std::vector<cv::Point2f> result;
    for(int j = 0; j < data.size(); j++)
    {
        cv::Point3f p = multMatVec(pose, data[j]);
        cv::Point3f uv = multMatVec(KRGB, p/p.z);
        result.push_back(cv::Point2f(uv.x, uv.y));
    }
    return result;
}

void drawCylinder(cv::Mat result, cv::Mat KRGB, cv::Mat pose, cv::Point3f baseCenter, cv::Point3f top, float radius, cv::Scalar color, int nbHeightSegment, int nbRadiusSegment)
{
    cv::Point3f mainAxis = top-baseCenter;
    float height = sqrt(mainAxis.dot(mainAxis));
    mainAxis /= height;
    cv::Point3f radiusAxis1 = mainAxis.cross(cv::Point3f(1,0,0));
    if(radiusAxis1.dot(radiusAxis1) < 0.01)
        radiusAxis1 = mainAxis.cross(cv::Point3f(0,1,0));
    radiusAxis1 /= sqrt(radiusAxis1.dot(radiusAxis1));
    cv::Point3f radiusAxis2 = mainAxis.cross(radiusAxis1);
    radiusAxis2 /= sqrt(radiusAxis2.dot(radiusAxis2));

    for(int j = 0; j < nbHeightSegment; j++)
    {
        for(int k = 0; k < nbRadiusSegment; k++)
        {
            int k2 = (k+1)%nbRadiusSegment;
            cv::Point3f p = baseCenter + height*(float(j)/(nbHeightSegment-1))*mainAxis;
            cv::Point3f p1 = p + radius*(cos(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
            cv::Point3f p2 = p + radius*(cos(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
            cv::Point3f p3 = p1+height*mainAxis/(nbHeightSegment-1);

            p1 = multMatVec(pose, p1);
            p2 = multMatVec(pose, p2);
            p3 = multMatVec(pose, p3);

            cv::Point3f uv1 = multMatVec(KRGB, p1/p1.z);
            cv::Point3f uv2 = multMatVec(KRGB, p2/p2.z);
            cv::Point3f uv3 = multMatVec(KRGB, p3/p3.z);
            cv::line(result, cv::Point(uv1.x, uv1.y), cv::Point(uv2.x, uv2.y), color);
            cv::line(result, cv::Point(uv1.x, uv1.y), cv::Point(uv3.x, uv3.y), color);
        }
    }
}

void drawFilledCylinder(cv::Mat result, cv::Mat KRGB, cv::Mat pose, cv::Point3f baseCenter, cv::Point3f top, float radius, cv::Scalar color, int nbHeightSegment, int nbRadiusSegment)
{
    cv::Point3f mainAxis = top-baseCenter;
    float height = sqrt(mainAxis.dot(mainAxis));
    mainAxis /= height;
    cv::Point3f radiusAxis1 = mainAxis.cross(cv::Point3f(1,0,0));
    if(radiusAxis1.dot(radiusAxis1) < 0.01)
        radiusAxis1 = mainAxis.cross(cv::Point3f(0,1,0));
    radiusAxis1 /= sqrt(radiusAxis1.dot(radiusAxis1));
    cv::Point3f radiusAxis2 = mainAxis.cross(radiusAxis1);
    radiusAxis2 /= sqrt(radiusAxis2.dot(radiusAxis2));

    for(int j = 0; j < nbHeightSegment; j++)
    {
        for(int k = 0; k < nbRadiusSegment; k++)
        {
            int k2 = (k+1)%nbRadiusSegment;
            cv::Point3f p = baseCenter + height*(float(j)/(nbHeightSegment-1))*mainAxis;
            cv::Point3f p1 = p + radius*(cos(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
            cv::Point3f p2 = p + radius*(cos(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
            cv::Point3f p3 = p1+height*mainAxis/(nbHeightSegment-1);
            cv::Point3f p4 = p2+height*mainAxis/(nbHeightSegment-1);

            p1 = multMatVec(pose, p1);
            p2 = multMatVec(pose, p2);
            p3 = multMatVec(pose, p3);
            p4 = multMatVec(pose, p4);

            cv::Point3f uv1 = multMatVec(KRGB, p1/p1.z);
            cv::Point3f uv2 = multMatVec(KRGB, p2/p2.z);
            cv::Point3f uv3 = multMatVec(KRGB, p3/p3.z);
            cv::Point3f uv4 = multMatVec(KRGB, p4/p4.z);
            cv::Point points[4] = {cv::Point2f(uv1.x, uv1.y), cv::Point2f(uv2.x, uv2.y), cv::Point2f(uv4.x, uv4.y), cv::Point2f(uv3.x, uv3.y)};
            cv::fillConvexPoly(result, points, 4, color);
        }
    }
}

void drawBox(cv::Mat result, cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& box, cv::Scalar color)
{
    if(box.size() != 8)
        return;
    std::vector<std::pair<int, int> > listLines;
    listLines.push_back(std::make_pair(0,1)); listLines.push_back(std::make_pair(1,3)); listLines.push_back(std::make_pair(3,2)); listLines.push_back(std::make_pair(2,0));
    listLines.push_back(std::make_pair(4,5)); listLines.push_back(std::make_pair(5,7)); listLines.push_back(std::make_pair(7,6)); listLines.push_back(std::make_pair(6,4));
    listLines.push_back(std::make_pair(0,4)); listLines.push_back(std::make_pair(1,5)); listLines.push_back(std::make_pair(2,6)); listLines.push_back(std::make_pair(3,7));
    std::vector<cv::Point2f> box2d = project2d(KRGB, pose, box);
    for(int k = 0; k < listLines.size(); k++)
        cv::line(result, box2d[listLines[k].first], box2d[listLines[k].second], color, 2);
}

void drawFilledBox(cv::Mat result, cv::Mat KRGB, cv::Mat pose, const std::vector<cv::Point3f>& box, cv::Scalar color)
{
    if(box.size() != 8)
        return;
    std::vector<std::vector<int> > listQuads;
    listQuads.push_back({0,1,3,2});
    listQuads.push_back({4,5,7,6});
    listQuads.push_back({0,1,5,4});
    listQuads.push_back({1,3,7,5});
    listQuads.push_back({3,2,6,7});
    listQuads.push_back({2,0,4,6});
    std::vector<cv::Point2f> box2d = project2d(KRGB, pose, box);
    for(int i = 0; i < listQuads.size(); i++)
    {
        cv::Point p[4];
        for(int j = 0; j < 4; j++)
            p[j] = box2d[listQuads[i][j]];
        cv::fillConvexPoly(result, p, 4, color);
    }
}

std::vector<cv::Point3f> getBoxPoints(cv::Point3f center, cv::Point3f size)
{
    std::vector<cv::Point3f> recoBBOX;
    recoBBOX.push_back(cv::Point3f(-size.x/2, -size.y/2, -size.z/2));
    recoBBOX.push_back(cv::Point3f(size.x/2, -size.y/2, -size.z/2));
    recoBBOX.push_back(cv::Point3f(-size.x/2, size.y/2, -size.z/2));
    recoBBOX.push_back(cv::Point3f(size.x/2, size.y/2, -size.z/2));
    recoBBOX.push_back(cv::Point3f(-size.x/2, -size.y/2, size.z/2));
    recoBBOX.push_back(cv::Point3f(size.x/2, -size.y/2, size.z/2));
    recoBBOX.push_back(cv::Point3f(-size.x/2, size.y/2, size.z/2));
    recoBBOX.push_back(cv::Point3f(size.x/2, size.y/2, size.z/2));
    for(int j = 0; j < recoBBOX.size(); j++)
        recoBBOX[j] += center;
    return recoBBOX;
}

bool pointInBox(cv::Point3f p, cv::Point3f center, cv::Point3f size)
{
    return (p.x >= center.x-size.x/2 && p.x <= center.x+size.x/2
            && p.y >= center.y-size.y/2 && p.y <= center.y+size.y/2
            && p.z >= center.z-size.z/2 && p.z <= center.z+size.z/2);
}

bool pointInCylinder(cv::Point3f p, cv::Point3f cylinderCenter, cv::Point3f cylinderMainAxis, float height, float radius)
{
    cv::Point3f cylinderP = p - cylinderCenter;
    double a = cylinderP.dot(cylinderMainAxis);
    cv::Point3f cylinderProj = cylinderCenter + a*cylinderMainAxis;
    cv::Point3f cylinderDir = p-cylinderProj;
    return (a >= -height/2 && a <= height/2 && sqrt(cylinderDir.dot(cylinderDir) <= radius));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterStatisticalOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int meanK, float stddevMulThresh)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}

cv::Ptr<cv::aruco::CharucoBoard> createCustomCharucoBoard(int squaresX, int squaresY, float squareLength,float markerLength, const cv::Ptr<cv::aruco::Dictionary> &dictionary, int firstId)
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    for(int i = 0; i < board->ids.size(); i++)
        board->ids[i] += firstId;
    return board;
}

cv::Mat addBorder(cv::Mat img, float ratio)
{
    cv::Mat img2 = cv::Mat::zeros((int)ceil(img.rows/(1.0-ratio)), (int)ceil(img.cols/(1.0-ratio)), img.type());
    img.copyTo(img2(cv::Rect((img2.cols-img.cols)/2,(img2.rows-img.rows)/2,img.cols, img.rows)));
    return img2;
}


void createArucoData(float squareLength, float markerLength, cv::Ptr<cv::aruco::Dictionary>& dictionary, cv::Ptr<cv::aruco::CharucoBoard>& boardTop, cv::Ptr<cv::aruco::CharucoBoard>& boardSmallSide1,
                     cv::Ptr<cv::aruco::CharucoBoard>& boardSmallSide2, cv::Ptr<cv::aruco::CharucoBoard>& boardLongSide1, cv::Ptr<cv::aruco::CharucoBoard>& boardLongSide2, cv::Ptr<cv::aruco::CharucoBoard>& additionalBoard, bool saveToDisk)
{
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    int squareX = 5, squareY = 8;

    boardTop = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, 0);
    boardSmallSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardTop->ids[boardTop->ids.size()-1] + 1);
    boardSmallSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide1->ids[boardSmallSide1->ids.size()-1] + 1);
    boardLongSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide2->ids[boardSmallSide2->ids.size()-1] + 1);
    boardLongSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide1->ids[boardLongSide1->ids.size()-1] + 1);
    additionalBoard = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide2->ids[boardLongSide2->ids.size()-1] + 1);

    if(saveToDisk)
    {
        cv::Mat boardTopImg, boardSmallSide1Img, boardSmallSide2Img, boardLongSide1Img, boardLongSide2Img, additionalBoardImg;
        int scale = 4;
        boardTop->draw(cv::Size(scale*210,scale*297), boardTopImg);
        boardSmallSide1->draw(cv::Size(scale*210,scale*297), boardSmallSide1Img);
        boardSmallSide2->draw(cv::Size(scale*210,scale*297), boardSmallSide2Img);
        boardLongSide1->draw(cv::Size(scale*210,scale*297), boardLongSide1Img);
        boardLongSide2->draw(cv::Size(scale*210,scale*297), boardLongSide2Img);
        additionalBoard->draw(cv::Size(scale*210,scale*297), additionalBoardImg);
        float borderPercent = 0;//0.25
        cv::imwrite("charucoBoardTop.png", cv::Scalar::all(255) - addBorder(boardTopImg, borderPercent));
        cv::imwrite("charucoBoardSmallSide1.png", cv::Scalar::all(255) - addBorder(boardSmallSide1Img, borderPercent));
        cv::imwrite("charucoBoardSmallSide2.png", cv::Scalar::all(255) - addBorder(boardSmallSide2Img, borderPercent));
        cv::imwrite("charucoBoardLongSide1.png", cv::Scalar::all(255) - addBorder(boardLongSide1Img, borderPercent));
        cv::imwrite("charucoBoardLongSide2.png", cv::Scalar::all(255) - addBorder(boardLongSide2Img, borderPercent));
        cv::imwrite("charucoBoardAdditional.png", cv::Scalar::all(255) - addBorder(additionalBoardImg, borderPercent));
    }
}
