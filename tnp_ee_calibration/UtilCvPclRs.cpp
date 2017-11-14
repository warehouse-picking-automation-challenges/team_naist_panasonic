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


std::string mat2str(cv::Mat mat)
{
    std::string str;
    char tmp[255];
    sprintf(tmp, "%dx%d\n", mat.cols, mat.rows);
    str += tmp;
    for(int i = 0; i < mat.rows; i++)
    {
        for(int j = 0; j < mat.cols; j++)
        {
            if(mat.type()==CV_32F)
                sprintf(tmp, "%f", mat.at<float>(i, j));
            else if(mat.type()==CV_64F)
                sprintf(tmp, "%lf", mat.at<double>(i, j));
            str += tmp;
            if(j < mat.cols-1)
                str += ", ";
            else str += "\n";
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
    }
    cv::Mat mat, inl;
    estimateAffine3D(model, OBB, mat, inl);
    mat = mat(cv::Rect(0,0,3,3)).clone();
    mat.convertTo(mat, CV_64F);
    return mat;
}

bool loadCalib(const char *filename, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses, std::vector<cv::Mat>& planeRt)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
      printf("calib : can't load %s\n", filename);
      return false;
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

bool loadCalib(const char *filename, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, std::vector<cv::Mat>& poses)
{
    std::vector<cv::Mat> planeRt;
    return loadCalib(filename, cameraK, cameraDist, poses, planeRt);
}


std::vector<std::string> loadFramesIdSerial(const char *seq_folder, int nbCameras, int frameId)
{
    std::vector<std::string> list;
    for(int j = 0; j < nbCameras; j++)
    {
        RgbdFrame frame;
        char name[255];
        sprintf(name, "capture%d_cam%d", frameId, j);
        if(frame.load(seq_folder, name))
            list.push_back(frame.camSerial);
    }
    return list;
}

std::vector<RgbdFrame> loadFramesId(const char *seq_folder, int nbCameras, int frameId)
{
    std::vector<RgbdFrame> list;
    for(int j = 0; j < nbCameras; j++)
    {
        RgbdFrame frame;
        char name[255];
        sprintf(name, "capture%d_cam%d", frameId, j);
        if(frame.load(seq_folder, name))
            list.push_back(frame);
    }
    return list;
}

std::vector<std::vector<RgbdFrame> > loadFrames(const char *seq_folder, int nbCameras, int nbFrames)
{
    std::vector<std::vector<RgbdFrame> > listFrames;
    for(int i = 0; i < nbFrames; i++)
        listFrames.push_back(loadFramesId(seq_folder, nbCameras, i));
    return listFrames;
}
