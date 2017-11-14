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

#include "BBoxExtractor.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "UtilCvPclRs.h"
#include <pcl/common/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/surface/convex_hull.h>

#include "EndEffectorQRDetector.h"

// Types
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

BBoxExtractor::BBoxExtractor(bool drawResult)
    :drawResult(drawResult)
{
    editMode = 0;

    #ifdef USE_GL
    GLWin = drawResult;
    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    if(GLWin)
    {
        renderer = renderMngr.createRenderer();
        renderer->cameraPos = cv::Point3f(-0.082839,-0.003967,0.060352);
        renderer->viewTheta = 2.241272;
        renderer->viewPhi = -0.011375;
        meshData = std::shared_ptr<GLMeshData>(new GLMeshData());
        meshData->type = GLMeshData::GLMeshDataType::Points;
        mesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
        mesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
        renderer->addMesh(mesh);
        bboxMeshData = std::shared_ptr<GLMeshData>(new GLMeshData());
        bboxMeshData->type = GLMeshData::GLMeshDataType::Lines;
        bboxMesh = std::shared_ptr<GLMesh>(new GLMesh(bboxMeshData));
        bboxMesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
        renderer->addMesh(bboxMesh);
        cv::waitKey(1000);
    }
    #endif

}

void BBoxExtractor::init(const std::vector<std::string>& cameraSerial, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, cv::Ptr<cv::aruco::Dictionary> qrDictionary, int minQrId, int maxQrId)
{
    this->cameraSerial = cameraSerial;
    this->cameraK = cameraK;
    this->cameraDist = cameraDist;
    this->cameraPoses = cameraPoses;
    this->qrDictionary = qrDictionary;
    this->minQrId = minQrId;
    this->maxQrId = maxQrId;
    for(int i = 0; i < cameraSerial.size(); i++)
    {
        backgroundSub.push_back(cv::createBackgroundSubtractorMOG2());
        backgroundInit.push_back(false);
        backgroundSub[i]->setVarThreshold(10);
    }
    recognitionCenter = cv::Point3f(0.13+0.075, 0.08, -0.15);
    recognitionSize = cv::Point3f(0.4-0.15+0.1,0.4,0.4);
    suctionCylinderCenter = cv::Point3f(0.32, 0.17, -0.1);
    suctionCylinderMainAxis = cv::Point3f(1,0,0);
    suctionCylinderRadiusAxis1 = cv::Point3f(0,1,0);
    suctionCylinderRadiusAxis2 = cv::Point3f(0,0,1);
    suctionCylinderHeight = 0.1+0.04;
    suctionCylinderRadius = 0.03;
    recoMoveDir = cv::Point3f(0.01,0,0);
}

void BBoxExtractor::loadConfig(const char *filename)
{
    loadRecoSpaceConfig(filename, recognitionCenter, recognitionSize, suctionCylinderCenter, suctionCylinderMainAxis, suctionCylinderHeight, suctionCylinderRadius);
}

int BBoxExtractor::getIdBySerial(std::string serial) const
{
    return ::getIdBySerial(cameraSerial, serial);
}

void BBoxExtractor::updateBackgroundFrame(const std::vector<RgbdFrame>& listFrames)
{
    for(int i = 0; i < listFrames.size(); i++)
        updateBackgroundFrame(listFrames[i]);
}

void BBoxExtractor::updateBackgroundFrame(const RgbdFrame& frame)
{
    cv::Mat fgMaskMOG2;
    int camIndex = getIdBySerial(frame.camSerial);
    if (camIndex != -1)
    {
        backgroundSub[camIndex]->apply(frame.img, fgMaskMOG2);
        backgroundInit[camIndex] = true;
    }
    else
       printf("ERROR, cam serialID %s not found\n", frame.camSerial.c_str());
}

#ifdef USE_GL
void BBoxExtractor::drawSuctionCylinder(std::shared_ptr<GLMeshData> meshData)
{
    ::drawCylinder(meshData, cv::Point3f(realCylinderPosX, suctionCylinderCenter.y, suctionCylinderCenter.z)-suctionCylinderMainAxis*suctionCylinderHeight/2, cv::Point3f(realCylinderPosX, suctionCylinderCenter.y, suctionCylinderCenter.z)+suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderRadius, cv::Point3f(0,0,1));
}
#endif

void BBoxExtractor::drawSuctionCylinder(cv::Mat result, cv::Mat KRGB, cv::Mat pose)
{
    ::drawCylinder(result, KRGB, pose, cv::Point3f(realCylinderPosX, suctionCylinderCenter.y, suctionCylinderCenter.z)-suctionCylinderMainAxis*suctionCylinderHeight/2, cv::Point3f(realCylinderPosX, suctionCylinderCenter.y, suctionCylinderCenter.z)+suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderRadius, cv::Scalar(255,0,0));
}

std::vector<cv::Point3f> BBoxExtractor::getRecognitionBBOX()
{
    return getBoxPoints(recognitionCenter, recognitionSize);
}

bool BBoxExtractor::testPointInRecognitionVolume(cv::Point3f p, float suctionCylinderPosX)
{
    if(!pointInBox(p, recognitionCenter, recognitionSize))
        return false;

    if(pointInCylinder(p, cv::Point3f(suctionCylinderPosX, suctionCylinderCenter.y, suctionCylinderCenter.z), suctionCylinderMainAxis, suctionCylinderHeight, suctionCylinderRadius))
        return false;

    return true;
}

void BBoxExtractor::processFrame(RgbdFrame frame, pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudRGB , float suctionCylinderPosX)
{
    int t0 = clock();
    int id = getIdBySerial(frame.camSerial);
    if(id == -1)
    {
        printf("ERROR, cam serialID %s not found\n", frame.camSerial.c_str());
        return ;
    }

    cv::Mat result = frame.img.clone();
    cv::cvtColor(result, result, CV_BGR2RGB);

    if(drawResult)
        cv::imshow("depth", frame.imgDepth);

    int t1 = clock();

    cv::Mat fgMaskMOG2;
    if(backgroundInit[id])
        backgroundSub[id]->apply(frame.img, fgMaskMOG2, 0);

    int t2 = clock();

    cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
    cv::Mat poseD = cameraPoses.size() > cameraSerial.size() ? cameraPoses[id+cameraSerial.size()] : depthToColor.inv()*cameraPoses[id];
    cv::Mat KD = cameraK.size() > cameraSerial.size() ? cameraK[id+cameraSerial.size()] : frame.getKMat(frame.depth_intrin);
    cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
    cv::Mat KRGB = cameraK[id];//frame.getKMat(frame.color_intrin);

    cv::Mat mat = cameraPoses[id].inv();

    #ifdef USE_GL
    if(GLWin)
        renderer->pauseRenderer();
    #endif

    cv::Mat cameraPoseInv = (cameraPoses[id]).inv();

    int t3 = clock();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = frame.rs_cloud_ptr;

    float cx = KRGB.at<double>(0,0);
    float cy = KRGB.at<double>(1,1);
    float tx = KRGB.at<double>(0,2);
    float ty = KRGB.at<double>(1,2);
    for(int v = 0; v < cloud->height; v++)
        for(int u = 0; u < cloud->width; u++)
        {
            int i = v*cloud->width+u;
            pcl::PointXYZRGB p = cloud->points[i];
            if(std::isnan(p.x))
                continue;
            cv::Point3f p2 = multMatVecT<double, 3, 4>(depthToColor, pcl2cv(p));
            cv::Point3f p3 = multMatVecT<double, 3, 4>(cameraPoseInv, p2);//multMatVec((depthToColor.inv()*poseRGB*cameraPoses[0].inv()).inv(), p);
            cv::Point2f uv2(cx*p2.x/p2.z + tx, cy*p2.y/p2.z + ty);
            
            if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
            {
                if(fgMaskMOG2.empty() || fgMaskMOG2.at<unsigned char>((int)uv2.y, (int)uv2.x) > 0)
                {
                    if(frame.mask.empty() || frame.mask.at<unsigned char>((int)uv2.y, (int)uv2.x) > 0)
                    {
                        if(!testPointInRecognitionVolume(p3, suctionCylinderPosX))
                            continue;

                        unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                        #ifdef USE_GL
                        if(GLWin)
                            meshData->addVertex(p3, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                        #endif
                        pcl::PointXYZRGB point(rgb[0], rgb[1], rgb[2]);
                        point.x = p3.x;
                        point.y = p3.y;
                        point.z = p3.z;
                        cloudRGB->push_back(point);
                    }
                }
            }
        }

    int t4 = clock();
    printf("processFrame %d %d %d %d\n", t1-t0, t2-t1, t3-t2, t4-t3);
}

void BBoxExtractor::handleKey(int key)
{
    if(key == '+')
    {
        if(editMode == 0)
            recognitionCenter += recoMoveDir;
        else suctionCylinderCenter += recoMoveDir;
    }
    else if(key == '-')
    {
        if(editMode == 0)
            recognitionCenter -= recoMoveDir;
        else suctionCylinderCenter -= recoMoveDir;
    }
    else if(key == 'x')
        recoMoveDir = cv::Point3f(0.01,0,0);
    else if(key == 'y')
        recoMoveDir = cv::Point3f(0,0.01,0);
    else if(key == 'z')
        recoMoveDir = cv::Point3f(0,0,0.01);
    else if(key == ' ')
        editMode = (editMode+1)%2;
    if(editMode == 0)
        printf("recoCenter : %lf,%lf,%lf\n", recognitionCenter.x, recognitionCenter.y, recognitionCenter.z);
    else printf("suctionCylinderCenter : %lf,%lf,%lf\n", suctionCylinderCenter.x, suctionCylinderCenter.y, suctionCylinderCenter.z);
}

void BBoxExtractor::getConnectedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr kdtree_cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, std::vector<int>& toEvaluate, std::vector<bool>& alreadyUsed, float maxDist)
{
    float maxDist2 = maxDist*maxDist;

    for(int i = 0; i < toEvaluate.size(); i++)
    {
        std::vector<int> idx;
        std::vector<float> dist;
        pcl::PointXYZ p1 = cloud->points[toEvaluate[i]];
        kdtree.radiusSearch(cloud->points[toEvaluate[i]], maxDist, idx, dist);
        float nan  = std::numeric_limits<float>::quiet_NaN();
        for(int id : idx)
        {
            if(alreadyUsed[id])
                continue;
            toEvaluate.push_back(id);
            kdtree_cloud->points[id].x = nan;
            kdtree_cloud->points[id].y = nan;
            kdtree_cloud->points[id].z = nan;
            #ifdef USE_GL
            if(GLWin)
            {
                pcl::PointXYZ p2 = cloud->points[id];
                bboxMeshData->addVertex(pcl2cv(p1), cv::Point3f(1,1,1));
                bboxMeshData->addVertex(pcl2cv(p2), cv::Point3f(1,1,1));
            }
            #endif
            alreadyUsed[id] = true;
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BBoxExtractor::getConnectedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Point3f startPoint, float maxDistInit, float maxDistCluster)
{
    int t0 = clock();
    float maxDistInit2 = maxDistInit*maxDistInit;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud_tmp = *cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_tmp);

    std::vector<bool> alreadyUsed(cloud->size());
    for(int i = 0; i < alreadyUsed.size(); i++)
        alreadyUsed[i] = false;

    std::vector<int> toEvaluate;

    std::vector<int> idx;
    std::vector<float> distSqr;
    kdtree.radiusSearch(pcl::PointXYZ(startPoint.x, startPoint.y, startPoint.z), maxDistInit, idx, distSqr);
    float nan  = std::numeric_limits<float>::quiet_NaN();
    for(int id : idx)
    {
        alreadyUsed[id] = true;
        toEvaluate.push_back(id);
        cloud_tmp->points[id].x = nan;
        cloud_tmp->points[id].y = nan;
        cloud_tmp->points[id].z = nan;
    }
    
    getConnectedCloud(cloud, cloud_tmp, kdtree, toEvaluate, alreadyUsed, maxDistCluster);

    if(toEvaluate.size() == 0)
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<std::vector<int> > clusters;
    clusters.push_back(toEvaluate);

    int firstRemainingPoint = 0;
    while(firstRemainingPoint < cloud->size())
    {
        toEvaluate.clear();
        while(firstRemainingPoint < cloud->size() && alreadyUsed[firstRemainingPoint])
            firstRemainingPoint++;
        if(firstRemainingPoint < cloud->size())
        {
            alreadyUsed[firstRemainingPoint] = true;
            toEvaluate.push_back(firstRemainingPoint);
            getConnectedCloud(cloud, cloud_tmp, kdtree, toEvaluate, alreadyUsed, maxDistCluster);
            clusters.push_back(toEvaluate);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersCloud(clusters.size());
    std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr> clustersTree(clusters.size());
    std::vector<pcl::ConvexHull<pcl::PointXYZ>::Ptr> clustersHull (clusters.size());

    bool modif = true;
    while(modif)
    {
        modif = false;
        std::vector<bool> toAdd(clusters.size());
        for(int i = 0; i < toAdd.size(); i++)
            toAdd[i] = false;
        for(int i = 0; i < clusters.size(); i++)
        {
            clustersCloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            for(int j = 0; j < clusters[i].size(); j++)
                clustersCloud[i]->push_back(cloud->points[clusters[i][j]]);
            if(clustersCloud[i]->size() >= 3)
            {
                clustersHull[i] = pcl::ConvexHull<pcl::PointXYZ>::Ptr(new pcl::ConvexHull<pcl::PointXYZ>());
                clustersHull[i]->setComputeAreaVolume(true);
                clustersHull[i]->setInputCloud(clustersCloud[i]);
                pcl::PointCloud<pcl::PointXYZ> cHull_points;
                clustersHull[i]->reconstruct(cHull_points);
                clustersTree[i] = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
                clustersTree[i]->setInputCloud(clustersCloud[i]);
                if(i > 0)
                {
                    float minDist = std::numeric_limits<float>::infinity();
                    int minId0 = 0, minId1;
                    for(int j = 0; j < clustersCloud[i]->size(); j++)
                    {
                        std::vector<int> idx;
                        std::vector<float> distSqr;
                        if(clustersTree[0]->nearestKSearch(clustersCloud[i]->points[j], 1, idx, distSqr) == 0)
                            continue;
                        if(distSqr[0] < minDist)
                        {
                            minDist = distSqr[0];
                            minId0 = idx[0];
                            minId1 = j;
                        }
                    }
                    minDist = sqrt(minDist);
                    printf("distance to cluster %d : %f\n", i, minDist);
                    float area0 = clustersHull[0]->getTotalArea(), area1 = clustersHull[i]->getTotalArea();
                    printf("area of cluster 0: %lf sqrt %lf\n", area0, sqrt(area0));
                    printf("area of cluster %d: %lf sqrt %lf\n", i, area1, sqrt(area1));
                    cv::Point3f color(1,0,0);
                    if(minDist < maxDistInit && minDist < sqrt(area1))
                    {
                        color = cv::Point3f(0,1,0);
                        toAdd[i] = true;
                        modif = true;
                    }
                    #ifdef USE_GL
                    if(GLWin)
                    {
                        bboxMeshData->addVertex(pcl2cv(clustersCloud[0]->points[minId0]), color);
                        bboxMeshData->addVertex(pcl2cv(clustersCloud[i]->points[minId1]), color);
                    }
                    #endif


                }
            }
            else if(i == 0)
                break;
        }

        if(modif)
        {
            for(int i = 0; i < clusters.size(); i++)
            {
                if(toAdd[i])
                {
                    for(int j = 0; j < clustersCloud[i]->size(); j++)
                    {
                        clustersCloud[0]->push_back(clustersCloud[i]->points[j]);
                        clusters[0].push_back(clusters[i][j]);
                    }
                    toAdd[i] = toAdd[clusters.size()-1];
                    clusters[i] = clusters[clusters.size()-1];
                    clustersCloud[i] = clustersCloud[clusters.size()-1];
                    clustersTree[i] = clustersTree[clusters.size()-1];
                    clustersHull[i] = clustersHull[clusters.size()-1];
                    toAdd.pop_back();
                    clusters.pop_back();
                    clustersCloud.pop_back();
                    clustersTree.pop_back();
                    clustersHull.pop_back();
                    i--;
                }
            }
        }
    }
    
    int t1 = clock();
    printf("time %d\n", t1-t0);
    return clustersCloud[0];
}

cv::Mat BBoxExtractor::generateView(const RgbdFrame& frame, const std::vector<cv::Point3f>& recoBBOX, const std::vector<cv::Point3f>& OBB, const std::vector<cv::Point3f>& AABB, const std::vector<std::vector<cv::Point3f> >& qrCodePoints)
{
    int id = getIdBySerial(frame.camSerial);
    cv::Mat KRGB = cameraK[id];//frame.getKMat(frame.color_intrin);
    cv::Mat pose = cameraPoses[id];
    cv::Mat result = frame.img.clone();
    cv::cvtColor(result, result, CV_BGR2RGB);
    drawSuctionCylinder(result, KRGB, pose);
    if(OBB.size() > 0)
        drawBox(result, KRGB, pose, OBB, cv::Scalar(0,255,0));
    if(AABB.size() > 0)
        drawBox(result, KRGB, pose, AABB, cv::Scalar(0,0,255));
    if(recoBBOX.size() > 0)
        drawBox(result, KRGB, pose, recoBBOX, cv::Scalar(255,0,0));
    for(int i2 = 0; i2 < qrCodePoints.size(); i2++)
    {
        for(int j = 0; j < qrCodePoints[i2].size(); j++)
        {
            int j2 = (j-(j%4)) + ((j+1)%4);
            std::vector<cv::Point3f> line = {qrCodePoints[i2][j], qrCodePoints[i2][j2]};
            std::vector<cv::Point2f> line2d = project2d(KRGB, pose, line);
            cv::line(result, line2d[0], line2d[1], cv::Scalar(0,255,0));
        }
    }
    return result;
}

void BBoxExtractor::compute(std::vector<RgbdFrame> listFrames)
{
    segmentedCloud = pcl::PointCloud <pcl::PointXYZ>::Ptr();
    _AABB = std::vector<cv::Point3f>();
    _OBB = std::vector<cv::Point3f>();
    _recoBBOX = getRecognitionBBOX();

    int t0 = clock();
    for(int i = 0; i < listFrames.size(); i++)
    {
        int id = getIdBySerial(listFrames[i].camSerial);
        if(listFrames[i].rs_cloud_ptr->empty())
            generatePointCloudOrganized2(listFrames[i], false, false, false);
    }
    int t1 = clock();
    printf("\ngeneratePointCloudOrganized on all frames : %d\n", t1-t0);//425000 clock

    std::vector<cv::Point3f> camOrig;
    std::vector<std::pair<cv::Point3f, cv::Point3f> > cylinderIntersections;
    std::vector<std::vector<std::vector<cv::Point3f> > > qrCodeVec;
    std::vector<std::vector<cv::Point3f> > qrCodePoints;
    float qrHeight;
    bool suctionQRFound = findEndEffectorQRPosX(suctionCylinderCenter, suctionCylinderHeight, suctionCylinderRadius, listFrames, cameraSerial, cameraK, cameraDist, cameraPoses, qrDictionary, minQrId, maxQrId, camOrig, cylinderIntersections, qrCodeVec, qrCodePoints, qrHeight);

    realCylinderPosX = suctionQRFound?(qrHeight-0.025+suctionCylinderHeight/2):suctionCylinderCenter.x;
    {

        pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud <pcl::PointXYZRGB>);
        int count = 0;
        #ifdef USE_GL
        if(GLWin)
        {
            renderer->pauseRenderer();
            meshData->clear();
        }
        #endif
        int t2 = clock();
        //compute each frame point cloud in unified space, and compute bbox
        for(int i = 0; i < listFrames.size(); i++)
            processFrame(listFrames[i], cloudRGB, realCylinderPosX);
        int t3 = clock();
        printf("time to process all frames : %d\n", t3-t2);

        //filter the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = filterStatisticalOutlier(cloudRGB);

        #ifdef USE_GL
        if(GLWin)
        {
            renderer->pauseRenderer();
            meshData->clear();
            bboxMeshData->clear();

            for(int i = 0; i < cloudRGB->size(); i++)
            {
                pcl::PointXYZRGB p = cloudRGB->points[i];
                meshData->addVertex(cv::Point3f(p.x, p.y, p.z), cv::Point3f(p.r, p.g, p.b)/255.0);
            }
        }
        #endif

        bool disable_transform = false;
        float voxel_resolution = 0.008f;
        float seed_resolution = 0.01f;//0.1f;
        float color_importance = 0.2f;
        float spatial_importance = 0.4f;
        float normal_importance = 1.0f;
        pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, false);
        super.setInputCloud (cloud_filtered);
        super.setColorImportance (color_importance);
        super.setSpatialImportance (spatial_importance);
        super.setNormalImportance (normal_importance);

        std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
        pcl::console::print_highlight ("Extracting supervoxels!\n");
        int t0 = clock();
        super.extract (supervoxel_clusters);
        int t1 = clock();
        printf("extract time %d\n", t1-t0);
        pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

        if(supervoxel_clusters.size() == 0)
        {
            _AABB = std::vector<cv::Point3f>();
            _OBB = std::vector<cv::Point3f>();
            _recoBBOX = getRecognitionBBOX();
            return ;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud <pcl::PointXYZ>);
        PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
        std::map<uint32_t, cv::Point3f> colors;
        for(int i = 0; i < labeled_voxel_cloud->size(); i++)
        {
            PointLT p = labeled_voxel_cloud->points[i];
            if(colors.find(p.label) == colors.end())
            {
                colors[p.label] = cv::Point3f(rand()%255,rand()%255,rand()%255)/255.0;
                colors[p.label] /= sqrt(colors[p.label].dot(colors[p.label]));
            }
            centroids->push_back(pcl::PointXYZ(p.x, p.y, p.z));
            #ifdef USE_GL
            if(GLWin)
                meshData->addVertex(cv::Point3f(p.x, p.y, p.z), colors[p.label]);
            #endif
        }
        int t4 = clock();
        printf("clustering time : %d\n", t4-t3);

        if(centroids->size() == 0)
            return ;

        cv::Point3f attachPoint = suctionCylinderCenter - suctionCylinderMainAxis*suctionCylinderHeight/2;
        segmentedCloud = getConnectedCloud(centroids, attachPoint);
        int t5 = clock();
        printf("getConnectedCloud time : %d\n", t5-t4);

        if(segmentedCloud->size() == 0)
            return ;



        //compute the recognition space bbox
        std::vector<cv::Point3f> recoBBOX = getRecognitionBBOX();

        //compute the object AABB and OBB
        std::vector<cv::Point3f> AABB = computeAABB(segmentedCloud);
        std::vector<cv::Point3f> OBB = computeOBB(segmentedCloud, true, 2*CV_PI/180);//_segmented);

        /*drawSuctionCylinder(result, KRGB, cameraPoses[id]);
        drawBox(result, KRGB, cameraPoses[id], recoBBOX, cv::Scalar(255,0,0));
        drawBox(result, KRGB, cameraPoses[id], OBB, cv::Scalar(0,255,0));
        drawBox(result, KRGB, cameraPoses[id], AABB, cv::Scalar(0,0,255));*/

        int t6 = clock();
        printf("computeOBB time : %d\n", t6-t5);



        #ifdef USE_GL
        if(GLWin)
        {
            //bboxMeshData->clear();
            drawSuctionCylinder(bboxMeshData);
            ::drawBox(bboxMeshData, OBB, cv::Point3f(0,1,0));
            ::drawBox(bboxMeshData, AABB, cv::Point3f(1,0,0));
            ::drawBox(bboxMeshData, recoBBOX, cv::Point3f(0,0,1));
            renderer->render();
        }
        #endif

        if(drawResult)
        {
            for(int i = 0; i < listFrames.size(); i++)
            {
                cv::Mat result = generateView(listFrames[i], recoBBOX, OBB, AABB, qrCodePoints);
                char name[255];
                sprintf(name, "result%s", listFrames[i].camSerial.c_str());
                cv::imshow(name, result);
            }
        }

        _recoBBOX = recoBBOX;
        _OBB = OBB;
        _AABB = AABB;

        int key = cv::waitKey(100);
        handleKey(key);
        count++;
    }
}
