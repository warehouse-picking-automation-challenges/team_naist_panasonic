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

#include <cpu_tsdf/tsdf_volume_octree.h>
#include "tsdf_integration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/ximgproc/lsc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/text.hpp>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "GLRenderer.h"

#include "UtilCvPclRs.h"
#include "RGBDFrame.h"
#include "CeresModels.h"
#include "VisualHull.h"
#include "Calibration.h"
#include "FeaturesMatcher.h"
#include "cArduino.h"

#include "BBoxExtractor.h"
#include "BackgroundSubtraction.h"

#include <sys/stat.h>

void groups_draw(cv::Mat &src, std::vector<cv::Rect> &groups)
{
    for (int i=(int)groups.size()-1; i>=0; i--)
    {
        if (src.type() == CV_8UC3)
            cv::rectangle(src,groups.at(i).tl(),groups.at(i).br(),cv::Scalar( 0, 255, 255 ), 3, 8 );
        else
            cv::rectangle(src,groups.at(i).tl(),groups.at(i).br(),cv::Scalar( 255 ), 3, 8 );
    }
}

void IRanalyser(int depthCamId)
{
    rs::context rsContext;
    printRSContextInfo( &rsContext );

    std::vector<rs::device*> rsCameras;
    for(int i = 0; i < rsContext.get_device_count(); i++)
    {
        rsCameras.push_back(rsContext.get_device(i));
        if(i == depthCamId)
            configureRSStreams( rsCameras[rsCameras.size()-1], true, false, false);
        else configureRSStreams( rsCameras[rsCameras.size()-1], false, true, false);
    }

    int laserPower = 0;
    bool IR = false;
    int count = 0;
    while( true )
    {
        if(count == 20)
        {
            IR = !IR;
            for(int camId = 0; camId < rsCameras.size(); camId++)
            {
                if(isR200(rsCameras[camId]))
                    rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, IR&&laserPower>0?1:0);
                else rsCameras[camId]->set_option(rs::option::f200_laser_power, IR?laserPower:0);
            }
            laserPower = (laserPower+1)%17;
            count = 0;
        }

        std::vector<RgbdFrame> frames(rsCameras.size());
        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            if(camId == depthCamId)
                frames[camId] = (extractDataFromRs(rsCameras[camId], true, false, false));
            else frames[camId] = (extractDataFromRs(rsCameras[camId], false, true, false));
        }

        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            RgbdFrame frame = frames[camId];

            char txt[255];
            double laserPower = 0;
            if(isR200(rsCameras[camId]))
                laserPower = rsCameras[camId]->get_option(rs::option::r200_emitter_enabled);
            else laserPower = rsCameras[camId]->get_option(rs::option::f200_laser_power);

            if(IR)
                sprintf(txt, "on %lf", laserPower);
            else sprintf(txt, "off %lf", laserPower);

            cv::Mat result;

            if(camId == depthCamId)
                result = frame.depth.clone();
            else result = frame.imgIR.clone();

            cv::putText(result, txt, cv::Point(100,100), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255,255,255));

            char name[255];
            if(camId == depthCamId)
                sprintf(name, "depth");
            else sprintf(name, "IR%d", camId);

            if(count == 2)
                cv::imshow(name, result);

        }

        int key = cv::waitKey(100);
        count++;

    }
}


void manualRecord(cArduino &arduino, cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, bool depth, bool sequential = false, int selectedCamId = -1)
{
    if(sequential && selectedCamId == -1)
        selectedCamId = 0;
    int nbCaptures = 0;
    while(true)
    {
        std::vector<std::vector<RgbdFrame> > listFrames;

        rs::context rsContext;
        printRSContextInfo( &rsContext );

        std::vector<rs::device*> rsCameras;
        for(int i = 0; i < rsContext.get_device_count(); i++)
        {
            if(selectedCamId >= 0 && selectedCamId != i)
                continue;
            rsCameras.push_back(rsContext.get_device(i));
            if(depth)
                configureRSStreams( rsCameras[rsCameras.size()-1], true, false, false);
            else configureRSStreams( rsCameras[rsCameras.size()-1], false, true, isR200(rsCameras[rsCameras.size()-1]));
        }

        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            if(isR200(rsCameras[camId]))
                rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 0);
            else
            {
                if(depth)
                    rsCameras[camId]->set_option(rs::option::f200_laser_power, 0);
                else rsCameras[camId]->set_option(rs::option::f200_laser_power, 15);
            }
        }

        int rotation = 0;
        bool capture = false;
        int record = 0;
        char name[255];

        std::vector<cv::Mat> cameraMatrix(rsCameras.size()*3);
        std::vector<cv::Mat> cameraDistCoeffs(rsCameras.size()*3);

        while( true )
        {
            if(rotation != 0)
            {
                if(rotation > 0)
                    arduino.write("1\n");
                else arduino.write("2\n");
                int key = cv::waitKey(1000);
                if(key != 83 && key != 81)
                {
                    arduino.write("0\n");
                    key = cv::waitKey(500);
                }
                if(key == 83)
                    rotation = 1;
                else if(key == 81)
                    rotation = -1;
                else rotation = 0;
            }

            std::vector<RgbdFrame> frames(rsCameras.size());
            for(int camId = 0; camId < rsCameras.size(); camId++)
            {
                if(depth)
                {
                    if(isR200(rsCameras[camId]))
                        rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 1);
                    else rsCameras[camId]->set_option(rs::option::f200_laser_power, 15);

                    for (int i = 0; i < 3; i++)
                    {
                        printf("waitForFrames %d...\n", camId);
                        rsCameras[camId]->wait_for_frames();
                        printf("done\n");
                    }
                    frames[camId] = (extractDataFromRs(rsCameras[camId], true, false, false));
                    if(isR200(rsCameras[camId]))
                        rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 0);
                    else rsCameras[camId]->set_option(rs::option::f200_laser_power, 0);
                }
                else frames[camId] = (extractDataFromRs(rsCameras[camId], false, true, isR200(rsCameras[camId])));
            }

            for(int camId = 0; camId < rsCameras.size(); camId++)
            {
                if(capture)
                    listFrames[listFrames.size()-1][camId] = frames[camId];
            }

            for(int camId = 0; camId < rsCameras.size(); camId++)
            {
                RgbdFrame frame = frames[camId];
                if(capture)
                {
                    sprintf(name, "capture%d_cam%d", nbCaptures-1, sequential?selectedCamId:camId);
                    frame.save("capture", name);
                    printf("capture\n");
                }
                std::vector<cv::Mat> listImg;
                listImg.push_back(frame.img);
                if(!depth)
                {
                    listImg.push_back(frame.imgIR);
                    if(isR200(rsCameras[camId]))
                        listImg.push_back(frame.imgIR2);
                }
                else listImg.push_back(frame.depth);

                for(int j = 0; j < listImg.size(); j++)
                {
                    cv::Mat imgInv = cv::Scalar::all(255) - listImg[j];
                    cv::Mat result = listImg[j].clone();
                    if(j == 1 && depth)
                        result = 10*result;
                    int j2 = j*rsCameras.size()+camId;

                    if(j == 0 || !depth)
                    {
                        if(cameraMatrix[j2].empty())
                        {
                            cameraMatrix[j2] = getCameraMatrix(frame, j);
                            cameraDistCoeffs[j2] = getCameraDistMatrix(frame, imgInv.size(),  j);
                        }

                        std::vector<int> markerIds;
                        std::vector<std::vector<cv::Point2f> > markerCorners;
                        std::vector<cv::Point2f> charucoCorners;
                        std::vector<int> charucoIds;
                        cv::Mat rvec, tvec;
                        detectMarker(imgInv, dictionary, board, cameraMatrix[j2], cameraDistCoeffs[j2], markerIds, markerCorners, charucoCorners, charucoIds, rvec, tvec, result);
                    }

                    if(j == 0)
                        sprintf(name, "img%d", camId);
                    else if(depth)
                    {
                        if(j == 1)
                            sprintf(name, "depth_%d", camId);
                    }
                    else
                    {
                        if(j == 1)
                            sprintf(name, "IR1_%d", camId);
                        else if(j == 2)
                            sprintf(name, "IR2_%d", camId);
                    }
                    cv::imshow(name, result);
                }
            }

            if(record > 0)
            {
                if(capture)
                {
                    rotation = 1;
                    capture = false;
                }
                else
                {
                    capture = true;
                    listFrames.push_back(std::vector<RgbdFrame>(rsCameras.size()));
                    nbCaptures++;
                    record--;
                }
            }
            else capture = false;

            int key = cv::waitKey(100);
            if(key != 255)
                printf("Key %d\n", key);
            if(key == 'l')//if(key == 83)
                rotation = 1;
            else if(key == 'r')//if(key == 81)
                rotation = -1;
            else if(key == 's')
            {
                capture = true;
                listFrames.push_back(std::vector<RgbdFrame>(rsCameras.size()));
                nbCaptures++;
            }
            else if(key == 'r')
            {
                record = 50;
                capture = true;
                listFrames.push_back(std::vector<RgbdFrame>(rsCameras.size()));
                nbCaptures++;
            }
            else if(key == 'n')
            {
                selectedCamId = (selectedCamId+1)%rsContext.get_device_count();
                break;
            }
            else if(key == 'q')
            {
                for(int i = 0; i < rsCameras.size(); i++) {
                    rsCameras[i]->stop();
                    printf("Camera stopped\n");
                    cv::waitKey(5000);
                }
                cv::waitKey(5000);
                return ;
            }
        }
        for(int camId = 0; camId < rsCameras.size(); camId++)
            rsCameras[camId]->stop();
    }
}

void calibrateFromRecord(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, const char *seq_folder, int nbCameras, int nbFrames, int nbIR, std::vector<cv::Mat>& cameraKOut, std::vector<cv::Mat>& cameraDistOut, std::vector<cv::Mat>& cameraPoseOut )
{
    std::vector<std::vector<RgbdFrame> > listFrames = loadFrames(seq_folder, nbCameras, nbFrames);
    calibrate(dictionary, board, listFrames, cameraKOut, cameraDistOut, cameraPoseOut, nbIR);
}

void calibrateMultiboardFromRecord(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const char *seq_folder, int nbCameras, int nbFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPose, std::vector<cv::Mat>& newCameraPoses, std::vector<cv::Mat>& objectRt, std::vector<cv::Mat>& planeRt, bool optimizePlaneRt, bool useDepth, int firstFrameId = 0, const char *prefix = "capture")
{
    std::vector<std::vector<RgbdFrame> > listFrames = loadFrames(seq_folder, nbCameras, nbFrames, firstFrameId, prefix);
    calibrateMultiboard(dictionary, listBoard, listFrames, cameraK, cameraDist, cameraPose, newCameraPoses, objectRt, planeRt, optimizePlaneRt, useDepth);
}

void getCameraPosesMultiboardFromRecord(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const char *seq_folder, int nbCameras, int nbFrames, std::vector<cv::Mat>& cameraK, std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& planeRt, std::vector<cv::Mat>& cameraPose, int firstFrameId = 0, const char *prefix = "capture")
{
    std::vector<std::vector<RgbdFrame> > listFrames = loadFrames(seq_folder, nbCameras, nbFrames, firstFrameId, prefix);
    if(listFrames.size()==0)
        return ;
    if(cameraK.size()==0)
    {
        for(int i = 0; i < listFrames[0].size(); i++)
        {
            RgbdFrame frame = listFrames[0][i];
            cameraK.push_back(frame.getKMat(frame.color_intrin));
            cameraDist.push_back(frame.getDistMat(frame.color_intrin));
        }
    }
    getCameraPoseFromMultiboard(dictionary, listBoard, listFrames, cameraK, cameraDist, planeRt, cameraPose);
}


void loadFeatures(const char *filename, std::vector<cv::Point3f>& featuresPos3d, std::vector<cv::Point3f>& featuresNormal3d, cv::Mat& featuresDesc)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    printf("read keypoints\n");
    featuresPos3d = std::vector<cv::Point3f>(fs["keypoints"].size());
    for(int i = 0; i < fs["keypoints"].size(); i++)
        fs["keypoints"][i] >> featuresPos3d[i];
    printf("read normals\n");
    featuresNormal3d = std::vector<cv::Point3f>(fs["normals"].size());
    for(int i = 0; i < fs["normals"].size(); i++)
        fs["normals"][i] >> featuresNormal3d[i];
    printf("read features\n");
    fs["features"] >> featuresDesc;
    fs.release();
}

void saveFeatures(const char *filename, const std::vector<cv::Point3f>& featuresPos3d, const std::vector<cv::Point3f>& featuresNormal3d, cv::Mat featuresDesc)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "keypoints" << "[";
    for(int i = 0; i < featuresPos3d.size(); i++)
        fs << featuresPos3d[i];
    fs << "]";
    fs << "normals" << "[";
    for(int i = 0; i < featuresNormal3d.size(); i++)
        fs << featuresNormal3d[i];
    fs << "]";
    fs << "features" << featuresDesc;
    fs.release();
}

void renderMultiboardFromRecord(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const char *seq_folder, int nbCameras, int nbFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, const std::vector<cv::Mat>& planeRt, cv::Mat& resultImg0, std::vector<cv::Point3f>& featuresPos3d, std::vector<cv::Point3f>& featuresNormal3d, cv::Mat& featuresDescFull)
{    
    bool GLWin = true;
    TSDFIntegration tsdf;

    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    std::shared_ptr<GLRenderer> renderer;
    if(GLWin)
    {
    renderer = renderMngr.createRenderer();
    renderer->cameraPos = cv::Point3f(-0.082839,-0.003967,0.060352);
    renderer->viewTheta = 0.343010;
    renderer->viewPhi = 0.297746;
    }
    std::shared_ptr<GLMeshData> meshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    meshData->type = GLMeshData::GLMeshDataType::Points;
    std::shared_ptr<GLMesh> mesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
    mesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    if(GLWin)
        renderer->addMesh(mesh);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    std::vector<cv::Mat> featuresDesc;

    cv::Mat firstImg;
    cv::Mat origRT, origPose;

    cv::Point3f minBoard = getMinBoard(listBoard[0]);
    cv::Point3f maxBoard = getMaxBoard(listBoard[0]);
    float maxHeight = std::max(maxBoard.x-minBoard.x, maxBoard.y-minBoard.y)/2;
    cv::Point3f bbox[8];
    for(int k = 0; k < 8; k++)
        bbox[k] = cv::Point3f((k%2)>=1?maxBoard.x:minBoard.y,(k%4)>=2?maxBoard.y:minBoard.y,k>=4?maxHeight:0);

    std::vector<RgbdFrame> listFrames;
    std::vector<cv::Mat> listKRGB;
    std::vector<cv::Mat> listKD;
    std::vector<cv::Mat> listPoseRGB;
    std::vector<cv::Mat> listPoseD;
    std::vector<cv::Mat> listBestRT;
    std::vector<cv::Mat> listFirstFramePose;
    std::vector<std::vector<int> > listMarkerIds;
    std::vector<std::vector<std::vector<cv::Point2f> > > listMarkerCorners;
    std::vector<std::vector<cv::Point2f> > listCharucoCorners;
    std::vector<std::vector<int> > listCharucoIds;
    std::vector<cv::Mat> resultImg;

    for(int i = 0; i < nbFrames; i++)
    {
        cv::Mat bestRT;
        std::vector<RgbdFrame> frames = loadFramesId(seq_folder, nbCameras, i);
        for(int j = 0; j < frames.size(); j++)
        {
            cv::Mat imageUndistorted;
            undistort(frames[j].img, imageUndistorted, cameraK[j], cameraDist[j]);
            frames[j].img = imageUndistorted;
        }
        bestRT = detectMultiboardPose(dictionary, listBoard, frames, cameraK, cameraDist, cameraPoses, planeRt)*planeRt[0];
        if(i == 0)
            firstImg = frames[0].img.clone();

        for(int j = 0; j < nbCameras; j++)
        {
            listKRGB.push_back(frames[j].getKMat(frames[j].color_intrin));//cameraK[j]);
            listPoseRGB.push_back(cameraPoses[j]);
            listKD.push_back(frames[j].getKMat(frames[j].depth_intrin));//cameraK[j+nbCameras]);
            listPoseD.push_back(frames[j].getRtMat(frames[j].depth_to_color).inv()*cameraPoses[j]);

            listFrames.push_back(frames[j]);
            listBestRT.push_back(bestRT.clone());
        }

        if(origRT.empty())
        {
            origRT = bestRT;
        }
    }

    for(int i = 0; i < listFrames.size(); i++)
    {
        cv::Mat firstFramePose = origRT*listBestRT[i].inv();

        listFirstFramePose.push_back(firstFramePose);
    }

    cv::Mat worldToBoard = origRT.inv();
    std::vector<cv::Mat> toBoard(listFrames.size());
    for(int i = 0; i < listFrames.size(); i++)
        toBoard[i] = worldToBoard*listFirstFramePose[i]*listPoseRGB[i].inv();

    for(int i = 0; i < listFrames.size(); i++)
    {
        generatePointCloudOrganized2(listFrames[i], true, false, false);
        RgbdFrame frame = listFrames[i];

        cv::Mat result = frame.img.clone();

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        akaze->detectAndCompute(frame.img, cv::Mat(), keypoints, descriptors);

        cv::Mat firstFramePose = origRT*listBestRT[i].inv();
        cv::Mat boardToWorld = (firstFramePose*listBestRT[i]);
        cv::Mat worldToBoard = boardToWorld.inv();

        for(int j = 0; j < listBoard.size(); j++)
        for(int k = 0; k < listBoard[j]->chessboardCorners.size(); k++)
        {
            cv::Point3f p = listBoard[j]->chessboardCorners[k];
            cv::Point3f p2 = multMatVec(toBoard[i].inv()*planeRt[j], p);//multMatVec(boardToWorld, p);
            cv::Point3f uv = multMatVec(listKRGB[i], p2/p2.z);
            cv::circle(result, cv::Point(uv.x, uv.y), 3, cv::Scalar(0,0,255));
        }
        cv::imshow("result", result);
        cv::waitKey(10000);

        cv::Mat poseRGBInv = listPoseRGB[i].inv();
        for(int k = 0; k < keypoints.size(); k++)
        {
            cv::Point p = keypoints[k].pt;
            int id = p.y * frame.imgPoints->width + p.x;

            pcl::PointXYZ p2t = frame.imgPoints->points[id];
            pcl::PointXYZ p2tN(p2t.x + frame.imgNormals->points[id].normal_x, p2t.y + frame.imgNormals->points[id].normal_y, p2t.z + frame.imgNormals->points[id].normal_z);

            cv::Point3f p2(p2t.x, p2t.y, p2t.z);
            cv::Point3f p2N(p2tN.x, p2tN.y, p2tN.z);
            cv::Point3f p3 = multMatVec(firstFramePose*poseRGBInv, p2);
            cv::Point3f p3N = multMatVec(firstFramePose*poseRGBInv, p2N);
            cv::Point3f p3c = multMatVec(worldToBoard, p3);
            cv::Point3f p3cN = multMatVec(worldToBoard, p3N);
            if(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z > 0 /*&& p3c.x > minBoard.x && p3c.x < maxBoard.x && p3c.y > minBoard.y && p3c.y < maxBoard.y && p3c.z > 0.01*/)
            {
                cv::circle(result, p, 3, cv::Scalar(0,0,255));
                featuresPos3d.push_back(p3c);
                featuresNormal3d.push_back(p3cN-p3c);
                featuresDesc.push_back(descriptors(cv::Rect(0,k,descriptors.cols,1)).clone());
            }
        }
        resultImg.push_back(result);
        if(i == 0)
            resultImg0 = result.clone();
    }

    renderer->cameraPos = multMatVec(planeRt[0].inv()*toBoard[1], cv::Point3f(0,0,0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0; i < 3 && i < listFrames.size(); i++)
    {
        if(!listPoseRGB[i].empty())
        {
            RgbdFrame frame = listFrames[i];

            if(GLWin)
                renderer->pauseRenderer();

            cv::Mat poseD = listPoseD[i];
            cv::Mat poseDInv = poseD.inv();
            cv::Mat poseRGB = listPoseRGB[i];
            cv::Mat KD = frame.getKMat(frame.depth_intrin);
            cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
            cv::Mat KRGB = listKRGB[i];//frame.getKMat(frame.color_intrin);
            cv::Mat firstFramePose = origRT*listBestRT[i].inv();


            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
            Eigen::Affine3d pose;
            cv::Mat poseCV = (firstFramePose * poseDInv);
            Eigen::Matrix4d eigenMat;
            cv2eigen(poseCV, eigenMat);
            pose = eigenMat;

            cv::Mat boardToWorld = (firstFramePose*listBestRT[i]);
            cv::Mat worldToBoard = boardToWorld.inv();

            for(int v = 0; v < frame.depth.rows; v++)
                for(int u = 0; u < frame.depth.cols; u++)
                {
                    uint16_t depth_val16 = frame.depth.at<uint16_t>(v, u);
                    if(depth_val16 == 0)
                        continue;
                    float depth_value = depth_val16*frame.scale;
                    cv::Point3f depth_point = frame.depthCameraDeproject(cv::Point2f(v,u), depth_value);
                    cv::Point3f p = frame.depthCameraToColorCamera(depth_point);
                    cv::Point2f uv2 = frame.colorCameraProject(p);
                    
                    cv::Point3f p3 = multMatVec(planeRt[0].inv()*toBoard[i], p);
                    
                    unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                    meshData->addVertex(p3, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                }

            if(i == listFrames.size()-1)
            {
                std::vector<cv::Point3f> OBB = computeOBB(cloud3);
                for(int k = 0; k < OBB.size(); k++)
                {
                    meshData->addVertex(OBB[k], cv::Point3f(0,1,0));
                }
            }

            if(cloud2->size() > 10)
            {
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients (true);
                // Mandatory
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold (0.01);

                seg.setInputCloud (cloud2);
                seg.segment (*inliers, *coefficients);

                std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

                std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
                  for (size_t i = 0; i < inliers->indices.size (); ++i)
                    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                               << cloud->points[inliers->indices[i]].y << " "
                                                               << cloud->points[inliers->indices[i]].z << std::endl;

                for (size_t i = 0; i < inliers->indices.size (); ++i)
                {
                    pcl::PointXYZ p = cloud2->points[inliers->indices[i]];
                }
            }

            tsdf.process(cloud, pose);
            for(int j = 0; j < listBoard.size(); j++)
                for(int k = 0; k < listBoard[j]->chessboardCorners.size(); k++)
                {
                    cv::Point3f p = multMatVec(planeRt[j], listBoard[j]->chessboardCorners[k]);
                    printf("board %f %f %f\n", p.x, p.y, p.z);
                    meshData->addVertex(p, cv::Point3f(1,0,0));
                }
            
            if(GLWin)
                renderer->render();
            cv::imshow("img", resultImg[i]);
            cv::waitKey(100);
        }
    }

    #if 0
    for(int i = 0; i < nbFrames; i++)
    {
        for(int j = 0; j < nbCameras; j++)
        {
            if(!poses[j].empty())
            {
                RgbdFrame frame = frames[j];

                if(GLWin)
                    renderer->pauseRenderer();
                cv::Mat poseD = poses[j+nbCameras];
                cv::Mat poseDInv = poseD.inv();
                cv::Mat poseRGB = poses[j];
                cv::Mat KD = frame.getKMat(frame.depth_intrin);
                cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
                cv::Mat KRGB = cameraK[j];//frame.getKMat(frame.color_intrin);
                cv::Mat firstFramePose = origRT*bestRT.inv();

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::Affine3d pose;
                cv::Mat poseCV = (firstFramePose * poseDInv);
                Eigen::Matrix4d eigenMat;
                cv2eigen(poseCV, eigenMat);
                pose = eigenMat;

                cv::Mat boardToWorld = (firstFramePose*bestRT);
                cv::Mat worldToBoard = boardToWorld.inv();

                editFrameVisualHull(frame, dictionary, board, listMarkerCorners[j], KRGB, poseRGB, KD, poseD,firstFramePose, boardToWorld);

                for(int v = 0; v < frame.depth.rows; v++)
                    for(int u = 0; u < frame.depth.cols; u++)
                    {
                        uint16_t depth_val16 = frame.depth.at<uint16_t>(v, u);
                        if(depth_val16 == 0)
                            continue;
                        float depth_value = depth_val16*frame.scale;
                        cv::Point3f uv(u,v,1);
                        cv::Point3f p = multMatVec(KDInv, uv);
                        //todo : inverse distortion
                        p  *= depth_value;
                        cv::Point3f p2 = multMatVec(poseDInv, p);
                        cv::Point3f p2b = multMatVec(firstFramePose, p2);
                        cv::Point3f p2c = multMatVec(worldToBoard, p2b);
                        cv::Point3f p3 = multMatVec(poseRGB, p2);
                        p3 /= p3.z;
                        cv::Point3f uv2 = multMatVec(KRGB, p3);
                        if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows && p2c.x > minBoard.x && p2c.x < maxBoard.x && p2c.y > minBoard.y && p2c.y < maxBoard.y && p2c.z > 0.01)// && p2b.x*chessPlane[0] + p2b.y*chessPlane[1]+ p2b.z*chessPlane[2] + chessPlane[3] > 0.01)
                        {
                            unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                            meshData->addVertex(p2b, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                            pcl::PointXYZRGBA t;
                            t.r = rgb[0];
                            t.g = rgb[1];
                            t.b = rgb[2];
                            t.a = 255;
                            t.x = p.x;
                            t.y = p.y;
                            t.z = p.z;
                            if (!pcl_isnan (t.z))
                            {
                                cloud->push_back(t);
                                cloud2->push_back(pcl::PointXYZ(p2b.x, p2b.y,p2b.z));
                            }
                        }
                    }

                if(cloud2->size() > 10)
                {
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    // Create the segmentation object
                    pcl::SACSegmentation<pcl::PointXYZ> seg;
                    // Optional
                    seg.setOptimizeCoefficients (true);
                    // Mandatory
                    seg.setModelType (pcl::SACMODEL_PLANE);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setDistanceThreshold (0.01);

                    seg.setInputCloud (cloud2);
                    seg.segment (*inliers, *coefficients);

                    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " "
                                          << coefficients->values[3] << std::endl;

                    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
                      for (size_t i = 0; i < inliers->indices.size (); ++i)
                        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                                   << cloud->points[inliers->indices[i]].y << " "
                                                                   << cloud->points[inliers->indices[i]].z << std::endl;

                    for (size_t i = 0; i < inliers->indices.size (); ++i)
                    {
                        pcl::PointXYZ p = cloud2->points[inliers->indices[i]];
                        //meshData->addVertex(cv::Point3f(p.x, p.y, p.z), cv::Point3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
                    }
                }

                tsdf.process(cloud, pose);
                for(int k = 0; k < board->chessboardCorners.size(); k++)
                {
                    cv::Point3f p = board->chessboardCorners[k];
                    printf("board %f %f %f\n", p.x, p.y, p.z);
                    p = multMatVec(bestRT, p);
                    p = multMatVec(firstFramePose, p);
                    meshData->addVertex(p, cv::Point3f(1,0,0));
                }
                
                if(GLWin)
                    renderer->render();
                cv::imshow("img", resultImg[j]);
                cv::waitKey(100);
            }
        }
    }
    #endif

    featuresDescFull = cv::Mat(featuresDesc.size(), featuresDesc[0].cols, featuresDesc[0].type());
    for(int i = 0; i < featuresDesc.size(); i++)
        featuresDesc[i].copyTo(featuresDescFull(cv::Rect(0,i,featuresDesc[i].cols,1)));
    char filename[255];
    sprintf(filename, "cloud_result/features_%s.yml", seq_folder);
    saveFeatures(filename, featuresPos3d, featuresNormal3d, featuresDescFull);
}

void track6dof(const std::vector<cv::Mat>& listImg, const std::vector<std::vector<cv::Point3f> >& featuresPos3d, const std::vector<std::vector<cv::Point3f> >& featuresNormal3d, const std::vector<cv::Mat>& featuresDesc)
{
    int currentObject = 1;
    std::vector<std::vector<RgbdFrame> > listFrames;

    rs::context rsContext;
    printRSContextInfo( &rsContext );

    std::vector<rs::device*> rsCameras;
    for(int i = 0; i < rsContext.get_device_count(); i++)
    {
        rsCameras.push_back(rsContext.get_device(i));
        configureRSStreams( rsCameras[i], true, false, false);
    }

    for(int camId = 0; camId < rsCameras.size(); camId++)
        rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 0);

    int rotation = 0;
    bool capture = false;
    int nbCaptures = 0;
    int record = 0;
    char name[255];

    std::vector<cv::Mat> cameraMatrix(rsCameras.size()*3);
    std::vector<cv::Mat> cameraDistCoeffs(rsCameras.size()*3);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    std::vector<cv::Mat> K;
    std::vector<cv::Mat> distCoeffs;

    while( true )
    {
        std::vector<RgbdFrame> frames(rsCameras.size());
        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 1);
            for (int i = 0; i < 3; i++)
                rsCameras[camId]->wait_for_frames();
            frames[camId] = (extractDataFromRs(rsCameras[camId], true, false, false));
            rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 0);
        }

        for(int camId = 0; camId < rsCameras.size(); camId++)
            generatePointCloudOrganized2( frames[camId] );

        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            printf("test1\n");
            RgbdFrame frame = frames[camId];
            cv::Mat img = frame.img.clone();
            cv::Mat result = img.clone();

            if(K.size() <= camId)
            {
                K.push_back(getCameraMatrix(frame, 0));
                distCoeffs.push_back(getCameraDistMatrix(frame, img.size(),  0));
            }

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            akaze->detectAndCompute(img, cv::Mat(), keypoints, descriptors);

            std::vector<cv::DMatch> matches;
            std::vector<cv::DMatch> good_matches;
            matcher->match(featuresDesc[currentObject], descriptors, matches);

            double max_dist = 0;
            double min_dist = 100;
            // Quick calculation of max and min distances between keypoints.
            for (int i = 0; i < matches.size(); i++) {
                double dist = matches[i].distance;
                if (dist < min_dist)
                  min_dist = dist;
                if (dist > max_dist)
                  max_dist = dist;
            }

            for (int i = 0; i < matches.size(); i++)
            {
                if (matches[i].distance < 3 * min_dist)
                  good_matches.push_back(matches[i]);
            }

            std::vector<cv::Point3f> obj;
            std::vector<cv::Point3f> normal;
            std::vector<cv::Point2f> scene;

            printf("test2\n");
            for(int i = 0; i < good_matches.size(); i++)
            {
                obj.push_back(featuresPos3d[currentObject][good_matches[i].queryIdx]);
                normal.push_back(featuresNormal3d[currentObject][good_matches[i].queryIdx]);
                scene.push_back(keypoints[good_matches[i].trainIdx].pt);
            }

            printf("findPoseWitrhDepthCue\n");
            cv::Mat mask;
            cv::Mat Pose = findPoseWithDepthCue(obj, normal, scene, frame, mask);
            printf("findPoseWitrhDepthCue finished\n");

            printf("test3\n");

            std::vector<cv::Point> listPoints;

            matches = good_matches;
            good_matches.clear();
            for(int i = 0; i < matches.size(); i++)
                if(mask.at<unsigned char>(i,0) > 0)
                    good_matches.push_back(matches[i]);
            for(int i = 0; i < good_matches.size(); i++)
            {
                listPoints.push_back(keypoints[good_matches[i].trainIdx].pt);
                cv::circle(result, keypoints[good_matches[i].trainIdx].pt, 3, cv::Scalar(255,0,0));
            }
            std::vector<std::vector<cv::Point>> hulls(1);
            cv::convexHull(cv::Mat(listPoints), hulls[0], false);
            cv::drawContours(result, hulls, 0, cv::Scalar(255, 255, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

            cv::Scalar color(0, 255, 0);
            double area = hulls[0].size() >= 3 ? cv::contourArea(hulls[0]) : 0;
            //printf("Area: %lf px, ratio %lf\n", area, area/(frameImg.cols * frameImg.rows));
            if (good_matches.size() < 5 || area < img.cols * img.rows / 400)
            {
              color = cv::Scalar(0, 0, 255);
            }
            else if (good_matches.size() < 10 || area < img.cols * img.rows / 20)
            {
              color = cv::Scalar(0, 255, 255);
            }

            if(!Pose.empty())
            {
                for(int i = 0; i < featuresPos3d[currentObject].size(); i++)
                {
                    cv::Point3f p = multMatVec(Pose, featuresPos3d[currentObject][i]);
                    p /= p.z;
                    cv::Point3f uv = multMatVec(K[camId], p);
                    cv::circle(result, cv::Point(uv.x, uv.y), 3, color);
                }
            }

            cv::imshow("obj", listImg[currentObject]);
            cv::imshow("img", result);
        }

        int key = cv::waitKey(100);
        printf("key %d\n", key);
        if(key == 'n')
            currentObject = (currentObject+1)%featuresPos3d.size();
        else if(key == 'p')
            currentObject = (currentObject+featuresPos3d.size()-1)%featuresPos3d.size();
    }
}

void renderFromRecord(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, const char *seq_folder, int nbCameras, int nbFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& poses, cv::Mat& resultImg0, std::vector<cv::Point3f>& featuresPos3d, std::vector<cv::Point3f>& featuresNormal3d, cv::Mat& featuresDescFull)
{
    bool GLWin = true;
    TSDFIntegration tsdf;

    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    std::shared_ptr<GLRenderer> renderer;
    if(GLWin)
    {
    renderer = renderMngr.createRenderer();
    renderer->cameraPos = cv::Point3f(-0.082839,-0.003967,0.060352);
    renderer->viewTheta = 0.343010;
    renderer->viewPhi = 0.297746;
    }
    std::shared_ptr<GLMeshData> meshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    meshData->type = GLMeshData::GLMeshDataType::Points;
    std::shared_ptr<GLMesh> mesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
    mesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    if(GLWin)
        renderer->addMesh(mesh);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    std::vector<cv::Mat> featuresDesc;

    cv::Mat firstImg;
    cv::Mat origRT, origPose;

    cv::Point3f minBoard = getMinBoard(board);
    cv::Point3f maxBoard = getMaxBoard(board);
    float maxHeight = std::max(maxBoard.x-minBoard.x, maxBoard.y-minBoard.y)/2;
    cv::Point3f bbox[8];
    for(int k = 0; k < 8; k++)
        bbox[k] = cv::Point3f((k%2)>=1?maxBoard.x:minBoard.y,(k%4)>=2?maxBoard.y:minBoard.y,k>=4?maxHeight:0);

    std::vector<RgbdFrame> listFrames;
    std::vector<cv::Mat> listKRGB;
    std::vector<cv::Mat> listKD;
    std::vector<cv::Mat> listPoseRGB;
    std::vector<cv::Mat> listPoseD;
    std::vector<cv::Mat> listBestRT;
    std::vector<cv::Mat> listFirstFramePose;
    std::vector<std::vector<int> > listMarkerIds;
    std::vector<std::vector<std::vector<cv::Point2f> > > listMarkerCorners;
    std::vector<std::vector<cv::Point2f> > listCharucoCorners;
    std::vector<std::vector<int> > listCharucoIds;
    std::vector<cv::Mat> resultImg;

    cv::Mat emptyDist = cv::Mat::zeros(1,5,CV_64F);
    for(int i = 0; i < nbFrames; i++)
    {
        int bestCornersCount  = 0;
        cv::Mat bestRT, bestPose;
        for(int j = 0; j < nbCameras; j++)
        {
            RgbdFrame frame;
            char name[255];
            sprintf(name, "capture%d_cam%d", i, j);
            frame.load(seq_folder, name);
            cv::Mat imageUndistorted;
            undistort(frame.img, imageUndistorted, cameraK[j], cameraDist[j]);
            frame.img = imageUndistorted;

            if(i == 0 && j == 0)
                firstImg = frame.img.clone();

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::Mat rvec, tvec;
            cv::Mat result = frame.img.clone();
            detectMarker(cv::Scalar::all(255) - frame.img, dictionary, board, cameraK[j], emptyDist, markerIds, markerCorners, charucoCorners, charucoIds, rvec, tvec, result);
            listMarkerIds.push_back(markerIds);
            listMarkerCorners.push_back(markerCorners);
            listCharucoCorners.push_back(charucoCorners);
            listCharucoIds.push_back(charucoIds);

            if(charucoCorners.size() > bestCornersCount)
            {
                bestRT = poses[j].inv()*RTVec2Mat(rvec, tvec);
                bestPose = poses[j];
                bestCornersCount = charucoCorners.size();
            }

            listKRGB.push_back(cameraK[j]);
            listPoseRGB.push_back(poses[j]);
            listKD.push_back(frame.getKMat(frame.depth_intrin));//cameraK[j+nbCameras]);
            listPoseD.push_back(frame.getRtMat(frame.depth_to_color).inv()*poses[j]);

            listFrames.push_back(frame);
        }
        for(int j = 0; j < nbCameras; j++)
            listBestRT.push_back(bestRT.clone());

        if(origRT.empty())
        {
            origRT = bestRT;
            origPose = bestPose;
        }
    }

    for(int i = 0; i < listFrames.size(); i++)
    {
        cv::Mat firstFramePose = origRT*listBestRT[i].inv();

        listFirstFramePose.push_back(firstFramePose);
    }

    std::vector<cv::Mat> listMask = editFrameVisualHull(listFrames, dictionary, board, listKRGB, listPoseRGB, listFirstFramePose, origRT);

    for(int i = 0; i < listFrames.size(); i++)
    {
        generatePointCloudOrganized2(listFrames[i]);
        RgbdFrame frame = listFrames[i];

        cv::Mat result = frame.img.clone();

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        akaze->detectAndCompute(frame.img, cv::Mat(), keypoints, descriptors);

        cv::Mat firstFramePose = origRT*listBestRT[i].inv();
        cv::Mat boardToWorld = (firstFramePose*listBestRT[i]);
        cv::Mat worldToBoard = boardToWorld.inv();

        for(int k = 0; k < board->chessboardCorners.size(); k++)
        {
            cv::Point3f p = board->chessboardCorners[k];
            cv::Point3f p2 = multMatVec(boardToWorld, p);
            p2 = multMatVec(listPoseRGB[i]*firstFramePose.inv(), p2);
            p2 /= p2.z;
            cv::Point3f uv = multMatVec(listKRGB[i], p2);
        }

        cv::Mat poseRGBInv = listPoseRGB[i].inv();
        for(int k = 0; k < keypoints.size(); k++)
        {
            cv::Point p = keypoints[k].pt;
            int id = p.y * frame.imgPoints->width + p.x;

            pcl::PointXYZ p2t = frame.imgPoints->points[id];
            pcl::PointXYZ p2tN(p2t.x + frame.imgNormals->points[id].normal_x, p2t.y + frame.imgNormals->points[id].normal_y, p2t.z + frame.imgNormals->points[id].normal_z);

            cv::Point3f p2(p2t.x, p2t.y, p2t.z);
            cv::Point3f p2N(p2tN.x, p2tN.y, p2tN.z);
            cv::Point3f p3 = multMatVec(firstFramePose*poseRGBInv, p2);
            cv::Point3f p3N = multMatVec(firstFramePose*poseRGBInv, p2N);
            cv::Point3f p3c = multMatVec(worldToBoard, p3);
            cv::Point3f p3cN = multMatVec(worldToBoard, p3N);
            if(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z > 0 /*&& p3c.x > minBoard.x && p3c.x < maxBoard.x && p3c.y > minBoard.y && p3c.y < maxBoard.y*/ && p3c.z > 0.01)
            {
                cv::circle(result, p, 3, cv::Scalar(0,0,255));
                meshData->addVertex(p3, cv::Point3f(255,0,0)/255.0);
                featuresPos3d.push_back(p3c);
                featuresNormal3d.push_back(p3cN-p3c);
                featuresDesc.push_back(descriptors(cv::Rect(0,k,descriptors.cols,1)).clone());
            }
        }
        resultImg.push_back(result);
        if(i == 0)
            resultImg0 = result.clone();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i = 0; i < listFrames.size(); i++)
    {
        if(!listPoseRGB[i].empty())
        {
            RgbdFrame frame = listFrames[i];

            if(GLWin)
                renderer->pauseRenderer();

            cv::Mat poseD = listPoseD[i];
            cv::Mat poseDInv = poseD.inv();
            cv::Mat poseRGB = listPoseRGB[i];
            cv::Mat KD = frame.getKMat(frame.depth_intrin);
            cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
            cv::Mat KRGB = listKRGB[i];//frame.getKMat(frame.color_intrin);
            cv::Mat firstFramePose = origRT*listBestRT[i].inv();


            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
            Eigen::Affine3d pose;
            cv::Mat poseCV = (firstFramePose * poseDInv);
            Eigen::Matrix4d eigenMat;
            cv2eigen(poseCV, eigenMat);
            pose = eigenMat;

            cv::Mat boardToWorld = (firstFramePose*listBestRT[i]);
            cv::Mat worldToBoard = boardToWorld.inv();

            for(int v = 0; v < frame.depth.rows; v++)
                for(int u = 0; u < frame.depth.cols; u++)
                {
                    uint16_t depth_val16 = frame.depth.at<uint16_t>(v, u);
                    if(depth_val16 == 0)
                        continue;
                    float depth_value = depth_val16*frame.scale;
                    cv::Point3f uv(u,v,1);
                    cv::Point3f p = multMatVec(KDInv, uv);
                    //todo : inverse distortion
                    p  *= depth_value;
                    cv::Point3f p2 = multMatVec(poseDInv, p);
                    cv::Point3f p2b = multMatVec(firstFramePose, p2);
                    cv::Point3f p2c = multMatVec(worldToBoard, p2b);
                    cv::Point3f p3 = multMatVec(poseRGB, p2);
                    p3 /= p3.z;
                    cv::Point3f uv2 = multMatVec(KRGB, p3);
                    if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows && p2c.x > minBoard.x && p2c.x < maxBoard.x && p2c.y > minBoard.y && p2c.y < maxBoard.y && p2c.z > 0.01)// && p2b.x*chessPlane[0] + p2b.y*chessPlane[1]+ p2b.z*chessPlane[2] + chessPlane[3] > 0.01)
                    {
                        unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                        meshData->addVertex(p2b, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                        pcl::PointXYZRGBA t;
                        t.r = rgb[0];
                        t.g = rgb[1];
                        t.b = rgb[2];
                        t.a = 255;
                        t.x = p.x;
                        t.y = p.y;
                        t.z = p.z;
                        if (!pcl_isnan (t.z))
                        {
                            cloud->push_back(t);
                            cloud2->push_back(pcl::PointXYZ(p2b.x, p2b.y,p2b.z));
                            cloud3->push_back(pcl::PointXYZ(p2b.x, p2b.y,p2b.z));
                        }
                    }
                }

            if(i == listFrames.size()-1)
            {
                std::vector<cv::Point3f> OBB = computeOBB(cloud3);
                for(int k = 0; k < OBB.size(); k++)
                {
                    meshData->addVertex(OBB[k], cv::Point3f(0,1,0));
                }
            }

            if(cloud2->size() > 10)
            {
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients (true);
                // Mandatory
                seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold (0.01);

                seg.setInputCloud (cloud2);
                seg.segment (*inliers, *coefficients);

                std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

                std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
                  for (size_t i = 0; i < inliers->indices.size (); ++i)
                    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                               << cloud->points[inliers->indices[i]].y << " "
                                                               << cloud->points[inliers->indices[i]].z << std::endl;

                for (size_t i = 0; i < inliers->indices.size (); ++i)
                {
                    pcl::PointXYZ p = cloud2->points[inliers->indices[i]];
                }
            }

            tsdf.process(cloud, pose);
            for(int k = 0; k < board->chessboardCorners.size(); k++)
            {
                cv::Point3f p = board->chessboardCorners[k];
                printf("board %f %f %f\n", p.x, p.y, p.z);
                p = multMatVec(listBestRT[i], p);
                p = multMatVec(firstFramePose, p);
                meshData->addVertex(p, cv::Point3f(1,0,0));
            }
            if(GLWin)
                renderer->render();
            cv::imshow("img", resultImg[i]);
            cv::waitKey(100);
        }
    }

    #if 0
    for(int i = 0; i < nbFrames; i++)
    {
        for(int j = 0; j < nbCameras; j++)
        {
            if(!poses[j].empty())
            {
                RgbdFrame frame = frames[j];

                if(GLWin)
                    renderer->pauseRenderer();
                
                cv::Mat poseD = poses[j+nbCameras];
                cv::Mat poseDInv = poseD.inv();
                cv::Mat poseRGB = poses[j];
                cv::Mat KD = frame.getKMat(frame.depth_intrin);
                cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
                cv::Mat KRGB = cameraK[j];//frame.getKMat(frame.color_intrin);
                cv::Mat firstFramePose = origRT*bestRT.inv();

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::Affine3d pose;
                cv::Mat poseCV = (firstFramePose * poseDInv);
                Eigen::Matrix4d eigenMat;
                cv2eigen(poseCV, eigenMat);
                pose = eigenMat;

                cv::Mat boardToWorld = (firstFramePose*bestRT);
                cv::Mat worldToBoard = boardToWorld.inv();

                editFrameVisualHull(frame, dictionary, board, listMarkerCorners[j], KRGB, poseRGB, KD, poseD,firstFramePose, boardToWorld);

                for(int v = 0; v < frame.depth.rows; v++)
                    for(int u = 0; u < frame.depth.cols; u++)
                    {
                        uint16_t depth_val16 = frame.depth.at<uint16_t>(v, u);
                        if(depth_val16 == 0)
                            continue;
                        float depth_value = depth_val16*frame.scale;
                        cv::Point3f uv(u,v,1);
                        cv::Point3f p = multMatVec(KDInv, uv);
                        //todo : inverse distortion
                        p  *= depth_value;
                        cv::Point3f p2 = multMatVec(poseDInv, p);
                        cv::Point3f p2b = multMatVec(firstFramePose, p2);
                        cv::Point3f p2c = multMatVec(worldToBoard, p2b);
                        cv::Point3f p3 = multMatVec(poseRGB, p2);
                        p3 /= p3.z;
                        cv::Point3f uv2 = multMatVec(KRGB, p3);
                        if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows && p2c.x > minBoard.x && p2c.x < maxBoard.x && p2c.y > minBoard.y && p2c.y < maxBoard.y && p2c.z > 0.01)// && p2b.x*chessPlane[0] + p2b.y*chessPlane[1]+ p2b.z*chessPlane[2] + chessPlane[3] > 0.01)
                        {
                            unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                            meshData->addVertex(p2b, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                            pcl::PointXYZRGBA t;
                            t.r = rgb[0];
                            t.g = rgb[1];
                            t.b = rgb[2];
                            t.a = 255;
                            t.x = p.x;
                            t.y = p.y;
                            t.z = p.z;
                            if (!pcl_isnan (t.z))
                            {
                                cloud->push_back(t);
                                cloud2->push_back(pcl::PointXYZ(p2b.x, p2b.y,p2b.z));
                            }
                        }
                    }

                if(cloud2->size() > 10)
                {
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    // Create the segmentation object
                    pcl::SACSegmentation<pcl::PointXYZ> seg;
                    // Optional
                    seg.setOptimizeCoefficients (true);
                    // Mandatory
                    seg.setModelType (pcl::SACMODEL_PLANE);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setDistanceThreshold (0.01);

                    seg.setInputCloud (cloud2);
                    seg.segment (*inliers, *coefficients);

                    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " " 
                                          << coefficients->values[3] << std::endl;

                    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
                      for (size_t i = 0; i < inliers->indices.size (); ++i)
                        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                                   << cloud->points[inliers->indices[i]].y << " "
                                                                   << cloud->points[inliers->indices[i]].z << std::endl;

                    for (size_t i = 0; i < inliers->indices.size (); ++i)
                    {
                        pcl::PointXYZ p = cloud2->points[inliers->indices[i]];
                        //meshData->addVertex(cv::Point3f(p.x, p.y, p.z), cv::Point3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
                    }
                }

                tsdf.process(cloud, pose);
                for(int k = 0; k < board->chessboardCorners.size(); k++)
                {
                    cv::Point3f p = board->chessboardCorners[k];
                    printf("board %f %f %f\n", p.x, p.y, p.z);
                    p = multMatVec(bestRT, p);
                    p = multMatVec(firstFramePose, p);
                    meshData->addVertex(p, cv::Point3f(1,0,0));
                }
                
                if(GLWin)
                    renderer->render();
                cv::imshow("img", resultImg[j]);
                cv::waitKey(100);
            }
        }
    }
    #endif

    featuresDescFull = cv::Mat(featuresDesc.size(), featuresDesc[0].cols, featuresDesc[0].type());
    for(int i = 0; i < featuresDesc.size(); i++)
        featuresDesc[i].copyTo(featuresDescFull(cv::Rect(0,i,featuresDesc[i].cols,1)));
    char filename[255];
    sprintf(filename, "cloud_result/features_%s.yml", seq_folder);
    saveFeatures(filename, featuresPos3d, featuresNormal3d, featuresDescFull);
}

void loadCalibBox(const char *filename, std::vector<cv::Mat>& planeRt)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::FileNode planeRtNode = fs["planeRt"];
    planeRt.resize(planeRtNode.size());
    for(int i = 0; i < planeRt.size(); i++)
    {
        planeRtNode[i] >> planeRt[i];
        printf("%s\n\n", mat2str(planeRt[i]).c_str());
    }
    fs.release();
}

void planeExtractor(const std::vector<std::string>& cameraSerial, const std::vector<RgbdFrame>& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses)
{
    cv::Point3f recognitionCenter(0.13+0.075, 0.08, -0.15);
    cv::Point3f recognitionSize(0.4-0.15+0.1,0.4,0.4);
    cv::Point3f suctionCylinderCenter(0.36, 0.07, -0.16);
    cv::Point3f suctionCylinderMainAxis(1,0,0);
    cv::Point3f suctionCylinderRadiusAxis1(0,1,0);
    cv::Point3f suctionCylinderRadiusAxis2(0,0,1);
    float suctionCylinderHeight = 0.1+0.04;
    float suctionCylinderRadius = 0.03;
    bool GLWin = true;

    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    std::shared_ptr<GLRenderer> renderer;
    if(GLWin)
    {
        renderer = renderMngr.createRenderer();
        renderer->cameraPos = cv::Point3f(-0.082839,-0.003967,0.060352);
        renderer->viewTheta = 0.343010;
        renderer->viewPhi = 0.297746;
    }
    std::shared_ptr<GLMeshData> meshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    meshData->type = GLMeshData::GLMeshDataType::Points;
    std::shared_ptr<GLMesh> mesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
    mesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    if(GLWin)
        renderer->addMesh(mesh);

    std::shared_ptr<GLMeshData> bboxMeshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    bboxMeshData->type = GLMeshData::GLMeshDataType::Lines;
    std::shared_ptr<GLMesh> bboxMesh = std::shared_ptr<GLMesh>(new GLMesh(bboxMeshData));
    bboxMesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    if(GLWin)
        renderer->addMesh(bboxMesh);

    cv::waitKey(1000);

    int nbCameras = listFrames.size();

    int countExtractedImg = 0;

    static const float nan = std::numeric_limits<float>::quiet_NaN( );

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud <pcl::Normal>);
    int count = 0;
    {
        cloudRGB->clear();
        cloudNormal->clear();
        for(int i = 0; i < listFrames.size(); i++)
        {
            RgbdFrame frame = listFrames[i];
            int id = i;
            for(int j = 0; j < cameraSerial.size(); j++)
                if(cameraSerial[j] == frame.camSerial)
                {
                    id = j;
                    break;
                }
            generatePointCloudOrganized2(frame, false, true, false);

            cv::Mat result = frame.img.clone();

            cv::cvtColor(result, result, CV_BGR2RGB);
            char name[255];
            sprintf(name, "img%d", i);
            cv::imshow(name, result);

            cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
            cv::Mat KRGB = cameraK[i];//frame.getKMat(frame.color_intrin);

            cv::Mat mat = cameraPoses[i].inv();
            if(GLWin)
            {
                renderer->pauseRenderer();
                if(i==0)
                    meshData->clear();
            }

            cv::Mat cameraPoseInv = (cameraPoses[id]).inv();
            cv::Mat normalTranformMat = (cameraPoseInv*depthToColor).inv().t();
            for(int v = 0; v < frame.rs_cloud_ptr->height; v++)
                for(int u = 0; u < frame.rs_cloud_ptr->width; u++)
                {
                    int i = v*frame.rs_cloud_ptr->width+u;
                    pcl::PointXYZRGB p = frame.rs_cloud_ptr->points[i];
                    if(std::isnan(p.x))
                        continue;
                    cv::Point3f p2 = multMatVec(depthToColor, pcl2cv(p));
                    cv::Point3f p3 = multMatVec(cameraPoseInv, p2);
                    cv::Point3f uv2 = multMatVec(KRGB, p2/p2.z);

                    if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
                    {
                        if(p3.x < recognitionCenter.x-recognitionSize.x/2 || p3.x > recognitionCenter.x+recognitionSize.x/2
                          || p3.y < recognitionCenter.y-recognitionSize.y/2 || p3.y > recognitionCenter.y+recognitionSize.y/2
                          || p3.z < recognitionCenter.z-recognitionSize.z/2 || p3.z > recognitionCenter.z+recognitionSize.z/2)
                                continue;

                        cv::Point3f cylinderP = p3 - suctionCylinderCenter;
                        double a = cylinderP.dot(suctionCylinderMainAxis);
                        cv::Point3f cylinderProj = suctionCylinderCenter + a*suctionCylinderMainAxis;
                        cv::Point3f cylinderDir = p3-cylinderProj;
                        if(a >= -suctionCylinderHeight/2 && a <= suctionCylinderHeight/2 && sqrt(cylinderDir.dot(cylinderDir) <= suctionCylinderRadius))
                            continue;

                        pcl::Normal n = frame.rs_cloud_normal_ptr->points[i];
                        cv::Point3f n2;
                        if(!std::isnan(n.normal_x))
                        {
                            //n = (p+n)-p
                            //M' n = M (p+n) - M p
                            n2 = multMatVec(normalTranformMat, pcl2cv(n));
                            if(GLWin)
                            {

                                bboxMeshData->addVertex(p3, cv::Point3f(1,0,0));
                                bboxMeshData->addVertex(p3+n2*0.001, cv::Point3f(1,0,0));
                            }
                        }
                        else
                        {
                            n2.x = nan;
                            n2.y = nan;
                            n2.z = nan;
                        }

                        unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                        if(GLWin)
                            meshData->addVertex(p3, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                        pcl::PointXYZRGB point(rgb[0], rgb[1], rgb[2]);
                        point.x = p3.x;
                        point.y = p3.y;
                        point.z = p3.z;
                        cloudRGB->push_back(point);
                     }
                }


        }
        if(GLWin)
            renderer->render();
        cv::waitKey(0);

        if(GLWin)
        {
            renderer->pauseRenderer();
            meshData->clear();
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
          sor.setInputCloud (cloudRGB);
          sor.setMeanK (50);
          sor.setStddevMulThresh (1.0);
          sor.filter (*cloud_filtered);

          // Create the segmentation object for the planar model and set all the parameters
          pcl::SACSegmentation<pcl::PointXYZRGB> seg;
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
          seg.setOptimizeCoefficients (true);
          seg.setModelType (pcl::SACMODEL_PLANE);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setMaxIterations (100);
          seg.setDistanceThreshold (0.02);

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

          int i=0, nr_points = (int) cloud_filtered->points.size ();
            while (cloud_filtered->points.size () > 0.3 * nr_points)
            {
              // Segment the largest planar component from the remaining cloud
              seg.setInputCloud (cloud_filtered);
              seg.segment (*inliers, *coefficients);
              if (inliers->indices.size () == 0)
              {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
              }

              // Extract the planar inliers from the input cloud
              pcl::ExtractIndices<pcl::PointXYZRGB> extract;
              extract.setInputCloud (cloud_filtered);
              extract.setIndices (inliers);
              extract.setNegative (false);

              // Get the points associated with the planar surface
              extract.filter (*cloud_plane);
              std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

              std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                                    << coefficients->values[1] << " "
                                                    << coefficients->values[2] << " "
                                                    << coefficients->values[3] << std::endl;

              if(cloud_plane->points.size () > 10)
              {
                  cv::Point3f n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                  n /= sqrt(n.dot(n));
                  cv::Point3f v1 = n.cross(cv::Point3f(1,0,0));
                  if(v1.dot(v1) < 0.001)
                      v1 = n.cross(cv::Point3f(0,1,0));
                  v1 /= sqrt(v1.dot(v1));
                  cv::Point3f v2 = n.cross(v1);
                  cv::Point3f p0 = pcl2cv(cloud_plane->points[0]);
                  cv::Point2f min(0,0), max(0,0);
                  for(int i = 0; i < cloud_plane->size(); i++)
                  {
                      pcl::PointXYZRGB p = cloud_plane->points[i];
                      cv::Point3f p2 = pcl2cv(p) - p0;
                      cv::Point2f p3(p2.dot(v1), p2.dot(v2));
                      min.x = std::min(p3.x, min.x);
                      min.y = std::min(p3.y, min.y);
                      max.x = std::max(p3.x, max.x);
                      max.y = std::max(p3.y, max.y);
                  }

                  for(int i = 0; i < listFrames.size(); i++)
                  {
                      RgbdFrame frame = listFrames[i];
                      int id = i;
                      for(int j = 0; j < cameraSerial.size(); j++)
                          if(cameraSerial[j] == frame.camSerial)
                          {
                              id = j;
                              break;
                          }

                      cv::Mat img = frame.img.clone();
                      cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
                      cv::Mat KRGB = cameraK[id];

                      cv::Mat cameraPose = (cameraPoses[id]);
                      cv::Mat result = cv::Mat::zeros(cvFloor((max.y-min.y)*1000), cvFloor((max.x-min.x)*1000), CV_8UC3);
                      for(int y = 0; y < result.rows; y++)
                      {
                          for(int x = 0; x < result.cols; x++)
                          {
                              cv::Point2f p(min.x+x*(max.x-min.x)/result.cols, min.y+y*(max.y-min.y)/result.rows);
                              cv::Point3f p3 = p0 + p.x*v1 + p.y*v2;
                              cv::Point3f p2 = multMatVec(cameraPose, p3);
                              cv::Point3f uv2 = multMatVec(KRGB, p2/p2.z);
                              if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
                              {
                                  for(int k = 0; k < 3; k++)
                                    result.ptr<unsigned char>(y)[x*3+k] = frame.img.ptr<unsigned char>((int)cvFloor(uv2.y))[((int)cvFloor(uv2.x))*3+k];
                              }
                          }
                      }
                      char name[255];
                      sprintf(name, "result%d", countExtractedImg++);
                      cv::cvtColor(result, result, CV_BGR2RGB);
                      cv::imshow(name, result);
                  }

                  cv::Point3f color = cv::Point3f(rand()%255, rand()%255, rand()%255)/255;
                  for(int i = 0; i < cloud_plane->size(); i++)
                  {
                      pcl::PointXYZRGB p = cloud_plane->points[i];
                      meshData->addVertex(cv::Point3f(p.x, p.y, p.z), color);
                  }
              }

              // Remove the planar inliers, extract the rest
              extract.setNegative (true);
              extract.filter (*cloud_f);
              *cloud_filtered = *cloud_f;
            }

            for(int i = 0; i < cloud_filtered->size(); i++)
            {
                pcl::PointXYZRGB p = cloud_filtered->points[i];
                meshData->addVertex(cv::Point3f(p.x, p.y, p.z), cv::Point3f(p.r, p.g, p.b)/255);
            }


            if(GLWin)
                renderer->render();
            cv::waitKey(1000);

        #if 0

        pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloudRGB);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*indices);

        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud (cloudRGB);
        reg.setIndices (indices);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (10);
        reg.setPointColorThreshold (6);
        reg.setRegionColorThreshold (5);
        reg.setMinClusterSize (600);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();


        std::vector<pcl::PointCloud <pcl::PointXYZRGB>::Ptr> listPlanes;
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr fullCloud(new pcl::PointCloud <pcl::PointXYZRGB>);
        *fullCloud = *cloudRGB;
        while(true)
        {
            //plane detection
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            seg.setInputCloud (fullCloud);
            seg.segment (*inliers, *coefficients);

            if(inliers->indices.size() == 0)
                break;

            pcl::PointCloud <pcl::PointXYZRGB>::Ptr planeCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr remainderCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
            std::vector<bool> selected(fullCloud->size());
            for(int i = 0; i < selected.size(); i++)
                selected[i] = false;
            for (size_t i = 0; i < inliers->indices.size(); ++i)
                selected[inliers->indices[i]] = true;
            for(int i = 0; i < fullCloud->size(); i++)
            {
                if(selected[i])
                    planeCloud->push_back(fullCloud->points[i]);
                else remainderCloud->push_back(fullCloud->points[i]);
            }
            *fullCloud = *remainderCloud;
            listPlanes.push_back(planeCloud);

        }

        if(GLWin)
        {
            renderer->pauseRenderer();
            meshData->clear();
            for (size_t i = 0; i < listPlanes.size (); ++i)
            {
                cv::Point3f color = cv::Point3f(rand()%255, rand()%255, rand()%255)/255.0;
                for(int j = 0; j < listPlanes[i]->size(); j++)
                {
                    pcl::PointXYZRGB p = listPlanes[i]->points[j];
                    meshData->addVertex(cv::Point3f(p.x, p.y, p.z), color);
                }
            }

            renderer->render();
        }

        #endif

        count++;
    }
    cv::waitKey(0);
}

void handEyeCalibration(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, RgbdFrame handEye)
{
    cv::Mat K = handEye.getKMat(handEye.color_intrin);
    cv::Mat dist = handEye.getDistMat(handEye.color_intrin);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::Mat rvec, tvec;
    cv::Mat result = handEye.img.clone();
    detectMarker(cv::Mat(cv::Scalar::all(255) - handEye.img), dictionary, board, K, dist, markerIds, markerCorners, charucoCorners, charucoIds, rvec, tvec, result);
    cv::Mat Rt = RTVec2Mat(rvec, tvec);
    int refPoint = 18;
    cv::Point3f p = (board->objPoints[refPoint][0]+board->objPoints[refPoint][1]+board->objPoints[refPoint][2]+board->objPoints[refPoint][3])/4;
    cv::Point3f px = p+cv::Point3f(0,1,0);
    cv::Point3f py = p+cv::Point3f(1,0,0);
    p = multMatVec(Rt, p);
    px = multMatVec(Rt, px);
    py = multMatVec(Rt, py);
    cv::Point3f dirX = px-p;
    cv::Point3f dirY = py-p;
    cv::Point3f dirZ = dirX.cross(dirY);
    px = p+0.01*dirX;
    py = p+0.01*dirY;
    cv::Point3f pz = p+0.01*dirZ;
    cv::Point3f uv = multMatVec(K, p/p.z);
    cv::Point3f uv_x = multMatVec(K, px/px.z);
    cv::Point3f uv_y = multMatVec(K, py/py.z);
    cv::Point3f uv_z = multMatVec(K, pz/pz.z);
    cv::circle(result, cv::Point2f(uv.x, uv.y), 3, cv::Scalar(0,255,0));
    cv::circle(result, cv::Point2f(uv_x.x, uv_x.y), 3, cv::Scalar(0,0,255));
    cv::circle(result, cv::Point2f(uv_y.x, uv_y.y), 3, cv::Scalar(255,0,0));
    cv::circle(result, cv::Point2f(uv_z.x, uv_z.y), 3, cv::Scalar(0,255,255));
    printf("\n");
    printf("pos : %f, %f, %f\n", p.x, p.y, p.z);
    printf("dirX : %f, %f, %f\n", dirX.x, dirX.y, dirX.z);
    printf("dirY : %f, %f, %f\n", dirY.x, dirY.y, dirY.z);
    printf("dirZ : %f, %f, %f\n", dirZ.x, dirZ.y, dirZ.z);
    cv::imshow("result", result);
    cv::waitKey(0);
}

void imshowResized(const char *name, cv::Mat img, int factor)
{
    cv::Mat img2;
    cv::resize(img, img2, cv::Size(img.cols/factor, img.rows/factor));
    cv::imshow(name, img2);
}

cv::Mat smoothThreshold(cv::Mat imgGray, int thresh1, int thresh2)
{
    cv::Mat result(imgGray.rows, imgGray.cols, imgGray.type());
    unsigned char diff = thresh2-thresh1;
    for(int i = 0; i < imgGray.rows; i++)
    {
        unsigned char *data = imgGray.ptr<unsigned char>(i);
        unsigned char *out = result.ptr<unsigned char>(i);
        for(int j = 0; j < imgGray.cols; j++)
        {
            unsigned char v = *data;
            if(v < thresh1)
                *out = 0;
            else if(v < thresh2)
                *out = 255*(v-thresh1)/diff;
            else *out = 255;
            out++;
            data++;
        }
    }
    return result;
}

cv::Point2f getMassCenter(cv::Mat imgGray)
{
    cv::Point2f p(0,0);
    float total = 0;
    for(int i = 0; i < imgGray.rows; i++)
    {
        unsigned char *data = imgGray.ptr<unsigned char>(i);
        for(int j = 0; j < imgGray.cols; j++)
        {
            p += data[j]*cv::Point2f(j,i);
            total += data[j];
        }
    }
    return p / total;
}

template<typename T>
cv::Mat sumCols(cv::Mat imgGray)
{
    cv::Mat result(imgGray.rows, 1, CV_32F);
    for(int i = 0; i < imgGray.rows; i++)
    {
        T *data = imgGray.ptr<T>(i);
        float *out = result.ptr<float>(i);
        *out = 0;
        for(int j = 0; j < imgGray.cols; j++)
            *out += data[j];
    }
    return result;
}

template<typename T>
cv::Mat sumRowsT(cv::Mat imgGray)
{
    return sumCols<T>(imgGray.t());
}

template<typename T>
cv::Mat sumRows(cv::Mat imgGray)
{
    return sumRowsT<T>(imgGray).t();
}

template<typename T>
cv::Mat integralCols(cv::Mat imgGray)
{
    cv::Mat result(imgGray.rows, imgGray.cols+1, CV_32F);
    for(int i = 0; i < imgGray.rows; i++)
    {
        T *data = imgGray.ptr<T>(i);
        float *out = result.ptr<float>(i);
        out[0] = 0;
        for(int j = 0; j < imgGray.cols; j++)
            out[j+1] = out[j]+data[j];
    }
    return result;
}

float sumTotal(cv::Mat imgGray)
{
    float total = 0;
    for(int i = 0; i < imgGray.rows; i++)
    {
        unsigned char *data = imgGray.ptr<unsigned char>(i);
        for(int j = 0; j < imgGray.cols; j++)
            total += data[j];
    }
    return total;
}

void getMinRange(cv::Mat totalRows, float total, float minTotal, int& min, int& max)
{
    min = 0;
    max = totalRows.cols-1;
    float width = totalRows.cols;
    for(int j = 0; j < totalRows.cols && total >= minTotal; j++)
    {
        float subTotal = total;
        for(int j2 = totalRows.cols-1; j2 >= j && subTotal >= minTotal; j2--)
        {
            if(j2-j+1 < width)
            {
                min = j;
                width = j2-j+1;
            }
            subTotal -= totalRows.at<float>(0,j2);
        }
        total -= totalRows.at<float>(0,j);
    }
    max = min+width-1;
}

cv::Rect getPercentageBBOX(cv::Mat imgGray, float alpha)
{
    int t0 = clock();
    int t0a = clock();

    cv::Mat totalCols = sumCols<unsigned char>(imgGray);
    int t0b = clock();
    cv::Mat totalRowsT = sumRowsT<unsigned char>(imgGray);
    int t0c = clock();
    float total = sumCols<float>(totalRowsT.t()).at<float>(0,0);
    int t0d = clock();

    float minTotal = alpha*total;

    cv::Rect result(0, 0, imgGray.cols, imgGray.rows);

    int min, max;
    int min1, max1;

    int t1 = clock();

    getMinRange(totalCols.t(), total, minTotal, min, max);

    result.y = min;
    result.height = max-min+1;

    getMinRange(totalRowsT.t(), total, minTotal, min, max);
    result.x = min;
    result.width = max-min+1;

    int t2 = clock();
    printf("%d %d\n", t1-t0, t2-t1);
    printf("%d %d %d %d\n", t0a-t0, t0b-t0a, t0c-t0b, t0d-t0c);
    return result;
}

void recognitionSpaceConfigurator(const char *filename, const std::vector<RgbdFrame>& listFrames, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses)
{
    cv::Point3f recognitionCenter(0.125-0.01, 0.15, -0.11);
    cv::Point3f recognitionSize(0.4-0.15+0.1-0.02,0.4,0.4);
    cv::Point3f suctionCylinderCenter(0.32, 0.17, -0.11);
    cv::Point3f suctionCylinderMainAxis(1,0,0);
    float suctionCylinderHeight = 0.1+0.04;
    float suctionCylinderRadius = 0.03;
    cv::Point3f recoMoveDir(0.01,0,0);
    int editMode = 0;

    loadRecoSpaceConfig(filename, recognitionCenter, recognitionSize, suctionCylinderCenter, suctionCylinderMainAxis, suctionCylinderHeight, suctionCylinderRadius);

    #ifdef USE_GL
    bool GLWin = true;
    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    std::shared_ptr<GLRenderer> renderer;
    std::shared_ptr<GLMeshData> meshData, bboxMeshData;
    std::shared_ptr<GLMesh> mesh, bboxMesh;
    if(GLWin)
    {
        renderer = renderMngr.createRenderer();
        renderer->cameraPos = cv::Point3f(0.7944,-0.1747,-0.1313);
        renderer->viewTheta = 4.69896;
        renderer->viewPhi = 0.443581;
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

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud <pcl::PointXYZRGB>);

    for(int i = 0; i < listFrames.size(); i++)
    {
        RgbdFrame frame = listFrames[i];

        if(frame.rs_cloud_ptr->empty())
            generatePointCloudOrganized2(frame, false, false, false);

        int id = getIdBySerial(serialId, frame.camSerial);
        if(id == -1)
        {
            printf("ERROR, cam serialID %s not found\n", frame.camSerial.c_str());
            return ;
        }

        cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
        cv::Mat KRGB = cameraK[id];
        cv::Mat cameraPoseInv = (cameraPoses[id]).inv();

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
                cv::Point3f p3 = multMatVecT<double, 3, 4>(cameraPoseInv, p2);
                cv::Point2f uv2(cx*p2.x/p2.z + tx, cy*p2.y/p2.z + ty);

                if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
                {
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

    renderer->render();

    while(true)
    {
        renderer->pauseRenderer();
        bboxMeshData->clear();
        drawLine(bboxMeshData, cv::Point3f(0,0,0), cv::Point3f(0.1,0,0), cv::Point3f(1,0,0));
        drawLine(bboxMeshData, cv::Point3f(0,0,0), cv::Point3f(0,0.1,0), cv::Point3f(0,1,0));
        drawLine(bboxMeshData, cv::Point3f(0,0,0), cv::Point3f(0,0,0.1), cv::Point3f(0,0,1));
        if(recoMoveDir.x > 0)
            drawCone(bboxMeshData, cv::Point3f(0.1,0,0), cv::Point3f(0.11,0,0), 0.005, cv::Point3f(1,0,0));
        if(recoMoveDir.y > 0)
            drawCone(bboxMeshData, cv::Point3f(0,0.1,0), cv::Point3f(0, 0.11,0), 0.005, cv::Point3f(0,1,0));
        if(recoMoveDir.z > 0)
            drawCone(bboxMeshData, cv::Point3f(0,0,0.1), cv::Point3f(0,0,0.11), 0.005, cv::Point3f(0,0,1));

        drawCylinder(bboxMeshData, suctionCylinderCenter-suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderCenter+suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderRadius, cv::Point3f(0,0,1));
        ::drawBox(bboxMeshData, getBoxPoints(recognitionCenter, recognitionSize), cv::Point3f(0,0,1));
        for(int i = 0; i < listFrames.size(); i++)
        {
            RgbdFrame frame = listFrames[i];
            cv::Mat result = frame.img.clone();

            char name[255];
            sprintf(name, "img %s", frame.camSerial.c_str());
            cv::imshow(name, result);
        }
        renderer->render();
        int key = cv::waitKey(0);

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
        saveRecoSpaceConfig(filename, recognitionCenter, recognitionSize, suctionCylinderCenter, suctionCylinderMainAxis, suctionCylinderHeight, suctionCylinderRadius);
    }
}

bool hasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending) && 0 != fullString.compare(0, 1, "."));
  else return false;
}

std::vector<std::string> getDirFiles(std::string dir, std::string fileType = "")
{
    std::vector<std::string> files;
    DIR *dp;

    struct dirent *dirp;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
        printf("can't open folder %s\n", dir.c_str());
        return files;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        if (hasEnding(dirp->d_name, fileType) || fileType.compare("") == 0)
            files.push_back(string(dirp->d_name));
    }

    closedir(dp);

    return files;

}

std::vector<std::string> getSubDir(const string& baseFolder)
{
  std::vector<std::string> subFolders;
  if (access(baseFolder.c_str(), F_OK) == -1)
  {
      printf("can't open folder %s\n", baseFolder.c_str());
      return subFolders;
  }

  // iterate over all subfolders where the item's background removed items are located

  DIR* pdir = opendir(baseFolder.c_str());
  struct dirent* entry = readdir(pdir);

  while (entry != NULL)
  {
    if (entry->d_type == DT_DIR && strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
      subFolders.push_back(std::string(entry->d_name));

    entry = readdir(pdir); // iterate;
  }
  closedir(pdir);

  return subFolders;
}

int getCaptureItems(const std::string& baseFolder, std::vector<std::string>& subFolders, std::vector<int>& firstFrame, std::vector<int>& nbFrames)
{
    int itemCount = 0;
    subFolders = getSubDir(baseFolder);
    for(int i = 0; i < subFolders.size(); i++)
    {
        std::vector<std::string> files = getDirFiles(baseFolder+"/"+subFolders[i], ".yml");
        bool init = false;
        int minId = 0, maxId = 0;
        for(int j = 0; j < files.size(); j++)
        {
            if(files[j].size() > subFolders[i].size() && !files[j].compare(0, subFolders[i].size(), subFolders[i]))
            {
                std::string cropped_filename = files[j].substr(subFolders[i].size());
                int index;
                sscanf(cropped_filename.c_str(), "%d", &index);
                printf("%s : %d\n", cropped_filename.c_str(), index);
                if(!init)
                {
                    minId = index;
                    maxId = index;
                    init = true;
                }
                else
                {
                    minId = std::min(minId, index);
                    maxId = std::max(maxId, index);
                }
            }
        }
        if(!init)
        {
            subFolders.erase(subFolders.begin()+i);
            i--;
        }
        else
        {
            firstFrame.push_back(minId);
            nbFrames.push_back(maxId-minId+1);
            itemCount += maxId-minId+1;
        }
    }
    return itemCount;
}

int main(int argc, char* argv[])
{
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> boardTop;
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1;
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2;
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1;
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2;
    cv::Ptr<cv::aruco::CharucoBoard> additionalBoard;
    float squareLength = (0.138/4);
    float markerLength = squareLength*0.025/0.035;
    printf("multiboard square %f marker %f\n", squareLength, markerLength);

    createArucoData(squareLength, markerLength, dictionary, boardTop, boardSmallSide1, boardSmallSide2, boardLongSide1, boardLongSide2, additionalBoard, true);

    std::vector<cv::Ptr<cv::aruco::CharucoBoard> > listBoard;
    listBoard.push_back(boardTop);
    listBoard.push_back(boardSmallSide1);
    listBoard.push_back(boardSmallSide2);
    listBoard.push_back(boardLongSide1);
    listBoard.push_back(boardLongSide2);

    std::vector<cv::Mat> cameraK, cameraDist, planeRt, cameraPoses, newCameraPoses, objectRt;
    std::vector<std::string> cameraSerial;

    loadCalib("calibMulti2.yml", cameraSerial, cameraK, cameraDist, newCameraPoses, planeRt);

    std::string baseFolder = "2017.07.21 competition_test_item_images_1500614454";
    std::vector<std::string> subFolders;
    std::vector<int> firstFrame, nbFrames;
    int nbItems = getCaptureItems(baseFolder, subFolders, firstFrame, nbFrames);

    BBoxExtractor bboxExtractor;
    bboxExtractor.init(cameraSerial, cameraK, cameraDist, newCameraPoses, dictionary, additionalBoard->ids[0], additionalBoard->ids[additionalBoard->ids.size()-1] );
    bboxExtractor.loadConfig("recoSpace2.yml");
    std::vector<std::vector<RgbdFrame> > backgroundFrames;
    for(int i = 1; i <= 13; i++)
    {
        backgroundFrames.push_back(loadFramesId((baseFolder+"/background_images_1500610774").c_str(), cameraSerial.size(), i));
    }

    char filename[255];
    sprintf(filename, "%s/masked_pictures/", baseFolder.c_str());
    mkdir(filename, 775);

    std::vector<cv::Point3f> listSize;
    for(int i = 0; i < subFolders.size(); i++)
    {
        if(subFolders[i] == "calibration" || subFolders[i] == "background_images_1500610774")
            continue;
        for(int ii = firstFrame[i]; ii < firstFrame[i]+nbFrames[i]; ii++)
        {
            printf("%s : %s%d\n", (baseFolder+"/"+subFolders[i]).c_str(), subFolders[i].c_str(), ii);
            std::vector<RgbdFrame> frames = loadFramesId((baseFolder+"/"+subFolders[i]).c_str(), cameraSerial.size()  , ii, subFolders[i].c_str());
            for(int j = 0; j < frames.size(); j++)
                generatePointCloudOrganized2(frames[j], false, false, false);
            int t0 = clock();
            bboxExtractor.compute(frames);
            int t1 = clock();
            printf("compute time %d\n", t1-t0);
            if(bboxExtractor._OBB.size() > 0)
            {
                std::vector<cv::Mat> result = recognitionSpaceBackgroundSubtractor(bboxExtractor, backgroundFrames, frames, cameraSerial, cameraK, cameraDist, newCameraPoses, false);
                for(int j = 0; j < result.size(); j++)
                {
                    char filename[255];
                    sprintf(filename, "%s/masked_pictures/%s", baseFolder.c_str(), subFolders[i].c_str());
                    mkdir(filename, 775);
                    sprintf(filename, "%s/masked_pictures/%s/%s%d_cam%d_masked.png", baseFolder.c_str(), subFolders[i].c_str(), subFolders[i].c_str(), ii, j);
                    cv::imwrite(filename, result[j]);
                }

                cv::Point3f center = extractCenterFromOBB(bboxExtractor._OBB);
                cv::Point3f size = extractSizeFromOBB(bboxExtractor._OBB);
                while(i >= listSize.size())
                    listSize.push_back(cv::Point3f(0,0,0));
                listSize[i] = size;
                cv::Mat rot = extractRotationFromOBB(bboxExtractor._OBB);
                cv::Mat R = getRVecFromMat(rot);
                printf("center %f %f %f\n", center.x, center.y, center.z);
                printf("size %f %f %f\n", size.x, size.y, size.z);
                printf("rotation %lf %lf %lf\n", R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2));
            }
            else
            {
                while(i >= listSize.size())
                    listSize.push_back(cv::Point3f(0,0,0));
            }

            for(int j = 0; j < frames.size(); j++)
            {
                cv::Mat result = bboxExtractor.generateView(frames[j], bboxExtractor._recoBBOX, bboxExtractor._OBB, bboxExtractor._AABB);
                char name[255];
                sprintf(name, "result_%d_%d.png", i, j);
                cv::imwrite(name, result);
            }

            cv::waitKey(100);
        }
    }
    printf("finished\n");

    return 0;

    int camId = -1;
    if(argc > 1)
    {
        sscanf(argv[1], "%d", &camId);
    }
    cArduino arduino(ArduinoBaundRate::B9600bps, "/dev/ttyACM0");
    if(!arduino.isOpen())
    {
        cerr<<"can't open arduino"<<endl;
        return 1;
    }

    cout<<"arduino open at "<<arduino.getDeviceName()<<endl;
    arduino.write("0\n");

    const char *listFolders[] = 
    {
        "scan1_bee",
        "scan1_crayola2",
        "scan1_kleenex1",
        "scan1_platespie",
        "scan1_reynold",
        "scan1_robotsdvd",
        "scan1_salt1",
        "scan1_sponge",
        "scan1_tablecover",
        "scan1_windex"
    };

    int nbImg[] = 
    {
        4,
        4,
        4,
        4,
        4,
        4,
        2,
        4,
        4,
        4
    };

    //IRanalyser(-1);
    {
        bool captureDepth = true;
        float squareLength = 0.05145;
        float markerLength = 0.03657;
        cv::Ptr<cv::aruco::CharucoBoard> boardScan = cv::aruco::CharucoBoard::create(5,8,squareLength, markerLength, dictionary);
        float ratio = (0.157/4)/squareLength;
        squareLength *= ratio;
        markerLength *= ratio;
        printf("calib square %f marker %f\n", squareLength, markerLength);
        cv::Ptr<cv::aruco::CharucoBoard> boardCalib = cv::aruco::CharucoBoard::create(5,8,squareLength, markerLength, dictionary);
        manualRecord(arduino, dictionary, captureDepth?boardScan:boardCalib, captureDepth, false, -1);
    }

    return 0;

#if 0
    printf("start0\n");
    GLRendererMngr& renderMngr = GLRendererMngr::GetInstance();
    printf("start1\n");
    std::shared_ptr<GLRenderer> renderer = renderMngr.createRenderer();
    renderer->cameraPos = cv::Point3f(-0.082839,-0.003967,0.060352);
    renderer->viewTheta = 0.343010;
    renderer->viewPhi = 0.297746;
    printf("start2\n");
    std::shared_ptr<GLMeshData> meshData = std::shared_ptr<GLMeshData>(new GLMeshData());
    meshData->type = GLMeshData::GLMeshDataType::Points;
    printf("start3\n");
    std::shared_ptr<GLMesh> mesh = std::shared_ptr<GLMesh>(new GLMesh(meshData));
    mesh->setShader(renderMngr.getSimpleShaderWithVertexColor());
    printf("start4\n");
    renderer->addMesh(mesh);

    std::vector<cv::Mat> cameraPose;

    std::vector<std::vector<RgbdFrame> > listFrames;

    cv::Mat boardImg;
    int scale = 4;
    board->draw(cv::Size(scale*210,scale*297), boardImg);
    cv::imwrite("charuco_board.png", boardImg);
    cv::imwrite("charuco_boardInv.png", cv::Scalar::all(255) - boardImg);
    cv::Ptr<cv::aruco::CharucoBoard> boardA3 = cv::aruco::CharucoBoard::create(5,8,0.035*sqrt(2), 0.025*sqrt(2), dictionary);
    cv::Mat boardImgA3;
    board->draw(cv::Size(scale*297,scale*210*2), boardImgA3);
    cv::imwrite("charuco_boardA3.png", boardImgA3);
    cv::imwrite("charuco_boardA3Inv.png", cv::Scalar::all(255) - boardImgA3);
    for(int i = 0; i < board->objPoints.size(); i++)
    {
        for(int j = 0; j < board->objPoints[i].size(); j++)
        {
            printf("%f,%f,%f\n", board->objPoints[i][j].x,board->objPoints[i][j].y, board->objPoints[i][j].z);
        }
        printf("\n");
    }

    // Create the RS context and display info about it
    rs::context rsContext;
    printRSContextInfo( &rsContext );

    std::vector<rs::device*> rsCameras;
    for(int i = 0; i < rsContext.get_device_count(); i++)
    {
        rsCameras.push_back(rsContext.get_device(i));
        if(cameraPose.size() > i && !cameraPose[i].empty())
            configureRSStreams( rsCameras[i], true, false, false);
        else configureRSStreams( rsCameras[i], false, true, true);
    }

    std::vector<cv::Mat> calibImg;
    std::vector< std::vector<cv::Point2f> > allCharucoCorners;
    std::vector< std::vector<int> > allCharucoIds;

    std::vector<std::vector<std::vector<cv::Point2f> > > detections;//dectection[frame][cameraId][pointId]
    std::vector<std::vector<cv::Mat> > capturedImg;
    std::vector<cv::Mat> listCameraMatrix(rsCameras.size());
    std::vector<cv::Mat> listCameraDist(rsCameras.size());
    std::vector<std::vector<cv::Mat> > poseR;//poseR[frame][cameraId]
    std::vector<std::vector<cv::Mat> > poseT;//poseT[frame][cameraId]

    while(cameraPose.size() < rsCameras.size())
        cameraPose.push_back(cv::Mat());

    int nbCaptures = 0;

    int count = 0;
    bool capture = false;
    bool rotating = true;

    int nbCharucoIds = board->chessboardCorners.size();
    bool first = true;
    int frameId = 0;
    int lastFrameUpdateCloud = -1000;
    for(int camId = 0; camId < rsCameras.size(); camId++)
        if(!cameraPose[camId].empty())
            rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 1);

    while( true )
    {
        if(rotating)
        {
            arduino.write("1\n");
            cv::waitKey(1000);
            arduino.write("0\n");
            cv::waitKey(500);
        }
        frameId++;
        //viewer.spinOnce ();
        std::vector<RgbdFrame> frames(rsCameras.size());
        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            if(!cameraPose[camId].empty())
            {
                rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 1);
                for (int i = 0; i < 3; i++)
                    rsCameras[camId]->wait_for_frames();
                frames[camId] = (extractDataFromRs(rsCameras[camId], true, false, false));
                rsCameras[camId]->set_option(rs::option::r200_emitter_enabled, 0);
            }
            else frames[camId] = (extractDataFromRs(rsCameras[camId], false, true, true));
        }

        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            if(!cameraPose[camId].empty())
                generatePointCloudOrganized2( frames[camId] );
            if(capture)
                listFrames[listFrames.size()-1][camId] = frames[camId];
        }

        renderer->pauseRenderer();
        meshData->clear();

        for(int camId = 0; camId < rsCameras.size(); camId++)
        {
            RgbdFrame frame = frames[camId];
            cv::Mat invImg = cv::Scalar::all(255) - frame.img;
            cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
            cv::Mat distCoeffs(1,5,CV_64F);

            char name[255];
            sprintf(name, "cam%d", camId);
            cv::imshow(name, frame.img);
            cameraMatrix.at<double>(0,0) = frame.color_intrin.fx;
            cameraMatrix.at<double>(0,2) = frame.color_intrin.ppx;
            cameraMatrix.at<double>(1,1) = frame.color_intrin.fy;
            cameraMatrix.at<double>(1,2) = frame.color_intrin.ppy;
            for(int i = 0; i < 5; i++)
                distCoeffs.at<double>(0,i) = frame.color_intrin.coeffs[i];

            listCameraMatrix[camId] = cameraMatrix.clone();
            listCameraDist[camId] = distCoeffs.clone();

            if(!cameraPose[camId].empty() && frameId%rsCameras.size() == camId)
            {                
                cv::Mat mat = cameraPose[camId].inv();
                for(int i = 0; i < frame.imgPoints->height; i++)
                    for(int j = 0; j < frame.imgPoints->width; j++)
                    {
                        int id = i*frame.imgPoints->width+j;
                        pcl::PointXYZ p = frame.imgPoints->points[id];
                        cv::Point3f pb(p.x,p.y,p.z);
                        cv::Point3f p2 = multMatVec(mat, pb);
                        cv::Point3f p2col(frame.img.ptr<unsigned char>(i)[j*3], frame.img.ptr<unsigned char>(i)[j*3+1], frame.img.ptr<unsigned char>(i)[j*3+2]);
                        p2col /= 255;
                        meshData->addVertex(p2, p2col);
                    }
            }

            std::vector< int > markerIds;
            std::vector< std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(invImg, dictionary, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::Mat result = frame.img.clone();
            if(capture)
            {
                capturedImg[nbCaptures-1][camId] = result.clone();
                char name[255];
                sprintf(name, "capture%d_cam%d", nbCaptures-1, camId);
                frame.save("capture", name);
                printf("capture\n");
            }
            if(markerIds.size() > 0)
            {
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, invImg, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                for(int i = 0; i < charucoCorners.size(); i++)
                    cv::circle(result, charucoCorners[i], 4, cv::Scalar(255,0,0), 2);

                if(capture)
                {
                    detections[nbCaptures-1][camId] = std::vector<cv::Point2f>(nbCharucoIds);
                    for(int i = 0; i < detections[nbCaptures-1][camId].size(); i++)
                        detections[nbCaptures-1][camId][i] = cv::Point2f(-1, -1);
                    for(int i = 0; i < charucoIds.size(); i++)
                        detections[nbCaptures-1][camId][charucoIds[i]] = charucoCorners[i];
                }

                if(!cameraMatrix.empty())
                {
                     cv::Mat rvec, tvec;
                     bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                     if(valid)
                     {
                         if(capture)
                         {
                            poseR[nbCaptures-1][camId] = rvec.clone();
                            poseT[nbCaptures-1][camId] = tvec.clone();
                         }
                         cv::aruco::drawAxis(result, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                         cv::Mat R;
                         cv::Rodrigues(rvec, R);
                         //R = R.t();
                         //tvec = -R*tvec;
                         cv::Mat Rmat = cv::Mat::eye(4,4,CV_64F);
                         cv::Mat Tmat = cv::Mat::eye(4,4,CV_64F);
                         R.copyTo(Rmat(cv::Rect(0,0,3,3)));
                         tvec.copyTo(Tmat(cv::Rect(3,0,1,3)));

                         static const float nan = std::numeric_limits<float>::quiet_NaN( );
                     }
                }

                cv::imshow(name, result);
            }
            sprintf(name, "ir%d", camId);
            cv::imshow(name, frame.imgIR);
        }
        if(capture)
        {
            int w = 2;
            int h = rsCameras.size()/w+1;
            int maxCols = 0, maxRows = 0;
            for(int i = 0; i < capturedImg[nbCaptures-1].size(); i++)
            {
                maxCols = std::max(maxCols, capturedImg[nbCaptures-1][i].cols);
                maxRows = std::max(maxRows, capturedImg[nbCaptures-1][i].rows);
            }
            cv::Mat matchingImg = cv::Mat::zeros(h*maxRows, w*maxCols, CV_8UC3);
            for(int i = 0; i < capturedImg[nbCaptures-1].size(); i++)
            {
                capturedImg[nbCaptures-1][i].copyTo(matchingImg(cv::Rect((i%w)*maxCols, (i/w)*maxRows, capturedImg[nbCaptures-1][i].cols, capturedImg[nbCaptures-1][i].rows)));
            }
            for(int i = 0; i < detections[nbCaptures-1].size(); i++)
            {
                for(int i2 = i+1; i2 < detections[nbCaptures-1].size(); i2++)
                {
                    for(int j = 0; j < detections[nbCaptures-1][i].size() && j < detections[nbCaptures-1][i2].size(); j++)
                    {
                        if(detections[nbCaptures-1][i][j].x >= 0 && detections[nbCaptures-1][i2][j].x >= 0)
                        {
                            cv::Point2f p1 = detections[nbCaptures-1][i][j] + cv::Point2f((i%w)*maxCols, (i/w)*maxRows);
                            cv::Point2f p2 = detections[nbCaptures-1][i2][j] + cv::Point2f((i2%w)*maxCols, (i2/w)*maxRows);
                            cv::line(matchingImg, p1, p2, cv::Scalar(rand()%255,rand()%255,rand()%255), 2);
                        }
                    }
                }
            }
            cv::imshow("matching", matchingImg);
            capture = false;
        }
        renderer->render();

        int key = cv::waitKey(100);
        if(key == 's')
        {
            capture = true;
            detections.push_back(std::vector<std::vector<cv::Point2f> >(rsCameras.size()));
            capturedImg.push_back(std::vector<cv::Mat>(rsCameras.size()));
            poseR.push_back(std::vector<cv::Mat>(rsCameras.size()));
            poseT.push_back(std::vector<cv::Mat>(rsCameras.size()));
            listFrames.push_back(std::vector<RgbdFrame>(rsCameras.size()));
            nbCaptures++;
        }
        else if(key == 'c')
        {
            //calibrate(dictionary, board, listFrames, cameraPose);
        }
    }
#endif
    return (0);
}
