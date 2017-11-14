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

#include <opencv2/opencv.hpp>
#include "Calibration.h"
#include "UtilCvPclRs.h"
#include "GLRendererDrawingPrimitive.h"
#include "EndEffectorQRDetector.h"

#define USE_GL

cv::Ptr<cv::aruco::CharucoBoard> createCustomCharucoBoard(int squaresX, int squaresY, float squareLength,float markerLength, const cv::Ptr<cv::aruco::Dictionary> &dictionary, int firstId = 0)
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    for(int i = 0; i < board->ids.size(); i++)
        board->ids[i] += firstId;
    return board;
}

void recognitionSpaceConfigurator(const char *filename, const std::vector<RgbdFrame>& listFrames, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, cv::Ptr<cv::aruco::Dictionary> dictionary, int minId, int maxId)
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

    std::vector<cv::Point3f> camOrig(listFrames.size());
    std::vector<std::vector<std::vector<cv::Point3f> > > qrCodeVec(listFrames.size());
    std::vector<std::vector<cv::Point3f> > qrCodePoints;
    std::vector<std::pair<cv::Point3f, cv::Point3f> > cylinderIntersections;

    float qrCylinderHeight = 0;
    bool qrCylinderFound = findEndEffectorQRPosX(suctionCylinderCenter, suctionCylinderHeight, suctionCylinderRadius, listFrames, serialId, cameraK, cameraDist, cameraPoses, dictionary, minId, maxId, camOrig, cylinderIntersections, qrCodeVec, qrCodePoints, qrCylinderHeight);

    int maxFoundId = 0;

    for(int i = 0; i < listFrames.size(); i++)
    {
        RgbdFrame frame = listFrames[i];
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

    #ifdef USE_GL
    renderer->render();
    #endif

    while(true)
    {
        #ifdef USE_GL
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

        for(int i = 0; i < cylinderIntersections.size(); i++)
                drawLine(bboxMeshData, cylinderIntersections[i].first, cylinderIntersections[i].second, cv::Point3f(1,0,0));

        for(int i = 0; i < camOrig.size(); i++)
        {   
            for(int j = 0; j < qrCodeVec[i].size(); j++)
                for(int k = 0; k < qrCodeVec[i][j].size(); k++)
                {
                    drawLine(bboxMeshData, camOrig[i], camOrig[i]+qrCodeVec[i][j][k], cv::Point3f(0,1,0));
                }
        }
        for(int i = 0; i < qrCodePoints.size(); i++)
        {
            for(int j = 0; j < qrCodePoints[i].size(); j++)
            {
                int j2 = (j-(j%4)) + ((j+1)%4);
                drawLine(bboxMeshData, qrCodePoints[i][j], qrCodePoints[i][j2], cv::Point3f(0,1,0));
            }
        }

        drawCylinder(bboxMeshData, suctionCylinderCenter-suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderCenter+suctionCylinderMainAxis*suctionCylinderHeight/2, suctionCylinderRadius, cv::Point3f(0,0,1));
        if(qrCylinderFound)
            drawCylinder(bboxMeshData, cv::Point3f(qrCylinderHeight-0.01, suctionCylinderCenter.y, suctionCylinderCenter.z), cv::Point3f(qrCylinderHeight+0.01, suctionCylinderCenter.y, suctionCylinderCenter.z), suctionCylinderRadius, cv::Point3f(1,0,0));
        ::drawBox(bboxMeshData, getBoxPoints(recognitionCenter, recognitionSize), cv::Point3f(0,0,1));
        #endif
        for(int i = 0; i < listFrames.size(); i++)
        {
            RgbdFrame frame = listFrames[i];
            int id = getIdBySerial(serialId, frame.camSerial);
            cv::Mat result = frame.img.clone();
            cv::cvtColor(result, result, CV_BGR2RGB);

            for(int i2 = 0; i2 < qrCodePoints.size(); i2++)
            {
                for(int j = 0; j < qrCodePoints[i2].size(); j++)
                {
                    int j2 = (j-(j%4)) + ((j+1)%4);
                    std::vector<cv::Point3f> line = {qrCodePoints[i2][j], qrCodePoints[i2][j2]};
                    std::vector<cv::Point2f> line2d = project2d(cameraK[id], cameraPoses[id], line);
                    cv::line(result, line2d[0], line2d[1], cv::Scalar(0,255,0));
                }
            }


            if(qrCylinderFound)
                drawCylinder(result, cameraK[id], cameraPoses[id], cv::Point3f(qrCylinderHeight-0.025, suctionCylinderCenter.y, suctionCylinderCenter.z), cv::Point3f(qrCylinderHeight+0.1, suctionCylinderCenter.y, suctionCylinderCenter.z), suctionCylinderRadius, cv::Scalar(255,0,0));
            else drawCylinder(result, cameraK[id], cameraPoses[id], cv::Point3f(suctionCylinderCenter.x-suctionCylinderHeight/2, suctionCylinderCenter.y, suctionCylinderCenter.z), cv::Point3f(suctionCylinderCenter.x+suctionCylinderHeight/2, suctionCylinderCenter.y, suctionCylinderCenter.z), suctionCylinderRadius, cv::Scalar(0,0,255));

            char name[255];
            sprintf(name, "img %s", frame.camSerial.c_str());

            
            cv::imshow(name, result);
        }
        #ifdef USE_GL
        renderer->render();
        #endif
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

int main(int argc, char *argv[])
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    int squareX = 5, squareY = 8;
    float squareLength = 0.035*sqrt(2);
    float markerLength = 0.025*sqrt(2);
    float ratio = (0.138/4)/squareLength;
    squareLength *= ratio;
    markerLength *= ratio;
    printf("multiboard square %f marker %f\n", squareLength, markerLength);
    cv::Ptr<cv::aruco::CharucoBoard> boardTop = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, 0);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardTop->ids[boardTop->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide1->ids[boardSmallSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide2->ids[boardSmallSide2->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide1->ids[boardLongSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> additionalBoard = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide2->ids[boardLongSide2->ids.size()-1] + 1);
    

    if(argc < 6)
    {
        printf("usage : RecognitionSpaceConfiguration capture_folder nbCamera idFrame calibration_file output_configuration_file (prefix)\n");
        return 0;
    }
    std::string prefix = "capture";
    int idFrame, nbCameras;
    sscanf(argv[2], "%d", &nbCameras);
    sscanf(argv[3], "%d", &idFrame);
    printf("RecognitionSpaceCalibration capture_folder:%s nbCamera:%d idFrame:%d calibration_file:%s output_configuration_file:%s", argv[1], nbCameras, idFrame, argv[4], argv[5]);
    if(argc >= 7)
        prefix = argv[6];
    printf("\n");

    std::vector<cv::Mat> cameraK, cameraDist, planeRt, cameraPoses, newCameraPoses, objectRt;
    std::vector<std::string> cameraSerial;

    loadCalib(argv[4], cameraSerial, cameraK, cameraDist, newCameraPoses, planeRt);

    std::vector<RgbdFrame> listFrames = loadFramesId(argv[1], nbCameras, idFrame, prefix.c_str());

    recognitionSpaceConfigurator(argv[5], listFrames, cameraSerial, cameraK, cameraDist, newCameraPoses, dictionary, additionalBoard->ids[0], additionalBoard->ids[additionalBoard->ids.size()-1]);

    return 0;
}
