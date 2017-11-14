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

#include "Calibration.h"
#include "CeresModels.h"
#include "UtilCvPclRs.h"
#include "GLRenderer.h"

void calibrate(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, const std::vector<std::vector<RgbdFrame> >& listFrames, std::vector<cv::Mat>& cameraKOut, std::vector<cv::Mat>& cameraDistOut, std::vector<cv::Mat>& cameraPosesOut, int nbInfraredCam)
{
    printf("board %lf %lf\n", board->getMarkerLength(), board->getSquareLength());
    if(listFrames.size() == 0)
        return ;
    int nbCaptures = listFrames.size();
    int nbCameras = listFrames[0].size();
    int totalNbCameras = nbCameras*(1+nbInfraredCam);
    std::vector<cv::Size> imgSize(totalNbCameras);
    std::vector<std::vector<std::vector<cv::Point2f> > > detections(nbCaptures);//dectection[frame][cameraId][pointId]
    std::vector<std::vector< std::vector<cv::Point2f> > > allCharucoCorners(totalNbCameras);
    std::vector<std::vector< std::vector<int> > > allCharucoIds(totalNbCameras);
    std::vector<std::vector< cv::Mat > > allPosesRt(nbCaptures);
    std::vector<cv::Mat> cameraMatrix(totalNbCameras);
    std::vector<cv::Mat> cameraDistCoeffs(totalNbCameras);
    char name[255];
    printf("1\n");
    for(int i = 0; i < listFrames.size(); i++)
    {
        detections[i] = std::vector<std::vector<cv::Point2f> >(totalNbCameras);
        allPosesRt[i] = std::vector< cv::Mat >(totalNbCameras);
        for(int j = 0; j < listFrames[i].size(); j++)
        {
            for(int k = 0; k <= nbInfraredCam; k++)
            {
                int j2 = j+k*listFrames[i].size();
                detections[i][j2] = std::vector<cv::Point2f>(board->chessboardCorners.size());
                for(int l = 0; l < detections[i][j2].size(); l++)
                    detections[i][j2][l] = cv::Point2f(-1,-1);
                RgbdFrame frame = listFrames[i][j];
                cv::Mat img;
                if(k == 0)
                    img = frame.img;
                else if(k == 1)
                    img = frame.imgIR;
                else img = frame.imgIR2;
                cv::Mat imgInv = cv::Scalar::all(255) - img;
                cv::Mat result = img.clone();

                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f> > markerCorners;
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;

                if(i == 0)
                {
                    cameraMatrix[j2] = getCameraMatrix(frame, k);
                    cameraDistCoeffs[j2] = getCameraDistMatrix(frame, img.size(),  k);
                }

                cv::Mat rvec, tvec;
                bool valid = detectMarker(imgInv, dictionary, board, cameraMatrix[j2], cameraDistCoeffs[j2], markerIds, markerCorners, charucoCorners, charucoIds, rvec, tvec, result);
                imgSize[j2] = imgInv.size();
                allCharucoIds[j2].push_back(charucoIds);
                allCharucoCorners[j2].push_back(charucoCorners);
                for(int l = 0; l < charucoCorners.size(); l++)
                    detections[i][j2][charucoIds[l]] = charucoCorners[l];
                if(valid)
                    allPosesRt[i][j2] = RTVec2Mat(rvec, tvec);


                sprintf(name, "result%d_%d", j, k);
            }
        }
    }
    printf("2\n");

    std::vector<std::vector<double> > calibRt(nbCaptures);
    std::vector<double> calibPoints(board->chessboardCorners.size()*3);
    std::vector<std::vector<double> > cameraPoses(totalNbCameras);
    std::vector<std::vector<double> > cameraK(totalNbCameras);
    std::vector<std::vector<double> > cameraDist(totalNbCameras);
    std::vector<std::vector<double> > detectionPoints3D(nbCaptures);
    std::vector<std::vector<bool> > useDetectionPoints3D(nbCaptures);
    std::vector<std::vector<cv::Mat> > cameraRelativePos(totalNbCameras);
    std::vector<std::vector<int> > cameraRelativePosScore(totalNbCameras);

    for(int i = 0; i < board->chessboardCorners.size(); i++)
    {
        calibPoints[i*3] = board->chessboardCorners[i].x;
        calibPoints[i*3+1] = board->chessboardCorners[i].y;
        calibPoints[i*3+2] = board->chessboardCorners[i].z;
    }
    for(int i = 0; i < totalNbCameras; i++)
    {
        cameraRelativePos[i] = std::vector<cv::Mat>(i+1);
        cameraRelativePosScore[i] = std::vector<int>(i+1);
        for(int j = 0; j < cameraRelativePosScore[i].size(); j++)
            cameraRelativePosScore[i][j] = 0;
    }

    for(int j = 0; j < nbCameras; j++)
    {
        for(int k = 0; k <= nbInfraredCam; k++)
        {
            int j2 = j+k*nbCameras;
            const RgbdFrame& frame = listFrames[0][j];
            cv::Mat img;
            if(k == 0)
                img = frame.img;
            else if(k == 1)
                img = frame.imgIR;
            else img = frame.imgIR2;

            cameraK[j2] = std::vector<double>(4);
            cameraK[j2][0] = cameraMatrix[j2].at<double>(0,0);
            cameraK[j2][1] = cameraMatrix[j2].at<double>(1,1);
            cameraK[j2][2] = cameraMatrix[j2].at<double>(0,2);
            cameraK[j2][3] = cameraMatrix[j2].at<double>(1,2);
            cameraDist[j2] = std::vector<double>(5);
            for(int l = 0; l < 5; l++)
                cameraDist[j2][l] = cameraDistCoeffs[j2].at<double>(0,l);

            if(k == 1)
            {
                cameraRelativePos[j2][j] = frame.getRtMat(frame.ir_to_color).inv();
                cameraRelativePosScore[j2][j] = 100;
            }
            else if(k == 2)
            {
                cameraRelativePos[j2][j] = frame.getRtMat(frame.ir2_to_color).inv();
                cameraRelativePosScore[j2][j] = 100;
            }
        }
    }
    printf("3\n");
    for(int i = 0; i < nbCaptures; i++)
    {
        calibRt[i] = std::vector<double>(6);
        for(int j = 0; j < 6; j++)
            calibRt[i][j] = 0;
        for(int j = 0; j < totalNbCameras; j++)
        {
            cv::Mat pose1 = allPosesRt[i][j];
            if(!pose1.empty())
            {
                for(int j2 = 0; j2 < j; j2++)
                {
                    cv::Mat pose2 = allPosesRt[i][j2];
                    if(!pose2.empty())
                    {
                        int score = std::min(allCharucoIds[j][i].size(), allCharucoIds[j2][i].size());
                        if(score > cameraRelativePosScore[j][j2])
                        {
                            cameraRelativePos[j][j2] = pose1*pose2.inv();
                            cameraRelativePosScore[j][j2] = score;
                        }
                    }
                }
            }
            cameraRelativePos[j][j] = cv::Mat::eye(4,4,CV_64F);
        }
        detectionPoints3D[i] = std::vector<double>(board->chessboardCorners.size()*3);
        useDetectionPoints3D[i] = std::vector<bool>(board->chessboardCorners.size());
        for(int j = 0; j < detectionPoints3D[i].size(); j++)
            detectionPoints3D[i][j] = 0;
        for(int j = 0; j < useDetectionPoints3D[i].size(); j++)
            useDetectionPoints3D[i][j] = false;
    }
    bool improved = true;
    while(improved)
    {
        improved = false;
        for(int j = 0; j < totalNbCameras; j++)
        {
            for(int j2 = 0; j2 < j; j2++)
            {
                if(cameraRelativePos[j][j2].empty())
                {
                    for(int k = 0; k < totalNbCameras; k++)
                    {
                        if(k == j || k == j2)
                            continue;
                        cv::Mat relJK, relKJ2;
                        if(j < k && !cameraRelativePos[k][j].empty())
                            relJK = cameraRelativePos[k][j].inv();
                        else if(k < j && !cameraRelativePos[j][k].empty())
                            relJK = cameraRelativePos[j][k];
                        else continue;

                        if(j2 < k && !cameraRelativePos[k][j2].empty())
                            relKJ2 = cameraRelativePos[k][j2];
                        else if(k < j2 && !cameraRelativePos[j2][k].empty())
                            relKJ2 = cameraRelativePos[j2][k].inv();
                        else continue;

                        cameraRelativePos[j][j2] = relJK*relKJ2;
                        improved = true;
                    }
                }
            }
        }
    }
    printf("4\n");
    for(int j = 0; j < totalNbCameras; j++)
    {
        cameraPoses[j] = std::vector<double>(6);

        if(cameraRelativePos[j][0].empty())
            continue;

        cv::Mat rvec = getRVecFromMat(cameraRelativePos[j][0]);
        cv::Mat tvec = getTVecFromMat(cameraRelativePos[j][0]);

        for(int k = 0; k < 3; k++)
        {
            cameraPoses[j][k] = rvec.at<double>(k,0);
            cameraPoses[j][k+3] = tvec.at<double>(k,0);
        }
    }
    for(int i = 0; i < nbCaptures; i++)
    {
        cv::Mat pose = allPosesRt[i][0];
        for(int k = 1; pose.empty() && k < nbCameras; k++)
            if(!allPosesRt[i][k].empty() && !cameraRelativePos[k][0].empty())
                pose = cameraRelativePos[k][0].inv()*allPosesRt[i][k];
        if(pose.empty())
        {
            printf("error!!!!!!!!!!!!!!\n");
            continue;
        }
        cv::Mat rvec = getRVecFromMat(pose);
        cv::Mat tvec = getTVecFromMat(pose);

        for(int k = 0; k < 3; k++)
        {
            calibRt[i][k] = rvec.at<double>(k,0);
            calibRt[i][k+3] = tvec.at<double>(k,0);
        }

        for(int k = 0; k < board->chessboardCorners.size(); k++)
        {
            cv::Point3f p = multMatVec(pose, board->chessboardCorners[k]);
            detectionPoints3D[i][k*3] = p.x;
            detectionPoints3D[i][k*3+1] = p.y;
            detectionPoints3D[i][k*3+2] = p.z;
            useDetectionPoints3D[i][k] = true;
        }
    }
    printf("5\n");
    ceres::Problem problem;
    for(int i = 0; i < nbCaptures; i++)
    {
        for(int j = 0; j < totalNbCameras; j++)
        {
            if(cameraRelativePos[j][0].empty())
                continue;
            for(int k = 0; k < detections[i][j].size(); k++)
            {
                cv::Point2f p = detections[i][j][k];
                if(p.x >= 0 && p.y >= 0 && useDetectionPoints3D[i][k])
                {
                    ceres::CostFunction* costFunc = ReprojectionError2<5>::Create(p.x, p.y, false);
                    problem.AddResidualBlock(costFunc, NULL, &cameraK[j][0], &cameraDist[j][0], &cameraPoses[j][0], &calibRt[i][0], &calibPoints[k*3]);
                    problem.SetParameterBlockConstant(&cameraK[j][0]);
                    problem.SetParameterBlockConstant(&cameraDist[j][0]);
                    problem.SetParameterBlockConstant(&calibPoints[k*3]);
                }
            }
        }
    }
    problem.SetParameterBlockConstant(&cameraPoses[0][0]);
    for(int j = 0; j < nbCameras; j++)
        for(int k = 1; k <= nbInfraredCam; k++)
            problem.SetParameterBlockConstant(&cameraPoses[k*nbCameras+j][0]);
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    for(int j = 0; j < nbCameras; j++)
        for(int k = 1; k <= nbInfraredCam; k++)
            problem.SetParameterBlockVariable(&cameraPoses[k*nbCameras+j][0]);

    for(int i = 0; i < nbCaptures; i++)
    {
        for(int j = 0; j < nbCameras; j++)
        {
            for(int k = 0; k <= nbInfraredCam; k++)
            {
                int j2 = j+k*listFrames[i].size();
                const RgbdFrame& frame = listFrames[i][j];
                cv::Mat img;
                if(k == 0)
                    img = frame.img;
                else if(k == 1)
                    img = frame.imgIR;
                else img = frame.imgIR2;
                cv::Mat result = img.clone();

                for(int l = 0; l < board->chessboardCorners.size(); l++)
                {
                    if(!useDetectionPoints3D[i][l])
                        continue;
                    cv::Point3f p1(calibPoints[l*3], calibPoints[l*3+1], calibPoints[l*3+2]);
                    cv::Point2f p = ReprojectionError2<5>::project(&cameraK[j2][0], &cameraDist[j2][0], &cameraPoses[j2][0], &calibRt[i][0], p1, false);
                    cv::circle(result, p, 4, cv::Scalar(0,0,255), 2);
                }
                sprintf(name, "result%d", j2);
                cv::imshow(name, result);
            }
        }
        cv::waitKey(200);
    }

    cameraKOut = std::vector<cv::Mat>(totalNbCameras);
    cameraDistOut = std::vector<cv::Mat>(totalNbCameras);
    cameraPosesOut = std::vector<cv::Mat>(totalNbCameras);
    for(int i = 0; i < totalNbCameras; i++)
    {
        cv::Point3f rvec(cameraPoses[i][0], cameraPoses[i][1], cameraPoses[i][2]);
        cv::Point3f tvec(cameraPoses[i][3], cameraPoses[i][4], cameraPoses[i][5]);
        cameraPosesOut[i] = RTVec2Mat(point2Mat(rvec), point2Mat(tvec));
        cameraKOut[i] = cv::Mat::eye(3,3,CV_64F);
        cameraKOut[i].at<double>(0,0) = cameraK[i][0];
        cameraKOut[i].at<double>(1,1) = cameraK[i][1];
        cameraKOut[i].at<double>(0,2) = cameraK[i][2];
        cameraKOut[i].at<double>(1,2) = cameraK[i][3];
        cameraDistOut[i] = cv::Mat::eye(1,5,CV_64F);
        for(int j = 0; j < 5; j++)
            cameraDistOut[i].at<double>(0,j) = cameraDist[i][j];
    }

    printf("%s\n\n", mat2str(listFrames[0][0].getRtMat(listFrames[0][0].ir_to_color)).c_str());
}

std::vector<std::vector<double> > cameraK2Ceres(const std::vector<cv::Mat>& cameraK)
{
    std::vector<std::vector<double> > cameraK_ceres(cameraK.size());
    for(int j = 0; j < cameraK_ceres.size(); j++)
    {
        cameraK_ceres[j] = std::vector<double>(4);
        cameraK_ceres[j][0] = cameraK[j].at<double>(0,0);
        cameraK_ceres[j][1] = cameraK[j].at<double>(1,1);
        cameraK_ceres[j][2] = cameraK[j].at<double>(0,2);
        cameraK_ceres[j][3] = cameraK[j].at<double>(1,2);
    }
    return cameraK_ceres;
}

std::vector<std::vector<double> > cameraDist2Ceres(const std::vector<cv::Mat>& cameraDist)
{
    std::vector<std::vector<double> > cameraDist_ceres(cameraDist.size());
    for(int j = 0; j < cameraDist_ceres.size(); j++)
    {
        cameraDist_ceres[j] = std::vector<double>(5);
        for(int l = 0; l < 5; l++)
            cameraDist_ceres[j][l] = cameraDist[j].at<double>(0,l);
    }
    return cameraDist_ceres;
}

std::vector<std::vector<double> > Rt2Ceres(const std::vector<cv::Mat>& Rt)
{
    std::vector<std::vector<double> >  Rt_ceres(Rt.size());
    for(int l = 0; l < Rt.size(); l++)
    {
        Rt_ceres[l] = std::vector<double>(6);
        cv::Mat rvec = getRVecFromMat(Rt[l]);
        cv::Mat tvec = getTVecFromMat(Rt[l]);
        for(int k = 0; k < 3; k++)
        {
            Rt_ceres[l][k] = rvec.at<double>(k,0);
            Rt_ceres[l][k+3] = tvec.at<double>(k,0);
        }
    }
    return Rt_ceres;
}

std::vector<cv::Mat> ceres2Rt(const std::vector<std::vector<double> >& Rt_ceres)
{
    std::vector<cv::Mat> Rt(Rt_ceres.size());
    for(int i = 0; i < Rt_ceres.size(); i++)
    {
        cv::Mat rvec = cv::Mat(3,1,CV_64F);
        cv::Mat tvec = cv::Mat(3,1,CV_64F);
        for(int k = 0; k < 3; k++)
        {
            rvec.at<double>(k,0) = Rt_ceres[i][k];
            tvec.at<double>(k,0) = Rt_ceres[i][k+3];
        }
        Rt[i] = RTVec2Mat(rvec, tvec);
    }
    return Rt;
}

void createMultiboardOptimizationProblem(ceres::Problem& problem, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > >& allMarkerCorners, std::vector<std::vector<double> >& cameraK, std::vector<std::vector<double> >& cameraDist, std::vector<std::vector<double> >& cameraPoses, std::vector<std::vector<double> >& objectRt, std::vector<std::vector<double> >& planeRt, std::vector<std::vector<std::vector<double> > >& markerPoints, bool fixPlaneRt = false)
{
    for(int i = 0; i < allMarkerCorners.size(); i++)
    {
        for(int j = 0; j < allMarkerCorners[i].size(); j++)
        {
            for(int l = 0; l < listBoard.size(); l++)
            {
                cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
                for(int m = 0; m < board->ids.size(); m++)
                {
                    int markerId = board->ids[m];
                    if(markerId >= markerPoints.size() || markerPoints[markerId].size() == 0)
                        continue;

                    if(allMarkerCorners[i][j].size() <= markerId)
                        continue;

                    for(int k = 0; k < allMarkerCorners[i][j][markerId].size(); k++)
                    {
                        cv::Point2f p = allMarkerCorners[i][j][markerId][k];
                        ceres::CostFunction* costFunc = ReprojectionErrorMultiPlane<5>::Create(p.x, p.y, false);
                        problem.AddResidualBlock(costFunc, NULL, &cameraK[j][0], &cameraDist[j][0], &cameraPoses[j][0], &objectRt[i][0], &planeRt[l][0], &markerPoints[markerId][k][0]);
                        problem.SetParameterBlockConstant(&cameraK[j][0]);
                        problem.SetParameterBlockConstant(&cameraDist[j][0]);
                        problem.SetParameterBlockConstant(&cameraPoses[j][0]);
                        if(fixPlaneRt || l == 0)
                            problem.SetParameterBlockConstant(&planeRt[l][0]);
                        problem.SetParameterBlockConstant(&markerPoints[markerId][k][0]);
                    }
                }
            }
        }
    }
}

void getCameraPoseFromMultiboard(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<std::vector<RgbdFrame> >& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& planeRt, std::vector<cv::Mat>& cameraPoses)
{
    std::vector<std::vector<cv::Point3f> > markerPos;
    for(int l = 0; l < listBoard.size(); l++)
    {
        cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
        for(int k = 0; k < board->objPoints.size(); k++)
        {
            int id = board->ids[k];
            while(markerPos.size() <= id)
                markerPos.push_back(std::vector<cv::Point3f>(4));
            for(int i = 0; i < 4; i++)
                markerPos[id][i] = multMatVec(planeRt[l], board->objPoints[k][i]);
        }
    }
    std::vector<int> poseScore(cameraK.size());
    for(int i = 0; i < poseScore.size(); i++)
        poseScore[i] = 0;

    cameraPoses = std::vector<cv::Mat>(cameraK.size());
    cameraPoses[0] = cv::Mat::eye(4,4,CV_64F);
    for(int i = 0; i < listFrames.size(); i++)
    {
        std::vector<int> bestNbCharucoCorner(listBoard.size());
        for(int j = 0; j < bestNbCharucoCorner.size(); j++)
            bestNbCharucoCorner[j] = 0;

        std::vector<cv::Mat> allRt(listFrames[i].size());
        std::vector<int> nbPoints(listFrames[i].size());

        for(int j = 0; j < listFrames[i].size(); j++)
        {
            nbPoints[j] = 0;
            RgbdFrame frame = listFrames[i][j];
            cv::Mat img = frame.img;
            cv::Mat imgInv = cv::Scalar::all(255) - img;
            cv::Mat result = img.clone();

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;

            detectMarker(imgInv, dictionary, markerIds, markerCorners, result);
            cv::imshow("img", result);
            cv::waitKey(1000);

            std::vector<cv::Point2f> scene;
            std::vector<cv::Point3f> obj;
            for(int k = 0; k < markerIds.size(); k++)
            {
                for(int l = 0; l < markerCorners[k].size(); l++)
                {
                    if(markerIds[k] < markerPos.size())
                    {
                        scene.push_back(markerCorners[k][l]);
                        obj.push_back(markerPos[markerIds[k]][l]);
                    }
                }
            }
            cv::Mat rvec, tvec;
            bool found = false;
            if(obj.size() > 3)
                found = cv::solvePnP(obj, scene, cameraK[j], cameraDist[j], rvec, tvec);
            if(found)
            {
                allRt[j] = RTVec2Mat(rvec, tvec);
                nbPoints[j] = obj.size();
            }

            cv::imshow("img", result);
            cv::waitKey(100);
        }

        for(int j = 1; j < listFrames[i].size(); j++)
        {
            int score = std::min(nbPoints[0], nbPoints[j]);
            if(score > poseScore[j])
            {
                cameraPoses[j] = allRt[j]*allRt[0].inv();
                poseScore[j] = score;
            }
        }
    }
}

void calibrateMultiboard(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<std::vector<RgbdFrame> >& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, std::vector<cv::Mat>& newCameraPoses, std::vector<cv::Mat>& objectRt, std::vector<cv::Mat>& planeRt, bool optimizePlaneRt, bool useDepth)
{
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

    char name[255];
    int nbCaptures = listFrames.size();
    int nbCameras = listFrames[0].size();
    std::vector<int> countMarkerView;
    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > allMarkerCorners(nbCaptures);//allMarkerCorners[frame][cameraId][markerId][corner]
    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > allCharucoCorners(nbCaptures);//allCharucoCorners[frame][cameraId][boardId][charucoId]
    std::vector<std::vector<cv::Mat> > allRt(nbCaptures);//allRt[frame][boardId]

    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(255,0,0));
    colors.push_back(cv::Scalar(0,255,0));
    colors.push_back(cv::Scalar(0,0,255));
    colors.push_back(cv::Scalar(255,255,0));
    colors.push_back(cv::Scalar(0,255,255));
    colors.push_back(cv::Scalar(255,0,255));

    std::vector<std::vector<cv::Mat> > planeRelRt(listBoard.size());
    for(int i = 0; i < planeRelRt.size(); i++)
        planeRelRt[i] = std::vector<cv::Mat>(listBoard.size());

    for(int i = 0; i < listFrames.size(); i++)
    {
        allMarkerCorners[i] = std::vector<std::vector<std::vector<cv::Point2f> > >(listFrames[i].size());
        allCharucoCorners[i] = std::vector<std::vector<std::vector<cv::Point2f> > >(listFrames[i].size());
        allRt[i] = std::vector<cv::Mat>(listBoard.size());

        std::vector<int> bestNbCharucoCorner(listBoard.size());
        for(int j = 0; j < bestNbCharucoCorner.size(); j++)
            bestNbCharucoCorner[j] = 0;

        for(int j = 0; j < listFrames[i].size(); j++)
        {
            allCharucoCorners[i][j] = std::vector<std::vector<cv::Point2f> >(listBoard.size());
            RgbdFrame frame = listFrames[i][j];
            cv::Mat img = frame.img;
            cv::Mat imgInv = cv::Scalar::all(255) - img;
            cv::Mat result = img.clone();

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;

            detectMarker(imgInv, dictionary, markerIds, markerCorners, result);

            for(int l = 0; l < markerIds.size(); l++)
            {
                while(countMarkerView.size() <= markerIds[l])
                    countMarkerView.push_back(0);
                countMarkerView[markerIds[l]]++;
            }

            for(int l = 0; l < listBoard.size(); l++)
            {
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::Mat rvec, tvec;
                bool found = detectCornersAndPose(markerIds, markerCorners, imgInv, listBoard[l], cameraK[j], cameraDist[j], charucoCorners, charucoIds, rvec, tvec, result);

                for(int m = 0; m < charucoIds.size(); m++)
                {
                    while(allCharucoCorners[i][j][l].size() <= charucoIds[m])
                        allCharucoCorners[i][j][l].push_back(cv::Point2f(-1,-1));
                    allCharucoCorners[i][j][l][charucoIds[m]] = charucoCorners[m];
                }

                if(found && charucoIds.size() > bestNbCharucoCorner[l])
                {
                    allRt[i][l] = cameraPoses[j].inv()*RTVec2Mat(rvec, tvec);
                    bestNbCharucoCorner[l] = charucoIds.size();
                }
            }

            for(int l = 0; l < markerIds.size(); l++)
            {
                while(allMarkerCorners[i][j].size() <= markerIds[l])
                    allMarkerCorners[i][j].push_back(std::vector<cv::Point2f>());
                allMarkerCorners[i][j][markerIds[l]] = markerCorners[l];
            }


            sprintf(name, "result%d", j);
            cv::imshow(name, result);
        }

        for(int l = 0; l < listBoard.size(); l++)
            for(int l2 = 0; l2 < listBoard.size(); l2++)
            {
                if(l < planeRt.size() && l2 < planeRt.size() && !planeRt[l].empty() && !planeRt[l2].empty())
                    planeRelRt[l][l2] = planeRt[l].inv()*planeRt[l2];
                else if(!allRt[i][l].empty() && !allRt[i][l2].empty())
                    planeRelRt[l][l2] = allRt[i][l].inv()*allRt[i][l2];
            }

        for(int j = 0; j < listFrames[i].size(); j++)
        {
            RgbdFrame frame = listFrames[i][j];
            cv::Mat img = frame.img;
            cv::Mat result = img.clone();
            for(int l = 0; l < listBoard.size(); l++)
            {
                if(bestNbCharucoCorner[l] > 0)
                {
                    cv::Mat Rt = cameraPoses[j]*allRt[i][l];
                    cv::aruco::drawAxis(result, cameraK[j], cameraDist[j], getRVecFromMat(Rt), getTVecFromMat(Rt), 0.1);
                }

            }
            sprintf(name, "result%d", j);
            cv::imshow(name, result);
        }
        cv::waitKey(100);
    }

    for(int l = 0; l < listBoard.size(); l++)
        for(int l2 = 0; l2 < listBoard.size(); l2++)
            printf("%s\n\n", mat2str(planeRelRt[l][l2]).c_str());

    for(int i = 0; i < listFrames.size(); i++)
    {
        for(int j = 0; j < listFrames[i].size(); j++)
        {
            RgbdFrame frame = listFrames[i][j];
            cv::Mat img = frame.img;
            cv::Mat result = img.clone();

            for(int l = 0; l < listBoard.size(); l++)
            {
                if(allRt[i][l].empty())
                    continue;

                for(int l2 = 0; l2 < listBoard.size(); l2++)
                {
                    if(planeRelRt[l][l2].empty())
                        continue;
                    cv::Mat Rt = cameraPoses[j]*allRt[i][l]*planeRelRt[l][l2];//allRt[i][l]     *     allRt[i][l].inv()*allRt[i][l2];

                    cv::aruco::drawAxis(result, cameraK[j], cameraDist[j], getRVecFromMat(Rt), getTVecFromMat(Rt), 0.1);

                    for(int m = 0; m < listBoard[l2]->chessboardCorners.size(); m++)
                    {
                        cv::Point3f p1 = listBoard[l2]->chessboardCorners[m];
                        cv::Point3f p2 = multMatVec(Rt, p1);
                        cv::Point3f p3 = multMatVec(cameraK[j], p2/p2.z);
                        cv::circle(result, cv::Point2f(p3.x, p3.y), 3, colors[l2%colors.size()], 2);
                    }
                }

                break;
            }

            sprintf(name, "result%d", j);
            cv::imshow(name, result);
        }
        cv::waitKey(1000);//good, just a little bit misaligned
    }

    std::vector<std::vector<double> > cameraK_ceres = cameraK2Ceres(cameraK);
    std::vector<std::vector<double> > cameraDist_ceres = cameraDist2Ceres(cameraDist);
    std::vector<std::vector<double> > cameraPoses_ceres = Rt2Ceres(cameraPoses);
    std::vector<std::vector<double> > objectRt_ceres(listFrames.size());
    std::vector<std::vector<double> > planeRt_ceres(listBoard.size());
    std::vector<std::vector<std::vector<double> > > markerPoints(countMarkerView.size());

    for(int l = 0; l < listBoard.size(); l++)
    {
        cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
        for(int m = 0; m < board->ids.size(); m++)
        {
            int markerId = board->ids[m];
            if(markerId >= countMarkerView.size() || countMarkerView[markerId] <= 1)
                continue;
            markerPoints[markerId] = std::vector<std::vector<double> >(4);
            for(int i = 0; i < 4; i++)
            {
                markerPoints[markerId][i] = std::vector<double>(3);
                markerPoints[markerId][i][0] = board->objPoints[m][i].x;
                markerPoints[markerId][i][1] = board->objPoints[m][i].y;
                markerPoints[markerId][i][2] = board->objPoints[m][i].z;
            }
        }
    }

    planeRt = std::vector<cv::Mat>(listBoard.size());
    for(int l = 0; l < planeRt.size(); l++)
        planeRt[l] = planeRelRt[0][l].empty()?cv::Mat::eye(4,4,CV_64F):planeRelRt[0][l];
    planeRt_ceres = Rt2Ceres(planeRt);

    objectRt = std::vector<cv::Mat>(listFrames.size());
    for(int i = 0; i < objectRt.size(); i++)
    {
        objectRt[i] = cv::Mat();
        for(int l = 0; l < listBoard.size(); l++)
        {
            if(!allRt[i][l].empty() && !planeRt[l].empty())
            {
                objectRt[i] = allRt[i][l]*planeRt[l];
                break;
            }
        }
        if(objectRt[i].empty())
        {
            objectRt[i] = cv::Mat::eye(4,4,CV_64F);
            printf("error frame %d :\n impossible to define orientation of the box!!!!!!!!!!!!\n", i);
            cv::waitKey(10000);
        }
    }
    objectRt_ceres = Rt2Ceres(objectRt);

    ceres::Problem problem;
    createMultiboardOptimizationProblem(problem, listBoard, allMarkerCorners, cameraK_ceres, cameraDist_ceres, cameraPoses_ceres, objectRt_ceres, planeRt_ceres, markerPoints, !optimizePlaneRt);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for(int j = 0; j < listFrames[0].size(); j++)
        problem.SetParameterBlockVariable(&cameraPoses_ceres[j][0]);

    for(int i = 0; i < listFrames.size(); i++)
    {
        for(int j = 0; j < listFrames[i].size(); j++)
        {
            if(!useDepth)
                break;
            RgbdFrame frame = listFrames[i][j];
            generatePointCloudOrganized2(frame, false, false, false);
            cv::Mat result = frame.img.clone();

            std::vector<cv::Mat> listH;
            std::vector<float> listMinX;
            std::vector<float> listMinY;
            std::vector<float> listMaxX;
            std::vector<float> listMaxY;
            std::vector<int> visibleBoard;
            for(int l = 0; l < listBoard.size(); l++)
            {
                int count = 0;
                std::vector<cv::Point2f> corners;
                std::vector<cv::Point2f> modelCorners;
                for(int k = 0; k < allCharucoCorners[i][j][l].size(); k++)
                {
                    if(allCharucoCorners[i][j][l][k].x >= 0 && allCharucoCorners[i][j][l][k].y >= 0)
                    {
                        corners.push_back(allCharucoCorners[i][j][l][k]);
                        cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
                        modelCorners.push_back(cv::Point2f(board->chessboardCorners[k].x, board->chessboardCorners[k].y));
                    }
                }
                if(corners.size() > 4)
                {
                    float minX = modelCorners[0].x;
                    float minY = modelCorners[0].y;
                    float maxX = modelCorners[0].x;
                    float maxY = modelCorners[0].y;
                    for(int k = 1; k < modelCorners.size(); k++)
                    {
                        minX = std::min(minX, modelCorners[k].x);
                        minY = std::min(minY, modelCorners[k].y);
                        maxX = std::max(maxX, modelCorners[k].x);
                        maxY = std::max(maxY, modelCorners[k].y);
                    }
                    cv::Mat H = cv::findHomography(corners, modelCorners);
                    if(!H.empty())
                    {
                        cv::Mat Hinv = H.inv();
                        for(float x = minX; x <= maxX; x+=0.001)
                            for(float y = minY; y <= maxY; y+=0.001)
                            {
                                cv::Point2f p = multMatVec(Hinv, cv::Point2f(x,y));
                            }
                        visibleBoard.push_back(l);
                        listH.push_back(H);
                        listMinX.push_back(minX);
                        listMinY.push_back(minY);
                        listMaxX.push_back(maxX);
                        listMaxY.push_back(maxY);
                    }
                    else
                    {
                        listH.push_back(cv::Mat());
                        listMinX.push_back(0);
                        listMinY.push_back(0);
                        listMaxX.push_back(0);
                        listMaxY.push_back(0);
                    }
                }
                else
                {
                    listH.push_back(cv::Mat());
                    listMinX.push_back(0);
                    listMinY.push_back(0);
                    listMaxX.push_back(0);
                    listMaxY.push_back(0);
                }
            }

            cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
            cv::Mat KD = frame.getKMat(frame.depth_intrin);
            cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
            cv::Mat KRGB = frame.getKMat(frame.color_intrin);

            for(int v = 0; v < frame.rs_cloud_ptr->height; v++)
                for(int u = 0; u < frame.rs_cloud_ptr->width; u++)
                {
                    int i = v*frame.rs_cloud_ptr->width+u;
                    cv::Point3f p = pcl2cv(frame.rs_cloud_ptr->points[i]);
                    if(std::isnan(p.x))
                        continue;
                    //todo : inverse distortion
                    cv::Point3f p2 = multMatVec(depthToColor, p);
                    cv::Point3f uv2 = multMatVec(KRGB, p2/p2.z);
                    if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
                    {
                        for(int l : visibleBoard)
                        {
                            cv::Point2f boardPos = multMatVec(listH[l], cv::Point2f(uv2.x,uv2.y));
                            if(boardPos.x >= listMinX[l] && boardPos.x <= listMaxX[l] && boardPos.y >= listMinY[l] && boardPos.y <= listMaxY[l])
                            {
                                result.ptr<unsigned char>((int)uv2.y)[((int)uv2.x)*3] = 255;
                                printf("%f %f %f -> %f %f %f\n", p2.x, p2.y, p2.z, boardPos.x, boardPos.y, 0.0);
                                ceres::CostFunction* costFunc = Reprojection3DErrorMultiPlane::Create(p2.x, p2.y, p2.z, boardPos.x, boardPos.y, 0.0);
                                problem.AddResidualBlock(costFunc, NULL, &cameraPoses_ceres[j][0], &objectRt_ceres[i][0], &planeRt_ceres[l][0]);
                            }
                        }
                    }
                }
            cv::imshow("img", result);
        }
    }

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    newCameraPoses = ceres2Rt(cameraPoses_ceres);
    objectRt = ceres2Rt(objectRt_ceres);
    planeRt = ceres2Rt(planeRt_ceres);

    cameraK_ceres = cameraK2Ceres(cameraK);
    cameraDist_ceres = cameraDist2Ceres(cameraDist);
    cameraPoses_ceres = Rt2Ceres(newCameraPoses);
    objectRt_ceres = Rt2Ceres(objectRt);
    planeRt_ceres = Rt2Ceres(planeRt);

    for(int i = 0; i < listFrames.size(); i++)
    {
        for(int j = 0; j < listFrames[i].size(); j++)
        {
            RgbdFrame frame = listFrames[i][j];
            cv::Mat img = frame.img;
            cv::Mat result = img.clone();
            for(int l = 0; l < listBoard.size(); l++)
            {
                cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
                for(int m = 0; m < board->ids.size(); m++)
                {
                    int markerId = board->ids[m];
                    if(markerId >= countMarkerView.size() || countMarkerView[markerId] <= 1)
                        continue;

                    for(int k = 0; k < 4; k++)
                    {
                        double res[2];
                        double p[3] = {board->objPoints[m][k].x, board->objPoints[m][k].y, board->objPoints[m][k].z};
                        ReprojectionErrorMultiPlane<5>::project(&cameraK_ceres[j][0], &cameraDist_ceres[j][0], &cameraPoses_ceres[j][0], &objectRt_ceres[i][0], &planeRt_ceres[l][0], &p[0], false, &res[0]);
                        cv::circle(result, cv::Point(res[0], res[1]), 3, colors[l]);
                    }
                }
                cv::Mat Rt = newCameraPoses[j]*objectRt[i]*planeRt[l];
                cv::aruco::drawAxis(result, cameraK[j], cameraDist[j], getRVecFromMat(Rt), getTVecFromMat(Rt), 0.1);
            }
            sprintf(name, "result%d", j);
            cv::imshow(name, result);
            cv::waitKey(1000);
        }
    }

    for(int i = 0; i < listFrames.size(); i++)
    {
        for(int j = 0; j < listFrames[i].size(); j++)
        {
            RgbdFrame frame = listFrames[i][j];
            generatePointCloudOrganized2(frame, false, false, false);
            cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
            cv::Mat KD = frame.getKMat(frame.depth_intrin);
            cv::Mat KDInv = KD.inv();//cameraK[j+nbCameras].inv();
            cv::Mat KRGB = frame.getKMat(frame.color_intrin);//cameraK[j];

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = frame.rs_cloud_ptr;
            cv::Mat cameraPoseInv = (newCameraPoses[j]*objectRt[i]*planeRt[0]).inv();
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
                        unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                        meshData->addVertex(p3, cv::Point3f(rgb[0], rgb[1], rgb[2])/255.0);
                    }
                }
        }
    }
    for(int l = 0; l < listBoard.size(); l++)
    {
        cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
        cv::Mat Rt = planeRt[l];
        for(int m = 0; m < board->chessboardCorners.size(); m++)
        {
            meshData->addVertex(multMatVec(Rt, board->chessboardCorners[m]), cv::Point3f(colors[l][2],colors[l][1],colors[l][0])/255.0);
        }
    }
    renderer->cameraPos = multMatVec((newCameraPoses[1]*objectRt[0]*planeRt[0]).inv(), cv::Point3f(0,0,0));
    renderer->render();
}

cv::Mat detectMultiboardPose(cv::Ptr<cv::aruco::Dictionary> dictionary, const std::vector<cv::Ptr<cv::aruco::CharucoBoard> >& listBoard, const std::vector<RgbdFrame>& listFrames, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, const std::vector<cv::Mat>& planeRt)
{
    std::vector<cv::Mat> objectRt2(1);
    objectRt2[0] = cv::Mat::eye(4,4,CV_64F);

    std::vector<int> countMarkerView;
    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > allMarkerCorners(1);//allMarkerCorners[frame=0][cameraId][markerId][corner]
    std::vector<std::vector<std::vector<std::vector<cv::Point2f> > > > allCharucoCorners(1);//allCharucoCorners[frame=0][cameraId][boardId][cornerId]
    allMarkerCorners[0] = std::vector<std::vector<std::vector<cv::Point2f> > >(listFrames.size());
    allCharucoCorners[0] = std::vector<std::vector<std::vector<cv::Point2f> > >(listFrames.size());
    int maxPoints = 0;
    for(int j = 0; j < listFrames.size(); j++)
    {
        allCharucoCorners[0][j] = std::vector<std::vector<cv::Point2f> >(listBoard.size());
        RgbdFrame frame = listFrames[j];
        cv::Mat img = frame.img;
        cv::Mat imgInv = cv::Scalar::all(255) - img;
        cv::Mat result = img.clone();

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;

        detectMarker(imgInv, dictionary, markerIds, markerCorners, result);

        std::vector<cv::Point3f> objPoints;
        std::vector<cv::Point2f> imgPoints;

        for(int l = 0; l < markerIds.size(); l++)
        {
            while(allMarkerCorners[0][j].size() <= markerIds[l])
                allMarkerCorners[0][j].push_back(std::vector<cv::Point2f>());
            allMarkerCorners[0][j][markerIds[l]] = markerCorners[l];
        }

        for(int l = 0; l < markerIds.size(); l++)
        {
            while(countMarkerView.size() <= markerIds[l])
                countMarkerView.push_back(0);
            countMarkerView[markerIds[l]]++;
        }

        for(int l = 0; l < listBoard.size(); l++)
        {
            cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
            for(int m = 0; m < board->ids.size(); m++)
            {
                int id = board->ids[m];
                if(allMarkerCorners[0][j].size() <= id)
                    continue;
                for(int n = 0; n < allMarkerCorners[0][j][id].size(); n++)
                {
                    cv::Point3f p3d = multMatVec(planeRt[l], board->objPoints[m][n]);
                    cv::Point2f p2d = allMarkerCorners[0][j][id][n];
                    objPoints.push_back(p3d);
                    imgPoints.push_back(p2d);
                }
            }
        }

        cv::Mat rvec, tvec;
        bool found = cv::solvePnP(objPoints, imgPoints, cameraK[j], cameraDist[j], rvec, tvec);

        if(found && objPoints.size() > maxPoints)
        {
            maxPoints = objPoints.size();
            objectRt2[0] = cameraPoses[j].inv()*RTVec2Mat(rvec, tvec);
            cv::aruco::drawAxis(result, cameraK[j], cameraDist[j], rvec, tvec, 0.1);

            char name[255];
            sprintf(name, "result%d_c", j);
            cv::imshow(name, result);
            cv::waitKey(100);
        }
    }

    std::vector<std::vector<double> > cameraK_ceres = cameraK2Ceres(cameraK);
    std::vector<std::vector<double> > cameraDist_ceres = cameraDist2Ceres(cameraDist);
    std::vector<std::vector<double> > cameraPoses_ceres = Rt2Ceres(cameraPoses);
    std::vector<std::vector<double> > objectRt_ceres = Rt2Ceres(objectRt2);
    std::vector<std::vector<double> > planeRt_ceres = Rt2Ceres(planeRt);
    std::vector<std::vector<std::vector<double> > > markerPoints(countMarkerView.size());

    for(int l = 0; l < listBoard.size(); l++)
    {
        cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
        for(int m = 0; m < board->ids.size(); m++)
        {
            int markerId = board->ids[m];
            if(markerId >= countMarkerView.size() || countMarkerView[markerId] == 0)
                continue;
            markerPoints[markerId] = std::vector<std::vector<double> >(4);
            for(int i = 0; i < 4; i++)
            {
                markerPoints[markerId][i] = std::vector<double>(3);
                markerPoints[markerId][i][0] = board->objPoints[m][i].x;
                markerPoints[markerId][i][1] = board->objPoints[m][i].y;
                markerPoints[markerId][i][2] = board->objPoints[m][i].z;
            }
        }
    }

    ceres::Problem problem;
    createMultiboardOptimizationProblem(problem, listBoard, allMarkerCorners, cameraK_ceres, cameraDist_ceres, cameraPoses_ceres, objectRt_ceres, planeRt_ceres, markerPoints, true);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    objectRt2 = ceres2Rt(objectRt_ceres);

    std::vector<cv::Scalar> colors(listBoard.size());
    for(int i = 0; i < colors.size(); i++)
        colors[i] = cv::Scalar(128+rand()%127, 128+rand()%127, 128+rand()%127);

    for(int j = 0; j < listFrames.size(); j++)
    {
        RgbdFrame frame = listFrames[j];
        cv::Mat img = frame.img;
        cv::Mat result = img.clone();
        for(int l = 0; l < listBoard.size(); l++)
        {
            cv::Ptr<cv::aruco::CharucoBoard> board = listBoard[l];
            for(int m = 0; m < board->ids.size(); m++)
            {
                int markerId = board->ids[m];
                if(markerId >= countMarkerView.size() || countMarkerView[markerId] == 0)
                    continue;

                for(int k = 0; k < 4; k++)
                {
                    double res[2];
                    double p[3] = {board->objPoints[m][k].x, board->objPoints[m][k].y, board->objPoints[m][k].z};
                    ReprojectionErrorMultiPlane<5>::project(&cameraK_ceres[j][0], &cameraDist_ceres[j][0], &cameraPoses_ceres[j][0], &objectRt_ceres[0][0], &planeRt_ceres[l][0], &p[0], false, &res[0]);
                    cv::circle(result, cv::Point(res[0], res[1]), 3, colors[l]);
                }
            }
            cv::Mat Rt = cameraPoses[j]*objectRt2[0]*planeRt[l];
            cv::aruco::drawAxis(result, cameraK[j], cameraDist[j], getRVecFromMat(Rt), getTVecFromMat(Rt), 0.1);
        }
        char name[255];
        sprintf(name, "result%d_c", j);
        cv::imshow(name, result);
        cv::waitKey(3000);
    }

    return objectRt2[0];
}
