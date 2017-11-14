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

cv::Ptr<cv::aruco::CharucoBoard> createCustomCharucoBoard(int squaresX, int squaresY, float squareLength,float markerLength, const cv::Ptr<cv::aruco::Dictionary> &dictionary, int firstId = 0)
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    for(int i = 0; i < board->ids.size(); i++)
        board->ids[i] += firstId;
    return board;
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

int main(int argc, char *argv[])
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    float squareLength = 0.05145;//0.0515;
    float markerLength = 0.03657;//0.03657;
    cv::Ptr<cv::aruco::CharucoBoard> boardScan = cv::aruco::CharucoBoard::create(5,8,squareLength, markerLength, dictionary);
    float ratio = (0.157/4)/squareLength;
    squareLength *= ratio;
    markerLength *= ratio;
    printf("calib square %f marker %f\n", squareLength, markerLength);
    cv::Ptr<cv::aruco::CharucoBoard> boardCalib = cv::aruco::CharucoBoard::create(5,8,squareLength, markerLength, dictionary);


    int squareX = 5, squareY = 8;
    squareLength = 0.035*sqrt(2);
    markerLength = 0.025*sqrt(2);
    ratio = (0.138/4)/squareLength;
    squareLength *= ratio;
    markerLength *= ratio;
    printf("multiboard square %f marker %f\n", squareLength, markerLength);

    cv::Ptr<cv::aruco::CharucoBoard> boardTop = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, 0);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardTop->ids[boardTop->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide1->ids[boardSmallSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide2->ids[boardSmallSide2->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide1->ids[boardLongSide1->ids.size()-1] + 1);

    std::vector<cv::Ptr<cv::aruco::CharucoBoard> > listBoard;
    listBoard.push_back(boardTop);
    listBoard.push_back(boardSmallSide1);
    listBoard.push_back(boardSmallSide2);
    listBoard.push_back(boardLongSide1);
    listBoard.push_back(boardLongSide2);


    if(argc < 4)
    {
        printf("usage : RecognitionSpaceCalibration calib_folder nbCamera nbFrames output_file calib_box_file (firstFrameId) (prefix)\n");
        return 0;
    }
    std::string prefix = "capture";
    int nbCalibFrames, nbCameras, firstFrameId = 0;
    sscanf(argv[2], "%d", &nbCameras);
    sscanf(argv[3], "%d", &nbCalibFrames);
    printf("RecognitionSpaceCalibration calib_folder:%s nbCamera:%d nbFrames:%d output_file:%s", argv[1], nbCameras, nbCalibFrames, argv[4]);
    if(argc >= 6)
        printf(" calib_box_file:%s", argv[5]);
    if(argc >= 7)
        sscanf(argv[6], "%d", &firstFrameId);
    if(argc >= 8)
        prefix = argv[7];
    printf(" firstFrameId:%d", firstFrameId);
    printf("\n");

    std::vector<cv::Mat> cameraK, cameraDist, planeRt, cameraPoses, newCameraPoses, objectRt;
    std::vector<std::string> cameraSerial;

    if(argc >= 6)
        loadCalibBox(argv[5], planeRt);
    cameraSerial = loadFramesIdSerial(argv[1], nbCameras, firstFrameId, prefix.c_str());
    getCameraPosesMultiboardFromRecord(dictionary, listBoard, argv[1], nbCameras, nbCalibFrames, cameraK, cameraDist, planeRt, cameraPoses, firstFrameId, prefix.c_str());
    calibrateMultiboardFromRecord(dictionary, listBoard, argv[1], nbCameras, nbCalibFrames, cameraK, cameraDist, cameraPoses, newCameraPoses, objectRt, planeRt, false, false, firstFrameId, prefix.c_str());
    for(int i = 0; i < newCameraPoses.size(); i++)
        newCameraPoses[i] = newCameraPoses[i]*objectRt[0];
    saveCalib(argv[4], cameraSerial, cameraK, cameraDist, newCameraPoses, planeRt);

    printf("calibration finished\n");
    cv::waitKey(0);

    return 0;
}
