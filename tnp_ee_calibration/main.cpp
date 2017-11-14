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
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "RGBDFrame.h"
#include "UtilCvPclRs.h"

void handEyeCalibration(cv::Ptr<cv::aruco::Dictionary> dictionary, cv::Ptr<cv::aruco::CharucoBoard> board, RgbdFrame handEye, const std::vector<RgbdFrame>& axisFrames)
{
    cv::Mat K = handEye.getKMat(handEye.color_intrin);
    cv::Mat dist = handEye.getDistMat(handEye.color_intrin);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::Mat rvec, tvec;
    cv::Mat result = handEye.img.clone();
    cv::cvtColor(result, result, CV_BGR2RGB);
    detectMarker(cv::Mat(cv::Scalar::all(255) - handEye.img), dictionary, board, K, dist, markerIds, markerCorners, charucoCorners, charucoIds, rvec, tvec, result);
    cv::Mat Rt = RTVec2Mat(rvec, tvec);
    int refPoint = 18;
    cv::Point3f p = (board->objPoints[refPoint][0]+board->objPoints[refPoint][1]+board->objPoints[refPoint][2]+board->objPoints[refPoint][3])/4;

    std::vector<cv::Point3f> listP;
    printf("handEye :%s\n", mat2str(Rt).c_str());
    for(int i = 0; i < axisFrames.size(); i++)
    {
        std::vector<int> markerIds2;
        std::vector<std::vector<cv::Point2f> > markerCorners2;
        std::vector<cv::Point2f> charucoCorners2;
        std::vector<int> charucoIds2;
        cv::Mat rvec2, tvec2;
        cv::Mat result2 = axisFrames[i].img.clone();
        cv::cvtColor(result2, result2, CV_BGR2RGB);
        detectMarker(cv::Mat(cv::Scalar::all(255) - axisFrames[i].img), dictionary, board, K, dist, markerIds2, markerCorners2, charucoCorners2, charucoIds2, rvec2, tvec2, result2);
        cv::Mat Rt2 = RTVec2Mat(rvec2, tvec2);
        printf("Rt %d :%s\n", i, mat2str(Rt2).c_str());
        listP.push_back(multMatVec(Rt2, p));
    }
    cv::Point3f px = p+cv::Point3f(0,1,0);
    cv::Point3f py = p+cv::Point3f(1,0,0);
    p = multMatVec(Rt, p);
    px = multMatVec(Rt, px);
    py = multMatVec(Rt, py);
    cv::Point3f dirX = px-p;
    cv::Point3f dirY = py-p;
    cv::Point3f dirZ = dirX.cross(dirY);

    cv::Point3f dirX2 = listP[1] - listP[2];
    cv::Point3f dirY2 = listP[0] - listP[1];
    cv::Point3f dirZ2 = listP[0] - p;
    dirX2 /= sqrt(dirX2.dot(dirX2));
    dirY2 /= sqrt(dirY2.dot(dirY2));
    dirZ2 /= sqrt(dirZ2.dot(dirZ2));

    px = p+0.01*dirX2;
    py = p+0.01*dirY2;
    cv::Point3f pz = p+0.01*dirZ2;
    cv::Point3f uv = multMatVec(K, p/p.z);
    cv::Point3f uv_x = multMatVec(K, px/px.z);
    cv::Point3f uv_y = multMatVec(K, py/py.z);
    cv::Point3f uv_z = multMatVec(K, pz/pz.z);
    cv::circle(result, cv::Point2f(uv.x, uv.y), 3, cv::Scalar(0,255,0));
    cv::circle(result, cv::Point2f(uv_x.x, uv_x.y), 3, cv::Scalar(0,0,255));
    cv::circle(result, cv::Point2f(uv_y.x, uv_y.y), 3, cv::Scalar(255,0,0));
    cv::circle(result, cv::Point2f(uv_z.x, uv_z.y), 3, cv::Scalar(0,255,255));
    printf("\n\nresult:\n");
    printf("pos : %f, %f, %f\n", p.x, p.y, p.z);
    printf("dirX : %f, %f, %f\n", dirX.x, dirX.y, dirX.z);
    printf("dirY : %f, %f, %f\n", dirY.x, dirY.y, dirY.z);
    printf("dirZ : %f, %f, %f\n", dirZ.x, dirZ.y, dirZ.z);


    printf("dirX2 : %f, %f, %f\n", dirX2.x, dirX2.y, dirX2.z);
    printf("dirY2 : %f, %f, %f\n", dirY2.x, dirY2.y, dirY2.z);
    printf("dirZ2 : %f, %f, %f\n", dirZ2.x, dirZ2.y, dirZ2.z);

    cv::imshow("result", result);
    //cv::imshow("board", board);
    cv::waitKey(0);
}

cv::Ptr<cv::aruco::CharucoBoard> createCustomCharucoBoard(int squaresX, int squaresY, float squareLength,float markerLength, const cv::Ptr<cv::aruco::Dictionary> &dictionary, int firstId = 0)
{
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    for(int i = 0; i < board->ids.size(); i++)
        board->ids[i] += firstId;
    return board;
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("usage : EECalibration calib_folder calib_name\n");
        return 0;
    }

    float squareLength = 0.05145;//0.0515;
    float markerLength = 0.03657;//0.03657;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    int squareX = 5, squareY = 8;
    squareLength = 0.035*sqrt(2);
    markerLength = 0.025*sqrt(2);
    float ratio = (0.138/4)/squareLength;
    squareLength *= ratio;
    markerLength *= ratio;
    printf("multiboard square %f marker %f\n", squareLength, markerLength);

    cv::Ptr<cv::aruco::CharucoBoard> boardTop = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, 0);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardTop->ids[boardTop->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide1->ids[boardSmallSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide2->ids[boardSmallSide2->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide1->ids[boardLongSide1->ids.size()-1] + 1);

    RgbdFrame frame;
    frame.load(argv[1], argv[2]);
    std::vector<RgbdFrame> axisFrames;
    for(int i = 0; i < 3 && argc >= 4+i; i++)
    {
        RgbdFrame frame1;
        frame1.load(argv[1], argv[3+i]);
        axisFrames.push_back(frame1);
    }

    handEyeCalibration(dictionary, boardLongSide2, frame, axisFrames);

    return 0;
}
