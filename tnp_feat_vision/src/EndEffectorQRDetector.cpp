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

#include "EndEffectorQRDetector.h"

bool rayCylinderXalignedIntersection(cv::Point3d p0, cv::Point3d dir, cv::Point3d cylinderCenter, double height, double radius, cv::Point3f& result)
{
    cv::Point2d p0B(p0.y, p0.z), dirB(dir.y, dir.z), cylinderCenterB(cylinderCenter.y, cylinderCenter.z);
    cv::Point2d p1B = p0B - cylinderCenterB;
    double a = dirB.dot(dirB);
    double b = 2*p1B.dot(dirB);
    double c = (p1B.dot(p1B) - radius*radius);
    double det = b*b-4*a*c;
    if(det >= 0)
    {
        double val0 = (-b-sqrt(det))/(2*a);
        double val1 = (-b+sqrt(det))/(2*a);
        double val = val0;
        if(val < 0)
            val = val1;
        cv::Point2d res = p0B + val1*dirB;
        printf("%lf == %lf\n", (res - cylinderCenterB).dot(res - cylinderCenterB), radius*radius);
        printf("%lf\n", a*val1*val1 + b*val1 + c);
        result = p0+val*dir;
        return true;
    }
    return false;
    //pI = p0B + d*dirB
    //(pI.x-cylinderCenterB.x)^2+(pI.y-cylinderCenterB.y)^2 = radius*radius
    //(p0B.x + d*dirB.x - cylinderCenterB.x)^2+(p0B.y + d*dirB.y - cylinderCenterB.y)^2 = radius*radius
    //p1B = p0B - cylinderCenterB
    //(p1B.x + d*dirB.x)^2+(p1B.y + d*dirB.y )^2 = radius*radius
    //p1B.x^2 + 2*p1B.x*d*dirB.x + (d*dirB.x)^2     +    p1B.y^2 + 2*p1B.y*d*dirB.y + (d*dirB.y)^2    =    radius*radius
    //d^2*(dirB.x^2 + dirB.y^2) + 2*d*(p1B.x*dirB.x+p1B.y*dirB.y) + (p1B.x^2 + p1B.y^2 - radius*radius) = 0
}

bool findEndEffectorQRPosX(cv::Point3f suctionCylinderCenter, float suctionCylinderHeight, float suctionCylinderRadius, const std::vector<RgbdFrame>& listFrames, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, cv::Ptr<cv::aruco::Dictionary> dictionary, int minId, int maxId, std::vector<cv::Point3f>& camOrig, std::vector<std::pair<cv::Point3f, cv::Point3f> >& cylinderIntersections, std::vector<std::vector<std::vector<cv::Point3f> > >& qrCodeVec, std::vector<std::vector<cv::Point3f> >& qrCodePoints, float& height)
{
    camOrig = std::vector<cv::Point3f>(listFrames.size());
    qrCodeVec = std::vector<std::vector<std::vector<cv::Point3f> > >(listFrames.size());
    std::vector<int> qrFrameCount;
    int maxQrFrameCount = 0;
    qrCodePoints.clear();
    cylinderIntersections.clear();

    int maxFoundId = 0;

    for(int i = 0; i < listFrames.size(); i++)
    {
        RgbdFrame frame = listFrames[i];

        int id = getIdBySerial(serialId, frame.camSerial);
        if(id == -1)
        {
            printf("ERROR, cam serialID %s not found\n", frame.camSerial.c_str());
            return false;
        }

        cv::Point3f P = multMatVecT<double, 3, 4>(cameraPoses[id].inv(), cv::Point3f(0,0,0));
        camOrig[i] = P;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::Mat result = listFrames[i].img.clone();
        std::vector<std::vector<cv::Point3f> > listPoints3d;
        std::vector<std::vector<cv::Point2f> > listPoints2d;
        detectMarker(cv::Scalar::all(255) - listFrames[i].img, dictionary, markerIds, markerCorners, result);

        for(int j = 0; j < markerIds.size(); j++)
        {
            if(markerIds[j] >= minId && markerIds[j] <= maxId)
            {
                
                std::vector<cv::Point2f> p1 = {cv::Point2f(0,0), cv::Point2f(1,0), cv::Point2f(1,1), cv::Point2f(0,1)};
                //std::vector<cv::Point3f> p3d = {cv::Point3f(0,0,0), cv::Point3f(squareLength,0,0), cv::Point3f(squareLength,0,squareLength), cv::Point3f(0,0,squareLength)};
                std::vector<cv::Scalar> colors = {cv::Scalar(0,0,0), cv::Scalar(255,0,0), cv::Scalar(255,0,255), cv::Scalar(0,0,255)};
                std::vector<cv::Point2f> p2d;

                float cx = cameraK[id].at<double>(0,0);
                float cy = cameraK[id].at<double>(1,1);
                float tx = cameraK[id].at<double>(0,2);
                float ty = cameraK[id].at<double>(1,2);


                for(int k = 0; k < markerCorners[j].size(); k++)
                {
                    p2d.push_back(markerCorners[j][k]);
                    cv::Point2f p = markerCorners[j][k];
                    cv::Point3f V = multMatVecT<double, 3, 4>(cameraPoses[id].inv(), cv::Point3f((p.x-tx)/cx, (p.y-ty)/cy,1)) - P;
                    V /= sqrt(V.dot(V));
                    while(qrCodeVec[i].size() <= markerIds[j])
                        qrCodeVec[i].push_back(std::vector<cv::Point3f>());
                    qrCodeVec[i][markerIds[j]].push_back(V);
                    cv::Point3f I;
                    printf("qrCodeVec[%d][%d].push_back(%f,%f,%f)\n", i, markerIds[j], V.x, V.y, V.z);
                    maxFoundId = std::max(maxFoundId, markerIds[j]);
                    printf("maxFoundId = %d\n", maxFoundId);
                    cv::circle(result, markerCorners[j][k], 3, colors[k]);
                }

                cv::Mat H = cv::findHomography(p1, p2d);
                printf("H : %s\n", mat2str(H).c_str());
                cv::Point3f p = multMatVecT<double, 3, 3>(H, cv::Point3f(0.5,0.5, 1));
                p /= p.z;
                cv::circle(result, cv::Point2f(p.x, p.y), 3, cv::Scalar(0,0,255));
                cv::Point3f V = multMatVecT<double, 3, 4>(cameraPoses[id].inv(), cv::Point3f((p.x-tx)/cx, (p.y-ty)/cy,1)) - P;
                V /= sqrt(V.dot(V));
                cv::Point3f I;
                if(rayCylinderXalignedIntersection(P, V, suctionCylinderCenter, suctionCylinderHeight, suctionCylinderRadius, I))
                    cylinderIntersections.push_back(std::make_pair(P,I));
            }
        }
    }


    for(int j = 0; j <= maxFoundId; j++)
    {
        std::vector<cv::Point3f> listP;
        std::vector<std::vector<cv::Point3f> > vecs;
        
        for(int i = 0; i < listFrames.size(); i++)
        {
            RgbdFrame frame = listFrames[i];
            int id = getIdBySerial(serialId, frame.camSerial);

            if(j < qrCodeVec[i].size() && qrCodeVec[i][j].size() > 0)
            {
                listP.push_back(camOrig[i]);
                vecs.push_back(qrCodeVec[i][j]);
            }
        }

        if(listP.size() == 0)
            continue;

//     0 1 2 3 4 5  6  7  8 9
//0   (1 0 0 0 0 0 -V0x 0 0 0) X (x0) = (Px)    //(x0,y0,z0) - d0(V0x,V0y,V0z) = (Px,Py,Pz)
//1   (0 1 0 0 0 0 -V0y 0 0 0)   (y0) = (Py)
//2   (0 0 1 0 0 0 -V0z 0 0 0)   (z0) = (Pz)
//3   (1 0 0 0 0 0 0 -V1x 0 0)   (dx) = (Px)    //(x0,y0+dy,z0+dz) - d1(V1x,V1y,V1z) = (Px,Py,Pz)
//4   (0 1 0 0 1 0 0 -V1y 0 0)   (dy) = (Py)
//5   (0 0 1 0 0 1 0 -V1z 0 0)   (dz) = (Pz)
//6   (1 0 0 1 0 0 0 0 -V2x 0)   (d0) = (Px)    //(x0+dx,y0+dy,z0+dz) - d2(V2x,V2y,V2z) = (Px,Py,Pz)
//7   (0 1 0 0 1 0 0 0 -V2y 0)   (d1) = (Py)
//8   (0 0 1 0 0 1 0 0 -V2z 0)   (d2) = (Pz)
//9   (1 0 0 1 0 0 0 0 0 -V3x)   (d3) = (Px)    //(x0+dx,y0,z0) - d3(V3x,V3y,V3z) = (Px,Py,Pz)
//10  (0 1 0 0 0 0 0 0 0 -V3y)        = (Py)
//11  (0 0 1 0 0 0 0 0 0 -V3z)        = (Pz)
//12  (0 0 0 1 0 0 0 0 0    0)        = (-0.025)
        cv::Mat A = cv::Mat::zeros(12*listP.size()+1, 6+4*listP.size(), CV_64F);
        cv::Mat B(12*listP.size()+1,1, CV_64F);
        for(int i = 0; i < listP.size(); i++)
        {
            for(int k = 0; k < 4; k++)
            {
                for(int l = 0; l < 3; l++)
                    A.at<double>(i*12+k*3+l, l) = 1.0;
                B.at<double>(i*12+k*3, 0) = listP[i].x;
                B.at<double>(i*12+k*3+1, 0) = listP[i].y;
                B.at<double>(i*12+k*3+2, 0) = listP[i].z;

                cv::Point3f V = vecs[i][k];
                A.at<double>(i*12+k*3, i*4+6+k) = -V.x;
                A.at<double>(i*12+k*3+1, i*4+6+k) = -V.y;
                A.at<double>(i*12+k*3+2, i*4+6+k) = -V.z;
            }
            A.at<double>(i*12+4,4) = 1.0;
            A.at<double>(i*12+5,5) = 1.0;
            A.at<double>(i*12+6,3) = 1.0;
            A.at<double>(i*12+7,4) = 1.0;
            A.at<double>(i*12+8,5) = 1.0;
            A.at<double>(i*12+9,3) = 1.0;
        }

        A.at<double>(listP.size()*12,3) = 1.0;
        B.at<double>(listP.size()*12,0) = -0.024643;

        printf("A :%s\n", mat2str(A).c_str());
        printf("B :%s\n", mat2str(B).c_str());
        cv::Mat res;
        solve(A, B, res, cv::DECOMP_SVD);
        printf("res :%s\n", mat2str(res).c_str());

        {
            cv::Point3f p0(res.at<double>(0,0), res.at<double>(1,0), res.at<double>(2,0));
            cv::Point3f p1(res.at<double>(3,0), res.at<double>(4,0), res.at<double>(5,0));
            
            std::vector<cv::Point3f> list;
            list.push_back(p0);
            list.push_back(p0+cv::Point3f(0, p1.y, p1.z));
            list.push_back(p0+cv::Point3f(p1.x, p1.y, p1.z));
            list.push_back(p0+cv::Point3f(p1.x, 0, 0));
            qrCodePoints.push_back(list);
            qrFrameCount.push_back(listP.size());
            maxQrFrameCount = std::max(maxQrFrameCount, (int)listP.size());
        }
    }
    if(maxQrFrameCount >= 2)
    {
        height = 0;
        int count = 0;
        for(int i = 0; i < qrCodePoints.size(); i++)
        {
            if(qrFrameCount[i] == maxQrFrameCount)
            {
                height += (qrCodePoints[i][0].x+qrCodePoints[i][1].x+qrCodePoints[i][2].x+qrCodePoints[i][3].x)/4;
                count++;
            }
        }
        height /= count;
    }
    else if(cylinderIntersections.size() > 0)
    {
        std::vector<float> list;
        for(int i = 0; i < cylinderIntersections.size(); i++)
            list.push_back(cylinderIntersections[i].second.x);
        std::sort(list.begin(), list.end());
        height = list[list.size()/2];
    }
    else return false;

    if(height < suctionCylinderCenter.x - suctionCylinderHeight/2 || height > suctionCylinderCenter.x + suctionCylinderHeight/2)
        return false;

    return true;
}
