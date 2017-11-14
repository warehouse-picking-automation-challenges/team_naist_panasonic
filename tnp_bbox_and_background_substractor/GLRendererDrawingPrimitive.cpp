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

#include "GLRendererDrawingPrimitive.h"

void drawLine(std::shared_ptr<GLMeshData> meshData, cv::Point3f p1, cv::Point3f p2, cv::Point3f color)
{
    meshData->addVertex(p1, color);
    meshData->addVertex(p2, color);
}

void drawCone(std::shared_ptr<GLMeshData> meshData, cv::Point3f baseCenter, cv::Point3f topPoint, float radius, cv::Point3f color, int nbRadiusSegment)
{
    cv::Point3f mainAxis = topPoint-baseCenter;
    float height = sqrt(mainAxis.dot(mainAxis));
    mainAxis /= height;
    cv::Point3f radiusAxis1 = mainAxis.cross(cv::Point3f(1,0,0));
    if(radiusAxis1.dot(radiusAxis1) < 0.01)
        radiusAxis1 = mainAxis.cross(cv::Point3f(0,1,0));
    radiusAxis1 /= sqrt(radiusAxis1.dot(radiusAxis1));
    cv::Point3f radiusAxis2 = mainAxis.cross(radiusAxis1);
    radiusAxis2 /= sqrt(radiusAxis2.dot(radiusAxis2));

    for(int k = 0; k < nbRadiusSegment; k++)
    {
        int k2 = (k+1)%nbRadiusSegment;
        cv::Point3f p1 = baseCenter + radius*(cos(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
        cv::Point3f p2 = baseCenter + radius*(cos(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis1 + sin(2*k2*CV_PI/(nbRadiusSegment-1))*radiusAxis2);
        drawLine(meshData, p1, p2, color);
        drawLine(meshData, p1, topPoint, color);
    }
}


void drawCylinder(std::shared_ptr<GLMeshData> meshData, cv::Point3f baseCenter, cv::Point3f top, float radius, cv::Point3f color, int nbHeightSegment, int nbRadiusSegment)
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
            drawLine(meshData, p1, p2, color);
            if(j < nbHeightSegment-1)
                drawLine(meshData, p1, p3, color);
        }
    }
}

void drawBox(std::shared_ptr<GLMeshData> meshData, const std::vector<cv::Point3f>& box, cv::Point3f color)
{
    if(box.size() != 8)
        return;
    std::vector<std::pair<int, int> > listLines;
    listLines.push_back(std::make_pair(0,1)); listLines.push_back(std::make_pair(1,3)); listLines.push_back(std::make_pair(3,2)); listLines.push_back(std::make_pair(2,0));
    listLines.push_back(std::make_pair(4,5)); listLines.push_back(std::make_pair(5,7)); listLines.push_back(std::make_pair(7,6)); listLines.push_back(std::make_pair(6,4));
    listLines.push_back(std::make_pair(0,4)); listLines.push_back(std::make_pair(1,5)); listLines.push_back(std::make_pair(2,6)); listLines.push_back(std::make_pair(3,7));

    for(int k = 0; k < listLines.size(); k++)
        drawLine(meshData, box[listLines[k].first], box[listLines[k].second], color);
}
