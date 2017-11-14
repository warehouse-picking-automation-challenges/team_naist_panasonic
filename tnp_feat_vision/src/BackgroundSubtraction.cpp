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

#include "BackgroundSubtraction.h"
#include "UtilCvPclRs.h"

cv::Mat dilateMap(cv::Mat mat, int val)
{
    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*val + 1 , 2*val +1), cv::Point( val, val ) );
    cv::Mat res;
    cv::dilate( mat, res, element );
    return res;
}

cv::Mat erodeMap(cv::Mat mat, int val)
{
    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*val + 1 , 2*val +1), cv::Point( val, val ) );
    cv::Mat res;
    cv::erode( mat, res, element );
    return res;
}

cv::Mat radiateMap2(cv::Mat mat, float alpha)
{
    cv::Mat mat2, mat3;
    cv::resize(mat, mat2, cv::Size(mat.cols/2, mat.rows/2));
    cv::resize(mat2, mat3, cv::Size(mat.cols, mat.rows));
    return mat+mat3*(alpha-1.0);
}

cv::Mat radiateMap(cv::Mat mat, float alpha)
{
    cv::Mat result = mat.clone();
    float alpha1 = (alpha-1.0)/4;
    result(cv::Rect(1,1,result.cols-2, result.rows-2)) += alpha1*mat(cv::Rect(1,0,result.cols-2, result.rows-2));
    result(cv::Rect(1,1,result.cols-2, result.rows-2)) += alpha1*mat(cv::Rect(0,1,result.cols-2, result.rows-2));
    result(cv::Rect(1,1,result.cols-2, result.rows-2)) += alpha1*mat(cv::Rect(1,2,result.cols-2, result.rows-2));
    result(cv::Rect(1,1,result.cols-2, result.rows-2)) += alpha1*mat(cv::Rect(2,1,result.cols-2, result.rows-2));
    return result;
}

cv::Mat combineForegroundMask(cv::Mat depthBasedMask, cv::Mat diffBasedMask)
{
    diffBasedMask = erodeMap(dilateMap(diffBasedMask, 4), 4);
    depthBasedMask = erodeMap(dilateMap(depthBasedMask, 4), 4);
    cv::Mat result = depthBasedMask.clone();
    cv::Mat tmpMask = depthBasedMask;
    int thresh = 20;
    for(int i = 0; i < 8; i++)
    {
        tmpMask = dilateMap(tmpMask, 4);
        for(int j = 0; j < tmpMask.rows; j++)
        {
            unsigned char *data = tmpMask.ptr<unsigned char>(j);
            unsigned char *srcData = diffBasedMask.ptr<unsigned char>(j);
            unsigned char *dstData = result.ptr<unsigned char>(j);
            for(int k = 0; k < tmpMask.cols; k++)
            {
                if(data[k] > 0 && srcData[k] >= thresh)
                    dstData[k] = 255;
            }
        }
        thresh += 20;
    }
    return result;
}

std::vector<cv::Mat> recognitionSpaceBackgroundSubtractor(BBoxExtractor& bboxExtractor, const std::vector<std::vector<RgbdFrame> >& backgroundFrames, const std::vector<RgbdFrame>& listFrames, const std::vector<std::string>& serialId, const std::vector<cv::Mat>& cameraK, const std::vector<cv::Mat>& cameraDist, const std::vector<cv::Mat>& cameraPoses, bool drawWindows /*= true*/)
{
    int t0 = clock();
    std::vector<cv::Mat> listResult;

    std::vector<cv::Mat> meanBackgroundPic;
    std::vector<cv::Mat> meanBackgroundPicMask;
    std::vector<cv::Mat> maskBackgroundPic;
    for(int i = 0; i < serialId.size(); i++)
    {
        meanBackgroundPic.push_back(cv::Mat());
        meanBackgroundPicMask.push_back(cv::Mat());
        maskBackgroundPic.push_back(cv::Mat());
    }
    for(int i = 0; i < backgroundFrames.size(); i++)
    {
        for(int j = 0; j < backgroundFrames[i].size(); j++)
        {
            RgbdFrame frame = backgroundFrames[i][j];
            int camIndex = getIdBySerial(serialId, frame.camSerial);
            if (camIndex != -1)
            {
                //backgroundSub[camIndex]->apply(frame.img, fgMaskMOG2);
                cv::Mat img16;
                frame.img.convertTo(img16, CV_16UC3);
                if(meanBackgroundPic[camIndex].empty())
                    meanBackgroundPic[camIndex] = img16;
                else meanBackgroundPic[camIndex] += img16;

                if(meanBackgroundPicMask[camIndex].empty() && !frame.mask.empty())
                    meanBackgroundPicMask[camIndex] = frame.mask.clone();
            }
            else printf("ERROR, cam serialID %s not found\n", frame.camSerial.c_str());
        }
    }
    for(int i = 0; i < meanBackgroundPic.size(); i++)
    {
        meanBackgroundPic[i] /= backgroundFrames.size();
        meanBackgroundPic[i].convertTo(meanBackgroundPic[i], CV_8UC3);
    }
    int t1 = clock();

    pcl::PointCloud <pcl::PointXYZ>::Ptr sparseCloud (new pcl::PointCloud <pcl::PointXYZ>);
    sparseCloud = bboxExtractor.segmentedCloud;
    bool kdtreeValid = false;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    if(sparseCloud->size() > 0)
    {
        kdtree.setInputCloud(sparseCloud);
        kdtreeValid = true;
    }

    int t1b = clock();

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
            return listResult;
        }

        cv::Mat depthToColor = frame.getRtMat(frame.depth_to_color);
        cv::Mat KRGB = cameraK[id];
        cv::Mat cameraPoseInv = (cameraPoses[id]).inv();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = frame.rs_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

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

                std::vector<int> idx;
                std::vector<float> distSqr;
                if(!kdtreeValid || kdtree.nearestKSearch(pcl::PointXYZ(p3.x, p3.y, p3.z), 1, idx, distSqr) == 0)
                    continue;
                if(distSqr[0] > 0.01*0.01)
                    continue;

                if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
                {
                    unsigned char *rgb = frame.img.ptr<unsigned char>((int)uv2.y) + ((int)uv2.x)*3;
                    pcl::PointXYZRGB point(rgb[0], rgb[1], rgb[2]);
                    point.x = p3.x;
                    point.y = p3.y;
                    point.z = p3.z;
                    cloud2->push_back(point);
                }
            }

        int t_before_statisticalOutlierDetect = clock();
        cloud = filterStatisticalOutlier(cloud2);
        int t_after_statisticalOutlierDetect = clock();
        printf("statistical outlier detect time : %d\n", t_after_statisticalOutlierDetect-t_before_statisticalOutlierDetect);

        for(int v = 0; v < cloud->height; v++)
            for(int u = 0; u < cloud->width; u++)
            {
                int i = v*cloud->width+u;
                pcl::PointXYZRGB p = cloud->points[i];
                if(std::isnan(p.x))
                    continue;
                cloudRGB->push_back(p);
            }


    }
    printf("kdtree compute time : %d\n", t1b-t1);
    int t2 = clock();

    for(int i = 0; i < listFrames.size(); i++)
    {
        RgbdFrame frame = listFrames[i];
        int id = getIdBySerial(serialId, frame.camSerial);

        cv::Mat result = frame.img.clone();
        cv::cvtColor(result, result, CV_RGB2BGR);
        cv::Mat resultMasked = result.clone();
        cv::Mat mask = cv::Mat::zeros(result.rows, result.cols, CV_8UC1);

        int camIndex = getIdBySerial(serialId, frame.camSerial);

        if(!meanBackgroundPic[camIndex].empty())
        {
            cv::Mat diffImg(result.rows, result.cols, CV_8UC1);
            for(int i = 0; i < diffImg.rows; i++)
            {
                unsigned char *diffData = diffImg.ptr<unsigned char>(i);
                unsigned char *data1 = frame.img.ptr<unsigned char>(i);
                unsigned char *data2 = meanBackgroundPic[camIndex].ptr<unsigned char>(i);
                for(int j = 0; j < diffImg.cols; j++)
                {
                    diffData[j] = (abs((int)data1[j*3]-(int)data2[j*3]) + abs((int)data1[j*3+1]-(int)data2[j*3+1]) + abs((int)data1[j*3+2]-(int)data2[j*3+2]))/3;
                }
            }
            char name[255];
            sprintf(name, "diffImg %s", frame.camSerial.c_str());

            drawFilledCylinder(diffImg, cameraK[id], cameraPoses[id], cv::Point3f(bboxExtractor.realCylinderPosX-bboxExtractor.suctionCylinderHeight/2, bboxExtractor.suctionCylinderCenter.y, bboxExtractor.suctionCylinderCenter.z), cv::Point3f(bboxExtractor.realCylinderPosX+bboxExtractor.suctionCylinderHeight/2, bboxExtractor.suctionCylinderCenter.y, bboxExtractor.suctionCylinderCenter.z), bboxExtractor.suctionCylinderRadius, cv::Scalar(0));
            if(!meanBackgroundPicMask[id].empty())
            {
                for(int i = 0; i < diffImg.rows; i++)
                {
                    unsigned char *diffData = diffImg.ptr<unsigned char>(i);
                    unsigned char *maskData = meanBackgroundPicMask[id].ptr<unsigned char>(i);
                    for(int j = 0; j < diffImg.cols; j++)
                    {
                        if(maskData[j] == 0)
                            diffData[j] = 0;
                    }
                }
            }
            maskBackgroundPic[id] = diffImg;
            cv::imshow(name, diffImg);
            cv::Mat tmpMat = diffImg;
            for(int i = 0; i < 3; i++)
            {
                sprintf(name, "radiate %d  %s", i, frame.camSerial.c_str());
                tmpMat = radiateMap2(tmpMat, 2);
            }
            sprintf(name, "radiate  %s", frame.camSerial.c_str());
        }

        float cx = cameraK[id].at<double>(0,0);
        float cy = cameraK[id].at<double>(1,1);
        float tx = cameraK[id].at<double>(0,2);
        float ty = cameraK[id].at<double>(1,2);

        for(int j = 0; j < cloudRGB->size(); j++)
        {
            cv::Point3f p = pcl2cv(cloudRGB->points[j]);
            if(std::isnan(p.x))
                continue;

            if(!pointInBox(p, bboxExtractor.recognitionCenter, bboxExtractor.recognitionSize))
                continue;

            if(pointInCylinder(p, cv::Point3f(bboxExtractor.realCylinderPosX, bboxExtractor.suctionCylinderCenter.y, bboxExtractor.suctionCylinderCenter.z), bboxExtractor.suctionCylinderMainAxis, bboxExtractor.suctionCylinderHeight, bboxExtractor.suctionCylinderRadius))
                continue;

            cv::Point3f p2 = multMatVecT<double, 3, 4>(cameraPoses[id], p);

            cv::Point2f uv2(cx*p2.x/p2.z + tx, cy*p2.y/p2.z + ty);

            if(uv2.x >= 0 && uv2.y >= 0 && uv2.x < frame.img.cols && uv2.y < frame.img.rows)
            {
                for(int y = std::max((int)uv2.y-1, 0); y <= std::min((int)uv2.y+1, frame.img.rows-1); y++)
                {
                    for(int x = std::max((int)uv2.x-1, 0); x <= std::min((int)uv2.x+1, frame.img.cols-1); x++)
                        mask.at<unsigned char>(y, x) = 255;
                }
            }

        }

        int t_beforeCombine = clock();
        cv::Mat resultMask;
        if(!meanBackgroundPic[camIndex].empty())
            resultMask = combineForegroundMask(mask, maskBackgroundPic[id]);
        else
        {
            printf("error : no background pictures, use depth-only background removal algorithm instead\n");
            resultMask = mask.clone();
        }
        int t_afterCombine = clock();
        printf("combine mask time : %d\n", t_afterCombine-t_beforeCombine);

        for(int i = 0; i < result.rows; i++)
        {
            unsigned char *maskData = resultMask.ptr<unsigned char>(i);
            unsigned char *rgbData = resultMasked.ptr<unsigned char>(i);
            for(int j = 0; j < result.cols; j++)
            {
                rgbData[j*3] = rgbData[j*3]*maskData[j]/255;
                rgbData[j*3+1] = rgbData[j*3+1]*maskData[j]/255;
                rgbData[j*3+2] = rgbData[j*3+2]*maskData[j]/255;
            }
        }

        if(drawWindows)
        {
            char name[255];
            sprintf(name, "img %s", frame.camSerial.c_str());
            sprintf(name, "imgMasked %s", frame.camSerial.c_str());
            cv::imshow(name, resultMasked);
            sprintf(name, "mask %s", frame.camSerial.c_str());

            sprintf(name, "result mask %s", frame.camSerial.c_str());
            cv::waitKey(100);
        }
        listResult.push_back(resultMasked);
    }
    int t_end = clock();
    printf("full time = %d, %d %d %d\n", t_end-t0, t1-t0, t2-t1, t_end-t2);
    return listResult;
}
