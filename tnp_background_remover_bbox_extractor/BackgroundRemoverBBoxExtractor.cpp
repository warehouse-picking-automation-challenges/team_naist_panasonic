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

#include "BackgroundRemoverBBoxExtractor.h"

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
    cv::Mat totalCols = sumCols<unsigned char>(imgGray);
    cv::Mat totalRowsT = sumRowsT<unsigned char>(imgGray);
    float total = sumCols<float>(totalRowsT.t()).at<float>(0,0);

    float minTotal = alpha*total;

    cv::Rect result(0, 0, imgGray.cols, imgGray.rows);

    int min, max;
    int min1, max1;

    getMinRange(totalCols.t(), total, minTotal, min, max);
    
    result.y = min;
    result.height = max-min+1;

    getMinRange(totalRowsT.t(), total, minTotal, min, max);
    result.x = min;
    result.width = max-min+1;

    return result;
}

cv::Mat BackgroundRemoverBBoxExtractor(cv::Mat img, int subScale, int thresh_low, int thresh_high, float inlier_ratio)
{
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, CV_RGB2GRAY);
    imgGray = smoothThreshold(imgGray, thresh_low, thresh_high);
    for(int i = 0; i < imgGray.rows; i++)
    {
        unsigned char *data = img.ptr<unsigned char>(i);
        unsigned char *maskData = imgGray.ptr<unsigned char>(i);
        for(int j = 0; j < imgGray.cols; j++)
        {
            if(maskData[j] == 0)
            {
                data[j*3] = 0;
                data[j*3+1] = 0;
                data[j*3+2] = 0;
            }
        }
    }

    cv::resize(imgGray, imgGray, cv::Size(imgGray.cols/subScale, imgGray.rows/subScale));
    int nbSample = 90/5;
    cv::Point imgCenter = cv::Point( imgGray.cols/2, imgGray.rows/2 );

    int bestScore = imgGray.cols*imgGray.rows*2;
    double bestAngle = 0;
    cv::Rect bestRect(0,0,imgGray.cols, imgGray.rows);
    for(int i = 0; i < nbSample; i++)
    {
        double angle = i*90/(nbSample-1);

        cv::Mat rot_mat = getRotationMatrix2D( imgCenter, angle, 1.0 );
        cv::Mat imgGrayRot;
        warpAffine( imgGray, imgGrayRot, rot_mat, imgGray.size() );
        cv::Rect box = getPercentageBBOX(imgGrayRot, inlier_ratio);

        if(box.width*box.height < bestScore)
        {
            bestScore = box.width*box.height;
            bestAngle = angle;
            bestRect = box;
        }
    }
    cv::Mat rot_mat = getRotationMatrix2D( subScale*imgCenter, bestAngle, 1.0 );
    cv::Mat imgRot;
    warpAffine( img, imgRot, rot_mat, img.size() ); 
    imgRot = imgRot(cv::Rect(bestRect.x*subScale, bestRect.y*subScale, bestRect.width*subScale, bestRect.height*subScale));
    return imgRot;
}
