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

#include "RGBDFrame.h"
#include "UtilCvPclRs.h"

int main(int argc, char *argv[])
{
    if(argc < 4)
    {
        printf("usage : testCameraOrientation inputFolder maskFolder nbCameras (prefix) (frameId) (prefixMask) (frameIdMask)\n");
        return 0;
    }

    int nbCameras = 4;
    if(argc >= 3)
        sscanf(argv[3], "%d", &nbCameras);

    std::string prefix1 = "capture";
    if(argc >= 4)
        prefix1 = argv[4];

    int idFrame1 = 0;
    if(argc >= 5)
        sscanf(argv[5], "%d", &idFrame1);

    std::string prefix2 = "capture";
    if(argc >= 6)
        prefix2 = argv[6];

    int idFrame2 = 0;
    if(argc >= 7)
        sscanf(argv[7], "%d", &idFrame2);

    printf("testCameraOrientation %s %s %d %s %d %s %d\n", argv[1], argv[2], nbCameras, prefix1.c_str(), idFrame1, prefix2.c_str(), idFrame2);
    
    std::vector<RgbdFrame> listFrames = loadFramesId(argv[1], nbCameras, idFrame1, prefix1.c_str());
    std::vector<RgbdFrame> listMask = loadFramesId(argv[2], nbCameras, idFrame2, prefix2.c_str());



    for(int i = 0; i < listFrames.size() && i < listMask.size(); i++)
    {
        cv::Mat img = listFrames[i].img;
        cv::Mat mask = listMask[i].img;

        cv::cvtColor(img, img, CV_RGB2BGR);
        cv::cvtColor(mask, mask, CV_RGB2BGR);
        
        cv::Mat result = img.clone();
        for(int k = 0; k < img.rows && k < mask.rows; k++)
        {
            unsigned char *resultData = result.ptr<unsigned char>(k);
            unsigned char *maskData = mask.ptr<unsigned char>(k);
            for(int j = 0; j < img.cols && j < mask.cols; j++)
            {
                if(maskData[j*3] != 0 || maskData[j*3+1] != 0 || maskData[j*3+2] != 0)
                {
                    resultData[j*3] = maskData[j*3];
                    resultData[j*3+1] = maskData[j*3+1];
                    resultData[j*3+2] = maskData[j*3+2];
                } 
            }
        }
        char name[255];
        sprintf(name, "img %d", i);
        cv::imshow(name, result);
        cv::waitKey(0);
    }
    return 0;
}
