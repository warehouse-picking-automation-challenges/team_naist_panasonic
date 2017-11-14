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

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("usage : BackgroundRemoverBBoxExtractor image out subscale thresh_low thresh_high inlier_ratio\n");
        return 0;
    }
    cv::Mat img = cv::imread(argv[1]);
    int subScale = 4, thresh_low = 10, thresh_high = 50;
    float inlier_ratio = 0.98;
    if(argc > 3)
        sscanf(argv[3], "%d", &subScale);
    if(argc > 4)
        sscanf(argv[4], "%d", &thresh_low);
    if(argc > 5)
        sscanf(argv[5], "%d", &thresh_high);
    if(argc > 6)
        sscanf(argv[6], "%f", &inlier_ratio);

    cv::Mat imgRot = BackgroundRemoverBBoxExtractor(img, subScale, thresh_low, thresh_high, inlier_ratio);

    cv::imwrite(argv[2], imgRot);
    return 0;
}
