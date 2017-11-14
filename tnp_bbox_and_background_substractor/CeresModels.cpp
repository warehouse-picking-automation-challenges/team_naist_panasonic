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

#include "CeresModels.h"
#include "UtilCvPclRs.h"

std::vector<double> modifiedToNormalBrownCoeffs(const std::vector<double>& modifiedBrownCoeffs, cv::Size imgSize)
{
    std::vector<double> dist = modifiedBrownCoeffs;
    ceres::Problem problem;
    for(int i = 0; i < imgSize.width; i+=10)
        for(int j = 0; j < imgSize.height; j+=10)
        {
            double orig[2] = {i,j};
            double distorted[2];
            applyDistortion<5>(&orig[0], &modifiedBrownCoeffs[0], &distorted[0], true);
            ceres::CostFunction* costFunc = DistortionError<5>::Create(i, j, distorted[0], distorted[1], false);
            problem.AddResidualBlock(costFunc, NULL, &dist[0]);
        }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    printf("%lf,%lf,%lf,%lf,%lf -> %lf,%lf,%lf,%lf,%lf\n", modifiedBrownCoeffs[0], modifiedBrownCoeffs[1], modifiedBrownCoeffs[2], modifiedBrownCoeffs[3], modifiedBrownCoeffs[4], dist[0], dist[1], dist[2], dist[3], dist[4]);
    return dist;
}

cv::Mat modifiedToNormalBrownCoeffs(cv::Mat modifiedBrownCoeffs, cv::Size imgSize)
{
    std::vector<double> modifiedBrownCoeffs2 = mat2vec<double>(modifiedBrownCoeffs);
    return vec2mat<double>(modifiedToNormalBrownCoeffs(modifiedBrownCoeffs2, imgSize), 1, 5, CV_64F);
}

std::vector<double> inversedToNormalBrownCoeffs(const std::vector<double>& inverseBrownCoeffs, cv::Size imgSize)
{
    std::vector<double> dist(5);
    for(int i = 0; i < 5; i++)
        dist[i] = 0;

    ceres::Problem problem;
    for(int i = 0; i < imgSize.width; i+=10)
        for(int j = 0; j < imgSize.height; j+=10)
        {
            double distorted[2] = {i,j};
            double orig[2];
            applyDistortion<5>(&distorted[0], &inverseBrownCoeffs[0], &orig[0], true);
            ceres::CostFunction* costFunc = DistortionError<5>::Create(orig[0], orig[1], distorted[0], distorted[1], false);
            problem.AddResidualBlock(costFunc, NULL, &dist[0]);
        }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    printf("%lf,%lf,%lf,%lf,%lf -> %lf,%lf,%lf,%lf,%lf\n", inverseBrownCoeffs[0], inverseBrownCoeffs[1], inverseBrownCoeffs[2], inverseBrownCoeffs[3], inverseBrownCoeffs[4], dist[0], dist[1], dist[2], dist[3], dist[4]);
    return dist;
}

cv::Mat inversedToNormalBrownCoeffs(cv::Mat inverseBrownCoeffs, cv::Size imgSize)
{
    std::vector<double> inverseBrownCoeffs2 = mat2vec<double>(inverseBrownCoeffs);
    return vec2mat<double>(inversedToNormalBrownCoeffs(inverseBrownCoeffs2, imgSize), 1, 5, CV_64F);
}
