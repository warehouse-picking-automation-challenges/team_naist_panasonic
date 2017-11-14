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
#include "GLRendererDrawingPrimitive.h"
#include "EndEffectorQRDetector.h"
#include "BackgroundSubtraction.h"

#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <string.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "UtilCvPclRs.h"

using namespace std;

//#define USE_GL

bool hasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending) && 0 != fullString.compare(0, 1, "."));
  else return false;
}

std::vector<std::string> getDirFiles(std::string dir, std::string fileType = "")
{
    std::vector<std::string> files;
    DIR *dp;

    struct dirent *dirp;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
        printf("can't open folder %s\n", dir.c_str());
        return files;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        if (hasEnding(dirp->d_name, fileType) || fileType.compare("") == 0)
            files.push_back(string(dirp->d_name));
    }

    closedir(dp);

    return files;

}

std::vector<std::string> getSubDir(const string& baseFolder)
{
  std::vector<std::string> subFolders;
  if (access(baseFolder.c_str(), F_OK) == -1)
  {
      printf("can't open folder %s\n", baseFolder.c_str());
      return subFolders;
  }

  // iterate over all subfolders where the item's background removed items are located

  DIR* pdir = opendir(baseFolder.c_str());
  struct dirent* entry = readdir(pdir);

  while (entry != NULL)
  {
    if (entry->d_type == DT_DIR && strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
      subFolders.push_back(std::string(entry->d_name));

    entry = readdir(pdir); // iterate;
  }
  closedir(pdir);

  return subFolders;
}

int getCaptureItems(const std::string& baseFolder, std::vector<std::string>& subFolders, std::vector<int>& firstFrame, std::vector<int>& nbFrames)
{
    int itemCount = 0;
    subFolders = getSubDir(baseFolder);
    for(int i = 0; i < subFolders.size(); i++)
    {
        std::vector<std::string> files = getDirFiles(baseFolder+"/"+subFolders[i], ".yml");
        bool init = false;
        int minId = 0, maxId = 0;
        for(int j = 0; j < files.size(); j++)
        {
            if(files[j].size() > subFolders[i].size() && !files[j].compare(0, subFolders[i].size(), subFolders[i]))
            {
                std::string cropped_filename = files[j].substr(subFolders[i].size());
                int index;
                sscanf(cropped_filename.c_str(), "%d", &index);
                printf("%s : %d\n", cropped_filename.c_str(), index);
                if(!init)
                {
                    minId = index;
                    maxId = index;
                    init = true;
                }
                else
                {
                    minId = std::min(minId, index);
                    maxId = std::max(maxId, index);
                }
            }
        }
        if(!init)
        {
            subFolders.erase(subFolders.begin()+i);
            i--;
        }
        else
        {
            firstFrame.push_back(minId);
            nbFrames.push_back(maxId-minId+1);
            itemCount += maxId-minId+1;
        }
    }
    return itemCount;
}

int main(int argc, char *argv[])
{

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    int squareX = 5, squareY = 8;
    float squareLength = 0.035*sqrt(2);
    float markerLength = 0.025*sqrt(2);
    float ratio = (0.138/4)/squareLength;
    squareLength *= ratio;
    markerLength *= ratio;
    printf("multiboard square %f marker %f\n", squareLength, markerLength);
    cv::Ptr<cv::aruco::CharucoBoard> boardTop = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, 0);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardTop->ids[boardTop->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardSmallSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide1->ids[boardSmallSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide1 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardSmallSide2->ids[boardSmallSide2->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> boardLongSide2 = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide1->ids[boardLongSide1->ids.size()-1] + 1);
    cv::Ptr<cv::aruco::CharucoBoard> additionalBoard = createCustomCharucoBoard(squareX,squareY,squareLength, markerLength, dictionary, boardLongSide2->ids[boardLongSide2->ids.size()-1] + 1);

    std::vector<cv::Mat> cameraK, cameraDist, planeRt, cameraPoses, newCameraPoses, objectRt;
    std::vector<std::string> cameraSerial;

    loadCalib("calibMulti2017_07_28_4pm.yml", cameraSerial, cameraK, cameraDist, newCameraPoses, planeRt);

    std::string baseFolder = "/root/share/item_images_1501380073";//"2017.07.21 competition_test_item_images_1500614454";
    std::string backgroundFolder = "__Background";
    std::vector<std::string> subFolders;
    std::vector<int> firstFrame, nbFrames;
    int nbItems = getCaptureItems(baseFolder, subFolders, firstFrame, nbFrames);

    BBoxExtractor bboxExtractor;
    bboxExtractor.init(cameraSerial, cameraK, cameraDist, newCameraPoses, dictionary, additionalBoard->ids[0], additionalBoard->ids[additionalBoard->ids.size()-1] );
    bboxExtractor.loadConfig("recoSpace2017_07_28_4pm.yml");
    std::vector<std::vector<RgbdFrame> > backgroundFrames;
    for(int i = 0; i <= 1; i++)
    {
        backgroundFrames.push_back(loadFramesId((baseFolder+"/"+backgroundFolder).c_str(), cameraSerial.size(), i));
    }

    char filename[255];
    sprintf(filename, "%s/masked_pictures/", baseFolder.c_str());
    mkdir(filename, 775);

    std::vector<cv::Point3f> listSize;
    for(int i = 0; i < subFolders.size(); i++)
    {
        if(subFolders[i] == "calibration" || subFolders[i] == backgroundFolder)
            continue;
        for(int ii = firstFrame[i]; ii < firstFrame[i]+nbFrames[i]; ii++)
        {
            {
                printf("%s : %s%d\n", (baseFolder+"/"+subFolders[i]).c_str(), subFolders[i].c_str(), ii);
                std::vector<RgbdFrame> frames = loadFramesId((baseFolder+"/"+subFolders[i]).c_str(), cameraSerial.size()  , ii, subFolders[i].c_str());
                for(int j = 0; j < frames.size(); j++)
                    generatePointCloudOrganized2(frames[j], false, false, false);
                int t0 = clock();
                bboxExtractor.compute(frames);
                int t1 = clock();
                printf("compute time %d\n", t1-t0);
                if(bboxExtractor._OBB.size() > 0)
                {
                    std::vector<cv::Mat> result = recognitionSpaceBackgroundSubtractor(bboxExtractor, backgroundFrames, frames, cameraSerial, cameraK, cameraDist, newCameraPoses, true);
                    for(int j = 0; j < result.size(); j++)
                    {
                        char filename[255];
                        sprintf(filename, "%s/masked_pictures/%s", baseFolder.c_str(), subFolders[i].c_str());
                        mkdir(filename, 775);
                        sprintf(filename, "%s/masked_pictures/%s/%s%d_cam%d_masked.png", baseFolder.c_str(), subFolders[i].c_str(), subFolders[i].c_str(), ii, j);
                        cv::imwrite(filename, result[j]);
                    }

                    cv::Point3f center = extractCenterFromOBB(bboxExtractor._OBB);
                    cv::Point3f size = extractSizeFromOBB(bboxExtractor._OBB);
                    while(i >= listSize.size())
                        listSize.push_back(cv::Point3f(0,0,0));
                    listSize[i] = size;
                    cv::Mat rot = extractRotationFromOBB(bboxExtractor._OBB);
                    cv::Mat R = getRVecFromMat(rot);
                    printf("center %f %f %f\n", center.x, center.y, center.z);
                    printf("size %f %f %f\n", size.x, size.y, size.z);
                    printf("rotation %lf %lf %lf\n", R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2));

		    {
		     	vector<double> dims;
			dims.push_back(size.x);
			dims.push_back(size.y);
			dims.push_back(size.z);

			std::ofstream bbx_file;
			std::string item_name = subFolders[i];
			long timeNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			bbx_file.open(std::string("bbox/")+item_name + "_" + to_string(timeNow) + ".csv");
			bbx_file << "item name, x, y, z\n";
			bbx_file << item_name << "," << dims[0] << "," << dims[1] << "," << dims[2] << "\n";
			bbx_file.close();
			printf("Bounding box of %s: x: %f m, y: %f m, z: %f m", item_name.c_str(), dims[0], dims[1], dims[2]);
		    }
                }
                else
                {
                    while(i >= listSize.size())
                        listSize.push_back(cv::Point3f(0,0,0));
                }

                cv::waitKey(100);//2000);
            }
        }
    }
    printf("finished\n");

    return 0;
}
