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
            files.push_back(std::string(dirp->d_name));
    }

    closedir(dp);

    return files;

}

std::vector<std::string> getSubDir(const std::string& baseFolder)
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

int getCaptureItems(const std::string& baseFolder, std::vector<std::string>& subFolders, std::vector<int>& firstFrame, std::vector<int>& nbFrames, std::vector<std::vector<std::vector<std::string> > >& listFiles)
{
    int itemCount = 0;
    subFolders = getSubDir(baseFolder);
    for(int i = 0; i < subFolders.size(); i++)
    {
        std::vector<std::string> files = getDirFiles(baseFolder+"/"+subFolders[i], "");
        bool init = false;
        int minId = 0, maxId = 0;
        std::vector<std::pair<int, std::string> > selectedFiles;
        std::vector<std::string> rejectedFiles;
        for(int j = 0; j < files.size(); j++)
        {
            if(files[j].size() > subFolders[i].size() && !files[j].compare(0, subFolders[i].size(), subFolders[i]))
            {
                std::string cropped_filename = files[j].substr(subFolders[i].size());
                int index;
                sscanf(cropped_filename.c_str(), "%d", &index);
                selectedFiles.push_back(std::make_pair(index, files[j]));
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
            else if (strcmp(files[j].c_str(), ".") != 0 && strcmp(files[j].c_str(), "..") != 0)
                rejectedFiles.push_back(files[j]);
        }
        std::vector<std::vector<std::string> > listFilesTmp(maxId-minId+1);
        for(int j = 0; j < selectedFiles.size(); j++)
          listFilesTmp[selectedFiles[j].first - minId].push_back(selectedFiles[j].second);
        listFilesTmp.push_back(rejectedFiles);
        if(!init)
        {
            firstFrame.push_back(0);
            listFiles.push_back(listFilesTmp);
            nbFrames.push_back(0);
        }
        else
        {
            firstFrame.push_back(minId);
            listFiles.push_back(listFilesTmp);
            nbFrames.push_back(maxId-minId+1);
            itemCount += maxId-minId+1;
        }
    }
    return itemCount;
}

void copyFile(std::string srcFile, std::string outFile)
{
    std::ifstream  src(srcFile.c_str(), std::ios::binary);
    std::ofstream  dst(outFile.c_str(), std::ios::binary);

    dst << src.rdbuf();
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("usage : mergeCaptureFolder folder1 folder2 ... out\n");
        return 0;
    }
    std::vector<std::string> folder;
    for(int i = 1; i < argc-1; i++)
        folder.push_back(argv[i]);
    std::vector<std::vector<std::string> > subFolders(folder.size());
    std::vector<std::vector<int> > firstFrame(folder.size()), nbFrames(folder.size());
    std::vector<std::vector<std::vector<std::vector<std::string> > > > listFiles(folder.size());

    std::string outFolder = argv[argc-1];

    mkdir(outFolder.c_str(), 775);
    
    std::vector<std::pair<std::string, std::pair<int, int> > > listFolders;
    for(int i = 0; i < folder.size(); i++)
    {
        getCaptureItems(folder[i], subFolders[i], firstFrame[i], nbFrames[i], listFiles[i]);
        for(int j = 0; j < subFolders[i].size(); j++)
            listFolders.push_back(std::make_pair(subFolders[i][j], std::make_pair(i, j)));
    }
    std::sort(listFolders.begin(), listFolders.end(), [](const std::pair<std::string, std::pair<int, int> >& a, const std::pair<std::string, std::pair<int, int> >& b){ return a < b;});
    int count = 0;
    for(int index = 0; index < listFolders.size(); index++)
    {
        int i = listFolders[index].second.first;
        int j = listFolders[index].second.second;
        std::string currentOutFolder = (outFolder+"/"+subFolders[i][j]);
        mkdir(currentOutFolder.c_str(), 775);
        for(int k = 0; k < listFiles[i][j].size(); k++)
        {
            if(k >= nbFrames[i][j])
            {
                printf("rejected:\n");
                for(int l = 0; l < listFiles[i][j][k].size(); l++)
                {
                    printf("%s\n", listFiles[i][j][k][l].c_str());
                    copyFile((folder[i]+"/"+subFolders[i][j]+"/"+listFiles[i][j][k][l]), (currentOutFolder+"/"+listFiles[i][j][k][l]));
                }
            }
            else
            {
                int id = k + firstFrame[i][j];
                char tmp[255];
                sprintf(tmp, "%s%d", subFolders[i][j].c_str(), id);
                std::string prefix = tmp;       
                sprintf(tmp, "%s%d", subFolders[i][j].c_str(), count++);
                std::string new_prefix = tmp;
                printf("prefix %s:\n", prefix.c_str());
                for(int l = 0; l < listFiles[i][j][k].size(); l++)
                {
                    std::string cropped_filename = listFiles[i][j][k][l].substr(prefix.size());
                    std::string newFilename = (new_prefix+cropped_filename);
                    printf("%s -> %s\n", listFiles[i][j][k][l].c_str(), newFilename.c_str());

                    copyFile((folder[i]+"/"+subFolders[i][j]+"/"+listFiles[i][j][k][l]), (currentOutFolder+"/"+newFilename));
                }
            }
        }
    }
    return 0;
}
