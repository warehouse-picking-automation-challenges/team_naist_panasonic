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

#include "helpers2.h"

using namespace std;

/**
 * Returns the current time in milliseconds
 */
long Helpers2::timeNow()
{
  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * recursive mkdir
 * @param dir path
 * @param a_rights access rights
 */
void Helpers2::_mkdir(const char *dir, int a_rights)
{
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", dir);
  len = strlen(tmp);
  if (tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for (p = tmp + 1; *p; p++)
    if (*p == '/')
    {
      *p = 0;
      mkdir(tmp, a_rights);
      *p = '/';
    }
  mkdir(tmp, a_rights);
}

/**
 *  Returns files in the directory *
 * @param dir directory string
 * @param file and folder names in given dir path
 * @param fileType fileType filter [default "", returns all files]
 * @return success of folder access
 */
bool Helpers2::getDir(string dir, vector<string> &files, string fileType /* = ""*/)
{
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(dir.c_str())) == NULL)
  {
    return false;
  }

  while ((dirp = readdir(dp)) != NULL)
  {
    if (hasEnding(dirp->d_name, fileType))
      files.push_back(string(dirp->d_name));
  }

  closedir(dp);
  return true;
}

bool Helpers2::file_exists(const std::string& name)
{
  if (FILE *file = fopen(name.c_str(), "r"))
  {
    fclose(file);
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * File name has a particular ending and is not a hidden file.
 * @param fullString fileName
 * @param ending fileType
 * @return hasEnding
 */
bool Helpers2::hasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending)
        && 0 != fullString.compare(0, 1, "."));
  }
  else
  {
    return false;
  }
}

/**
 * Sorts the file structure descending, smallest first
 * @param a
 * @param b
 * @return a > b
 */
bool Helpers2::asc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b)
{
  return a.first < b.first;
}

/**
 * Sorts the file structure descending, largest first
 * @param a
 * @param b
 * @return a < b
 */
bool Helpers2::desc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b)
{
  return a.first > b.first;
}

void Helpers2::printFirstN(int n, std::vector<std::pair<double, std::pair<int, int> > > data_item,
                          vector<string> itemNames)
{
  ROS_DEBUG("printFirstN        ");
  for (int id = 0; id < n && id < data_item.size(); id++)
  {
    int j = data_item[id].second.first; // item
    int k = data_item[id].second.second; // picture
    ROS_INFO("Similarity to %s - Training data %i | Distance %f", itemNames[j].c_str(), k, data_item[id].first);
  }
}

