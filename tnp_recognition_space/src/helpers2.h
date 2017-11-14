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

#ifndef HELPERS2_H_
#define HELPERS2_H_

#include <chrono>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <sys/dir.h>
#include <sys/stat.h>

class Helpers2
{

public:
  static long timeNow();
  static void _mkdir(const char *dir, int a_rights);
  static bool file_exists(const std::string& name);
  static bool getDir(std::string dir, std::vector<std::string> &files, std::string fileType = "");
  static bool hasEnding(std::string const &fullString, std::string const &ending);
  static bool asc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b);
  static bool desc(const std::pair<double, std::pair<int, int> >& a, const std::pair<double, std::pair<int, int> >& b);
  static void printFirstN(int n, std::vector<std::pair<double, std::pair<int, int> > > data_item,
                          std::vector<std::string> itemNames);
private:

};

#endif /* HELPERS2_H_ */
