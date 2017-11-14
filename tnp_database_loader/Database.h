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

#ifndef DATABASE_H
#define DATABASE_H

#include <string>
#include <vector>

class ARCItem
{
public:
    ARCItem();
    void load(std::string dirname);
    void loadJson(std::string filename);
    std::string toString();

    std::string name;
    double dimensions[3];
    double weight;
    std::string type;
    std::string description;
    std::string dirname;
    std::vector<std::string> pictures;
    std::vector<std::string> pointclouds;
};

class ARCItemDatabase
{
public:
    ARCItemDatabase();
    void loadItems(std::string dirname);
    std::string toString();

    std::vector<ARCItem> listItems;
};

class ARCBoxSize
{
public:
    ARCBoxSize();
    std::string toString();

    std::string sizeId;
    double dimensions[3];
};

class ARCBoxSizeList
{
public:
    std::vector<ARCBoxSize> listSizes;

    void loadJson(std::string filename);
    ARCBoxSize getBoxSizeById(std::string id);
    std::string toString();
};

class ARCLocation
{
public:
    ARCLocation();
    std::string toString();

    std::string id;
    std::string type;
    std::string sizeId;
    std::vector<std::string> content;
};

class ARCLocationList
{
public:
    void loadJson(std::string filename);
    std::string toString();

    std::vector<ARCLocation> bins;
    std::vector<ARCLocation> boxes;
    ARCLocation tote;
};

class ARCOrder
{
public:
    ARCOrder();
    std::string toString();

    std::string sizeId;
    std::vector<std::string> content;
};

class ARCOrderList
{
public:
    void loadJson(std::string filename);
    std::string toString();

    std::vector<ARCOrder> listOrders;
};

#endif // DATABASE_H
