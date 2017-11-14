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

#include "Database.h"
#include <iostream>

int main (int argc, char **argv)
{
    ARCItemDatabase database;
    database.loadItems("/home/arnaud/Documents/Training_items_20170224/Training items");
    std::cout << database.toString();

    ARCBoxSizeList boxSizeList;
    boxSizeList.loadJson("/home/arnaud/Documents/team-naist-panasonic/DatabaseLoader/box_sizes.json");
    std::cout << boxSizeList.toString();

    ARCLocationList locationList;
    locationList.loadJson("/home/arnaud/Documents/team-naist-panasonic/DatabaseLoader/item_location_file.json");
    std::cout << locationList.toString();

    ARCOrderList orderList;
    orderList.loadJson("/home/arnaud/Documents/team-naist-panasonic/DatabaseLoader/order_file.json");
    std::cout << orderList.toString();
}
