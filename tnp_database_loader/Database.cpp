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
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"

#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using namespace rapidjson;

int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

ARCItem::ARCItem()
{
}

void ARCItem::load(std::string dirname)
{
    printf("%s\n", dirname.c_str());
    this->dirname = dirname;
    std::vector<std::string> listfiles;
    getdir(dirname, listfiles);
    for(int i = 0; i < listfiles.size(); i++)
    {
        if(has_suffix(listfiles[i], ".json"))
            loadJson(listfiles[i]);
        else if(has_suffix(listfiles[i], ".pcd"))
            pointclouds.push_back(listfiles[i]);
        else if(has_suffix(listfiles[i], ".png"))
            pictures.push_back(listfiles[i]);
    }
}

std::string ARCItem::toString()
{
    std::string str;
    char tmp[255];
    str += "name: "+name+"\n";
    sprintf(tmp, "dim: %lf %lf %lf\n", dimensions[0], dimensions[1], dimensions[2]);
    str += tmp;
    sprintf(tmp, "weight: %lf\n", weight);
    str += tmp;
    str += "type: "+type+"\n";
    str += "description: "+description+"\n";
    str += "pics: ";
    for(int i = 0; i < pictures.size(); i++)
    {
        sprintf(tmp, "%s%s", pictures[i].c_str(), i==pictures.size()-1?"\n":", ");
        str += tmp;
    }
    str += "pointclouds: ";
    for(int i = 0; i < pointclouds.size(); i++)
    {
        sprintf(tmp, "%s%s", pointclouds[i].c_str(), i==pointclouds.size()-1?"\n":", ");
        str += tmp;
    }
    return str;
}

void ARCItem::loadJson(std::string filename)
{
    FILE* fp = fopen((dirname+"/"+filename).c_str(), "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("name"))
        name = d["name"].GetString();
    if(d.HasMember("dimensions") && d["dimensions"].IsArray() && d["dimensions"].Size() == 3)
        for(int i = 0; i < 3; i++)
            dimensions[i] = d["dimensions"][i].GetDouble();
    if(d.HasMember("weight"))
        weight = d["weight"].GetDouble();
    if(d.HasMember("type"))
        type = d["type"].GetString();
    if(d.HasMember("description"))
        description = d["description"].GetString();

    fclose(fp);
}

ARCItemDatabase::ARCItemDatabase()
{
}


void ARCItemDatabase::loadItems(std::string dirname)
{
    std::vector<std::string> listDir;
    getdir(dirname, listDir);
    for(int i = 0; i < listDir.size(); i++)
    {
        if(listDir[i].size() == 0 || listDir[i][0] == '.')
            continue;

        ARCItem item;
        item.load(dirname+"/"+listDir[i]);
        listItems.push_back(item);
    }
}

std::string ARCItemDatabase::toString()
{
    std::string str;
    for(int i = 0; i < listItems.size(); i++)
        str += listItems[i].toString();
    return str;
}

ARCBoxSize::ARCBoxSize()
{
    sizeId = "";
    dimensions[0] = 0;
    dimensions[1] = 0;
    dimensions[2] = 0;
}

std::string ARCBoxSize::toString()
{
    std::string str;
    char tmp[255];
    str += "size_id: "+sizeId+"\n";
    sprintf(tmp, "dim: %lf %lf %lf\n", dimensions[0], dimensions[1], dimensions[2]);
    str += tmp;
    return str;
}


void ARCBoxSizeList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("boxes") && d["boxes"].IsArray())
    {
        for (SizeType i = 0; i < d["boxes"].Size(); i++)
        {
            auto& data = d["boxes"][i];
            if(!data.HasMember("size_id") || !data.HasMember("dimensions") || !data["dimensions"].IsArray() || data["dimensions"].Size() != 3)
            {
                printf("error loading box size\n");
                continue;
            }
            ARCBoxSize boxSize;
            boxSize.sizeId = data["size_id"].GetString();
            for(int i = 0; i < 3; i++)
                boxSize.dimensions[i] = data["dimensions"][i].GetDouble();
            listSizes.push_back(boxSize);
        }
    }
    fclose(fp);
}

std::string ARCBoxSizeList::toString()
{
    std::string str;
    for(int i = 0; i < listSizes.size(); i++)
        str += listSizes[i].toString();
    return str;
}


ARCBoxSize ARCBoxSizeList::getBoxSizeById(std::string id)
{
    for(int i = 0; i < listSizes.size(); i++)
        if(listSizes[i].sizeId == id)
            return listSizes[i];
    return ARCBoxSize();
}

ARCLocation::ARCLocation()
{
    id = "";
    type = "";
    sizeId = "";
}

std::string ARCLocation::toString()
{
    std::string str;
    char tmp[255];
    str += "id: "+id+"\n";
    str += "type: "+type+"\n";
    str += "size_id: "+sizeId+"\n";
    str += "content: ";
    for(int i = 0; i < content.size(); i++)
    {
        sprintf(tmp, "%s%s", content[i].c_str(), i==content.size()-1?"\n":", ");
        str += tmp;
    }
    return str;
}

void ARCLocationList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("bins") && d["bins"].IsArray())
        for (SizeType i = 0; i < d["bins"].Size(); i++)
        {
            auto& data = d["bins"][i];
            if(data.HasMember("bin_id") &&  data.HasMember("contents") && data["contents"].IsArray())
            {
                ARCLocation bin;
                bin.id = data["bin_id"].GetString();
                bin.type = "bin";
                for (SizeType j = 0; j < data["contents"].Size(); j++)
                    bin.content.push_back(data["contents"][j].GetString());
                bins.push_back(bin);
            }
        }
    if(d.HasMember("boxes") && d["boxes"].IsArray())
        for (SizeType i = 0; i < d["boxes"].Size(); i++)
        {
            auto& data = d["boxes"][i];
            if(data.HasMember("size_id") &&  data.HasMember("contents") && data["contents"].IsArray())
            {
                ARCLocation box;
                box.type = "box";
                box.sizeId = data["size_id"].GetString();
                for (SizeType j = 0; j < data["contents"].Size(); j++)
                    box.content.push_back(data["contents"][j].GetString());
                boxes.push_back(box);
            }
        }
    if(d.HasMember("tote"))
    {
        auto& data = d["tote"];
        tote.type = "tote";
        for (SizeType j = 0; j < data["contents"].Size(); j++)
            tote.content.push_back(data["contents"][j].GetString());
    }
    fclose(fp);
}

std::string ARCLocationList::toString()
{
    std::string str;
    for(int i = 0; i < bins.size(); i++)
        str += bins[i].toString();
    for(int i = 0; i < boxes.size(); i++)
        str += boxes[i].toString();
    str += tote.toString();
    return str;
}

ARCOrder::ARCOrder()
{
    sizeId = "";
}

std::string ARCOrder::toString()
{
    std::string str;
    char tmp[255];
    str += "size_id: "+sizeId+"\n";
    str += "content: ";
    for(int i = 0; i < content.size(); i++)
    {
        sprintf(tmp, "%s%s", content[i].c_str(), i==content.size()-1?"\n":", ");
        str += tmp;
    }
    return str;
}

void ARCOrderList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("orders") && d["orders"].IsArray())
        for (SizeType i = 0; i < d["orders"].Size(); i++)
        {
            auto& data = d["orders"][i];
            if(data.HasMember("size_id") &&  data.HasMember("contents") && data["contents"].IsArray())
            {
                ARCOrder order;
                order.sizeId = data["size_id"].GetString();
                for (SizeType j = 0; j < data["contents"].Size(); j++)
                    order.content.push_back(data["contents"][j].GetString());
                listOrders.push_back(order);
            }
        }
    fclose(fp);
}

std::string ARCOrderList::toString()
{
    std::string str;
    for(int i = 0; i < listOrders.size(); i++)
        str += listOrders[i].toString();
    return str;
}
