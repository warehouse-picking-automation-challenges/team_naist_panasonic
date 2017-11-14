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

#include "ros/ros.h"
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <unistd.h>
#define GetCurrentDir getcwd

using namespace rapidjson;

int itemsNum = 0;

std::string knownItemList[] =
{
    "avery_binder",
    "balloons",
    "burts_bees_baby_wipes",
    "toilet_brush",
    "colgate_toothbrush_4pk",
    "crayons",
    "epsom_salts",
    "robots_dvd",
    "glue_sticks",
    "expo_eraser",
    "fiskars_scissors",
    "composition_book",
    "hanes_socks",
    "irish_spring_soap",
    "band_aid_tape",
    "tissue_box",
    "black_fashion_gloves",
    "laugh_out_loud_jokes",
    "mesh_cup",
    "marbles",
    "hand_weight",
    "plastic_wine_glass",
    "poland_spring_water",
    "pie_plates",
    "reynolds_wrap",
    "robots_everywhere",
    "duct_tape",
    "scotch_sponges",
    "speed_stick",
    "hinged_ruled_index_cards",
    "ice_cube_tray",
    "table_cloth",
    "measuring_spoons",
    "bath_sponge",
    "ticonderoga_pencils",
    "mouse_traps",
    "white_facecloth",
    "tennis_ball_container",
    "windex",
    "flashlight"
};

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
    std::sort( files.begin(), files.end());
    return 0;
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

ARCItem::ARCItem()
{
    item_id = "";
    dim_x = 0.0;
    dim_y = 0.0;
    dim_z = 0.0;
    weight = 0.0;
    type = "";
    description = "";
    known = false;
    retrieve_method = 0;
    force = -1;
    forbidden_bins.push_back( "" );
    forbidden_bins.push_back( "" );
}

void ARCItem::load(std::string dirname)
{
    std::vector<std::string> listfiles;
    getdir(dirname, listfiles);

    for(int i = 0; i < listfiles.size(); i++)
    {
        if(has_suffix(listfiles[i], ".json"))
            loadJson(dirname, listfiles[i]);
    }
}

std::string ARCItem::toString()
{
    std::string str;
    char tmp[255];
    str += "item_id: "+item_id+"\n";
    sprintf(tmp, "dim: %lf %lf %lf\n", dim_x, dim_y, dim_z );
    str += tmp;
    sprintf(tmp, "weight: %lf\n", weight);
    str += tmp;
    str += "type: "+type+"\n";
    sprintf(tmp, "known: %d\n\n", known);
    str += tmp;

    return str;
}

void ARCItem::loadJson(std::string dirname, std::string filename)
{
    FILE* fp = fopen((dirname+"/"+filename).c_str(), "r"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("name"))
    {
        item_id = d["name"].GetString(); // this has to be 'name' because that's how it's written in the json file
        known = getKnown(item_id);
        retrieve_method = 0;
    }
    if(d.HasMember("dimensions") && d["dimensions"].IsArray() && d["dimensions"].Size() == 3)
    {
        dim_x = d["dimensions"][0].GetDouble();
        dim_y = d["dimensions"][1].GetDouble();
        dim_z = d["dimensions"][2].GetDouble();
    }

    if(d.HasMember("weight"))
        weight = d["weight"].GetDouble();
    if(d.HasMember("type"))
        type = d["type"].GetString();
    if(d.HasMember("description"))
        description = d["description"].GetString();

    fclose(fp);
}

bool ARCItem::getKnown(std::string item_id)
{
    bool result = false;
    for(int i = 0; i < itemsNum; i++)
    {
        if( knownItemList[i].compare(item_id) == 0 )
        {
            result = true;
            break;
        }
    }

    return result;
}

ARCItemDatabase::ARCItemDatabase()
{
}


void ARCItemDatabase::loadItems(std::string dirname, std::string filename)
{
    itemsNum = sizeof(knownItemList) / sizeof(knownItemList[0]);
    std::vector<std::string> listDir;
    getdir(dirname, listDir);

    for(int i = 0; i < listDir.size(); i++)
    {
        if( listDir[i][0] == '.' || has_suffix(listDir[i], ".json") )
            continue;

        ARCItem item;
        item.load(dirname+"/"+listDir[i]);
        listItems.push_back(item);
    }

    FILE* fp = fopen(filename.c_str(), "r"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("items"))
    {
        for( SizeType i = 0; i < d["items"].Size(); i++ )
        {
           for( int j = 0; j < listItems.size(); ++j  )
            {
                auto& data = d["items"][i];
                if( listItems[j].item_id.compare( data["item_id"].GetString() ) == 0 )
                {
                    std::string tmp_forbidden_bins;
                    listItems[j].retrieve_method = data["retrieve_method"].GetDouble();
                    listItems[j].force = data["force"].GetDouble();
                    tmp_forbidden_bins = data["forbidden_bins"].GetString();
                    if( tmp_forbidden_bins.compare( "bc" ) == 0 || tmp_forbidden_bins.compare( "BC" ) == 0 )
                    {
                        listItems[j].forbidden_bins[0] = "B";
                        listItems[j].forbidden_bins[1] = "C";
                    }
                    else if( tmp_forbidden_bins.compare( "c" ) == 0 || tmp_forbidden_bins.compare( "C" ) == 0 )
                    {
                        listItems[j].forbidden_bins[0] = "C";
                    }
                    break;
                }
           }
        }
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
    FILE* fp = fopen(filename.c_str(), "r"); // non-Windows use "r"
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

    if( strcmp( type.c_str(), "bin") == 0 )
    {
        str += "\t\t{\n\t\t\t\"bin_id\": \""+id+"\",\n\t\t\t\"contents\": [\n";
        if( content.size() > 0 )
        {
            for(int i = 0; i < content.size(); i++)
            {
                sprintf( tmp, "\t\t\t\t\"%s\"%s", content[i].c_str(), i==content.size()-1 ? "\n\t\t\t]\n\t\t}" : ",\n" );
                str += tmp;
            }
        }
        else
        {
            str += "\t\t\t]\n\t\t}";
        }
    }
    else if( strcmp( type.c_str(), "box") == 0 )
    {
        str += "\t\t{\n\t\t\t\"size_id\": \""+sizeId+"\",\n\t\t\t\"contents\": [\n";
        if( content.size() > 0 )
        {
            for(int i = 0; i < content.size(); i++)
            {
                sprintf(tmp, "\t\t\t\t\"%s\"%s", content[i].c_str(), i==content.size()-1?"\n\t\t\t]\n\t\t}":",\n");
                str += tmp;
            }
        }
        else
        {
            str += "\t\t\t]\n\t\t}";
        }
    }
    else
    {
        if( content.size() > 0 )
        {
            for(int i = 0; i < content.size(); i++)
            {
                sprintf(tmp, "\t\t\t\"%s\"%s", content[i].c_str(), i==content.size()-1 ? "\n\t\t]\n\t}\n}\n" : ",\n" );
                str += tmp;
            }
        }
        else
        {
            str += "\t\t]\n\t}\n}\n";
        }
    }

    return str;
}

void ARCLocationList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "r"); // non-Windows use "r"
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
                char tmp[2];
                sprintf( tmp, "%d", i+1 );
                box.id = tmp;
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
    char tmp[255];
    str += "{\n\t\"bins\": [\n";
    if( bins.size() > 0 )
    {
        for(int i = 0; i < bins.size(); i++)
        {
            str += bins[i].toString();
            sprintf( tmp, "%s", i == bins.size() - 1 ? "\n\t],\n\t\"boxes\": [\n" : ",\n" );
            str += tmp;
        }
    }
    else
    {
        str += "\t],\n\t\"boxes\": [\n";
    }

    if( boxes.size() > 0 )
    {
        for(int i = 0; i < boxes.size(); i++)
        {
            str += boxes[i].toString();
            sprintf( tmp, "%s", i == boxes.size() - 1 ? "\n\t],\n\t\"tote\": {\n\t\t\"contents\": [\n" : ",\n" );
            str += tmp;
        }
    }
    else
    {
        str += "\t],\n\t\"tote\": {\n\t\t\"contents\": [\n";
    }

    str += tote.toString();

    return str;
}

void ARCLocationList::getLocation(std::string itemName, std::string &locationType, std::string &locationId)
{
    int located = 0;

    for(int i = 0; i < bins.size() && located == 0; i++)
    {
        for(int j = 0; j < bins[i].content.size() && located == 0; j++)
        {
            if( itemName.compare( bins[i].content[j].c_str() ) == 0 )
            {
                locationType = "bin";
                locationId = bins[i].id;
                located = 1;
            }
        }
    }

    for(int i = 0; i < boxes.size() && located == 0; i++)
    {
        for(int j = 0; j < boxes[i].content.size() && located == 0; j++)
        {
            if( itemName.compare( boxes[i].content[j].c_str() ) == 0 )
            {
                locationType = "box";
                locationId = boxes[i].sizeId;
                located = 1;
            }
        }
    }

    for(int i = 0; i < tote.content.size() && located == 0; i++ )
    {
        if( itemName.compare( tote.content[i].c_str() ) == 0 )
        {
            locationType = "tote";
            locationId = "";
            located = 1;
        }
    }
}

ARCOrder::ARCOrder()
{
    sizeId = "";
    orderProcessed = 0;
}

std::string ARCOrder::toString()
{
    std::string str;
    char tmp[255];
    str += "box_size_id: "+sizeId+"\n";
    str += "content: \n";
    for(int i = 0; i < content.size(); i++)
    {
        sprintf(tmp, "   %s [%d][%s]<%s>%s", content[i].c_str(), itemProcessed[i], locationType[i].c_str(), locationId[i].c_str(), i==content.size()-1?"\n":",\n");
        str += tmp;
    }
    sprintf( tmp, "order processed: %d\n\n", orderProcessed );
    str += tmp;

    return str;
}

void ARCOrderList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "r"); // non-Windows use "r"
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
                {
                    order.content.push_back(data["contents"][j].GetString());
                    order.locationType.push_back("");
                    order.locationId.push_back("");
                    order.itemProcessed.push_back(0);
                }
                order.orderProcessed = 0;

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

ARCBoxLocation::ARCBoxLocation()
{
    boxId = 0;
    sizeId = "";
    orderId = -1;
}

void ARCBoxLocationList::loadJson(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "r"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);

    if(d.HasMember("boxes") && d["boxes"].IsArray())
    {
        for (SizeType i = 0; i < d["boxes"].Size(); i++)
        {
            auto& data = d["boxes"][i];
            if( !data.HasMember("id") || !data.HasMember("size_id") )
            {
                printf("error loading box size\n");
                continue;
            }

            ARCBoxLocation boxLocation;
            boxLocation.boxId = data["id"].GetDouble();
            boxLocation.sizeId = data["size_id"].GetString();
            boxLocation.orderId = -1;
            boxLocationList.push_back(boxLocation);
        }
    }
    fclose(fp);
}

ARCRetrieveMethod::ARCRetrieveMethod()
{
    item_id = "";
    retrieve_method = 0;
    force = -1;
    forbidden_bins = "";
}

void ARCRetrieveMethodList::load(std::string dirname)
{
    std::vector<std::string> listDir;
    getdir(dirname, listDir);

    for(int i = 0; i < listDir.size(); i++)
    {
        if( listDir[i][0] == '.' || has_suffix( listDir[i], ".json" ) )
            continue;

        ARCRetrieveMethod item;

        std::vector<std::string> listfiles;
        getdir(dirname+"/"+listDir[i], listfiles);

        for(int i = 0; i < listfiles.size(); i++)
        {
            if(has_suffix(listfiles[i], ".json"))
            {
                item.item_id = listfiles[i].substr(0, listfiles[i].find_last_of("."));
                break;
            }
        }

        retrieve_methods_list.push_back(item);
    }
}

std::string ARCRetrieveMethodList::toString()
{
    std::string str;
    char tmp[255];

    str += "{\n\t\"items\": [\n";
    if( retrieve_methods_list.size() > 0 )
    {
        for(int i = 0; i < retrieve_methods_list.size(); i++)
        {
            sprintf( tmp,
                "\t\t{\n\t\t\t\"item_id\": \"%s\",\n\t\t\t\"retrieve_method\": %d,\n\t\t\t\"force\": %d,\n\t\t\t\"forbidden_bins\": \"%s\"\n\t\t}%s",
                retrieve_methods_list[i].item_id.c_str(),
                retrieve_methods_list[i].retrieve_method,
                retrieve_methods_list[i].force,
                retrieve_methods_list[i].forbidden_bins.c_str(),
                i == retrieve_methods_list.size() - 1 ? "\n\t]\n}\n" : ",\n"
            );
            str += tmp;
        }
    }
    else
    {
        str += "\t]\n}\n";
    }

    return str;
}
