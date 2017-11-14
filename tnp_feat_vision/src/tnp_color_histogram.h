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

#ifndef SRC_TNP_COLOR_HISTOGRAM_H_
#define SRC_TNP_COLOR_HISTOGRAM_H_

#include "tnp_feat_vision/set_items_info.h"
#include "tnp_feat_vision/set_items_pics.h"
#include "tnp_feat_vision/identify_item.h"
#include "helpers.h"
#include "UtilCvPclRs.h"

#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorHistogramNode
{
public:

  ColorHistogramNode(ros::NodeHandle &nh);
  virtual ~ColorHistogramNode();

  bool setItemsInfo(tnp_feat_vision::set_items_info::Request &req, tnp_feat_vision::set_items_info::Response &res);
  bool identifyItemCallbackOLD(tnp_feat_vision::identify_item::Request &req,
                            tnp_feat_vision::identify_item::Response &res);
  bool identifyItemCallback(tnp_feat_vision::identify_item::Request &req,
                              tnp_feat_vision::identify_item::Response &res);
  std::vector<std::pair<double, int> > identifyItem(const std::vector<cv::Mat>& images, const std::vector<std::string>& considered_items, int db = 0, int histoType = 1, int compareMethod = 0);
  void selfTest();
  void setSelectedDatabaseHistogram(int item, int picture);
  void setSelectedReqHistogram(int picture);
  void drawHistoDetail();
  void clustering(int db, int histoType, int compareMethod);

  cv::Size databaseViewerImgSize;
  int databaseViewerNbLine;
  int databaseViewerNbPictureDrawn;

  int selectedReqPicture, selectedDatabaseItem, selectedDatabasePicture;
  std::vector<std::vector<std::vector<cv::Mat> > > currentDbHisto;
  std::vector<std::vector<std::vector<cv::Mat> > > currentDbImg;
  std::vector<cv::Mat> currentReqImg;
  std::vector<cv::Mat> currentReqHisto;
  std::vector<std::pair<double, int> > currentClassification;
  int currentHistoType;

  std::vector<std::string> item_order_list_;
  std::vector<std::vector<std::string> > folder_trees_; // folder (paths) of the db

private:
  
  ros::NodeHandle nh_;

  enum NodeState
  {
    INIT, READY, ERROR
  };

  NodeState node_state_ = INIT;

  ros::Publisher pub_myState_;
  ros::ServiceServer srv_color_hist_matching_;

  std::vector<std::string> ama_item_folders_ = std::vector<std::string>();
  std::vector<std::vector<std::string> > ama_images_in_folders_ = std::vector<std::vector<std::string> >();
  std::vector<std::string> own_item_folders_ = std::vector<std::string>();
  std::vector<std::vector<std::string> > own_images_in_folders_ = std::vector<std::vector<std::string> >();

  std::vector<std::vector<std::vector<cv::Mat> > > ama_histogram_DB_ =
      std::vector<std::vector<std::vector<cv::Mat> > >(); // [item][picture][histotype]-> histogram
  std::vector<std::vector<std::vector<cv::Mat> > > ama_image_DB_ = std::vector<std::vector<std::vector<cv::Mat> > >(); // [item][picture][hsv]-> image
  std::vector<std::vector<std::vector<cv::Mat> > > own_histogram_DB_ =
      std::vector<std::vector<std::vector<cv::Mat> > >(); // [item][picture][histotype]-> histogram
  std::vector<std::vector<std::vector<cv::Mat> > > own_image_DB_ = std::vector<std::vector<std::vector<cv::Mat> > >(); // [item][picture][hsv]-> image

  std::string amazon_backgroundless_pic_base_folder_;
  std::string amazon_histogram_backup_folder_;
  std::string own_backgroundless_pic_base_folder_;
  std::string own_histogram_backup_folder_;

  std::vector<std::string> db_name_;
  std::vector<std::vector<std::vector<std::vector<cv::Mat> > > > db_locations_; // histograms of the dbs
  std::vector<std::vector<std::vector<std::vector<cv::Mat> > > > image_db_locations_; // miniature item images of the dbs
  std::vector<std::vector<std::vector<float> > > white_db_locations_;
  std::vector<std::vector<std::vector<float> > > black_db_locations_;
  std::vector<std::vector<std::vector<std::string> > > image_trees_; // picture contents (paths) of the folders
  std::vector<std::string> bgr_pic_folder_locations_;
  std::vector<std::string> histo_bck_folder_locations_;

  void init();
  int indexResources(const std::string& baseFolder, std::vector<std::string>& item_folders,
                     std::vector<std::vector<std::string> >& images_in_folders);
  cv::Mat calculateHistograms1(cv::Mat & img, cv::Mat mask, int h_bins = 50, int s_bins = 60);
  cv::Mat calculateHistograms2(cv::Mat & img, cv::Mat mask, int h_bins = 50, int s_bins = 60);
  std::vector<std::pair<double, int> > classify(const std::vector<cv::Mat> imgHisto,
                                                const std::vector<std::vector<cv::Mat> > dbHisto, int compareMethod = 0);
  std::vector<std::pair<double, int> > classifyOLD(const std::vector<cv::Mat> imgHisto,
                                                  const std::vector<std::vector<std::vector<cv::Mat> > > dbHisto);
  void showIntermediateResults(cv::Mat &inputImg, cv::Mat &inputHist, cv::Mat &dbImg, cv::Mat &dbHist);

  cv::Mat generate_1D_histogram(const std::vector<cv::Mat>& histo_planes_input);
};

static std_msgs::String rosMessage(std::string txt);

const char * getTextForEnum(int enumVal);

#endif /* SRC_TNP_COLOR_HISTOGRAM_H_ */
