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

#include "tnp_color_histogram.h"

using namespace std;

std::vector<cv::Mat> calc_hue_sat_histogram(const cv::Mat & inputHist)
{
  cv::Mat huePlane = cv::Mat::zeros(1, inputHist.rows, CV_32F);
  cv::Mat satPlane = cv::Mat::zeros(1, inputHist.cols, CV_32F);
  float* huePlaneData = huePlane.ptr<float>(0);
  float* satPlaneData = satPlane.ptr<float>(0);
  float total = 0;
  for (int i = 0; i < inputHist.rows; i++)
  {
    const float* data = inputHist.ptr<float>(i);
    for (int j = 0; j < inputHist.cols; j++)
    {
      huePlaneData[i] += data[j];
      satPlaneData[j] += data[j];
      total += data[j];
    }
  }
  huePlane /= total;
  satPlane /= total;
  vector<cv::Mat> histo_planes;
  histo_planes.push_back(huePlane);
  histo_planes.push_back(satPlane);
  return histo_planes;
}

cv::Mat ColorHistogramNode::generate_1D_histogram(const vector<cv::Mat>& histo_planes_input)
{
  // Draw the histograms for H S and V
  int hist_img_w = 500;
  int hist_img_h = 185;
  // Allocate new image for input_histogram
  cv::Mat hueLine(1, hist_img_w, CV_8UC3);
  for (int i = 0; i < hueLine.cols; i++)
  {
    hueLine.ptr<unsigned char>(0)[i * 3] = i * 180 / hueLine.cols;
    hueLine.ptr<unsigned char>(0)[i * 3 + 1] = 255;
    hueLine.ptr<unsigned char>(0)[i * 3 + 2] = 255;
  }
  cvtColor(hueLine, hueLine, CV_HSV2BGR);
  int hueLineHeight = 10;
  cv::Mat input_hist_image(hist_img_h + hueLineHeight, hist_img_w, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < histo_planes_input.size(); i++)
  {
    for (int j = 1; j < histo_planes_input[i].cols; j++)
    {
      float lastBinVal = histo_planes_input[i].at<float>(0, j - 1);
      float currentBinVal = histo_planes_input[i].at<float>(0, j);
      cv::Point p1((j - 1) * hist_img_w / histo_planes_input[i].cols, hueLineHeight + (1.0 - lastBinVal) * hist_img_h);
      cv::Point p2(j * hist_img_w / histo_planes_input[i].cols, hueLineHeight + (1.0 - currentBinVal) * hist_img_h);
      cv::line(input_hist_image, p1, p2, cv::Scalar(0, i == 1 ? 255 : 0, i == 0 ? 255 : 0));
    }
  }
  for (int i = 0; i < hueLineHeight; i++)
    hueLine.copyTo(input_hist_image(cv::Rect(0, i, hist_img_w, 1)));
  return input_hist_image;
}

float blackObjectProbability(cv::Mat hsvMat, cv::Mat mask)
{
    float total = 0;
    float result = 0;
    int threshold = 255*3/10;
    for(int i = 0; i < hsvMat.rows; i++)
    {
      unsigned char *src = hsvMat.ptr<unsigned char>(i);
      unsigned char *maskData = mask.ptr<unsigned char>(i);
      for(int j = 0; j < hsvMat.cols; j++)
      {
        if(maskData[j] > 0)
        {
          if(src[j*3+2] < threshold)
            result += float(threshold-src[j*3+2])/threshold;
          total += 1;
        }
      }
    }
    return result/total;
}

float whiteObjectProbability(cv::Mat hsvMat, cv::Mat mask)
{
    float total = 0;
    float result = 0;
    int thresholdS = 255*4/10, thresholdS2 = 255*1/10;
    int thresholdV = 255*4/10, thresholdV2 = 255*8/10;
    for(int i = 0; i < hsvMat.rows; i++)
    {
      unsigned char *src = hsvMat.ptr<unsigned char>(i);
      unsigned char *maskData = mask.ptr<unsigned char>(i);
      for(int j = 0; j < hsvMat.cols; j++)
      {
        if(maskData[j] > 0)
        {
          if(src[j*3+1] < thresholdS && src[j*3+2] > thresholdV)
          {
            float score1 = std::min(1.0f, float(thresholdS-src[j*3+1])/(thresholdS-thresholdS2));
            float score2 = std::min(1.0f, float(src[j*3+2]-thresholdV)/(thresholdV2-thresholdV));
            result += score1*score2;
          }
          total += 1;
        }
      }
    }
    return result/total;
}

/**
 * Contructor of Color histogram-based classifier node
 * @param nh ros node
 */
ColorHistogramNode::ColorHistogramNode(ros::NodeHandle &nh) :
    nh_(nh)
{

  pub_myState_ = nh_.advertise<std_msgs::String>("tnp_color_histogram/state", 2);
  srv_color_hist_matching_ = nh_.advertiseService("tnp_color_histogram/identify_item",
                                                  &ColorHistogramNode::identifyItemCallback, this);

  pub_myState_.publish(rosMessage(getTextForEnum(node_state_)));

  amazon_backgroundless_pic_base_folder_ = "/root/share/tnp_feature_vision/color_histogram/amazon_data/bgr_img/";
  amazon_histogram_backup_folder_ = "/root/share/tnp_feature_vision/color_histogram/amazon_data/saved_histograms/";

  own_backgroundless_pic_base_folder_ = "/root/share/tnp_feature_vision/color_histogram/own_data/bgr_img/";
  own_histogram_backup_folder_ = "/root/share/tnp_feature_vision/color_histogram/own_data/saved_histograms/";

  Helpers::_mkdir(amazon_backgroundless_pic_base_folder_.c_str(), 0775);
  Helpers::_mkdir(amazon_histogram_backup_folder_.c_str(), 0775);
  Helpers::_mkdir(own_backgroundless_pic_base_folder_.c_str(), 0775);
  Helpers::_mkdir(own_histogram_backup_folder_.c_str(), 0775);

  init();
}

/**
 * Destructor
 */
ColorHistogramNode::~ColorHistogramNode()
{
  // TODO Auto-generated destructor stub
}

bool loadHistogram(std::string filename, std::vector<cv::Mat>& histos, float &white, float &black)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened())
    return false;
  cv::FileNode histosNode = fs["histos"];
  for (int i = 0; i < histosNode.size(); i++)
  {
    cv::Mat hist;
    histosNode[i] >> hist;
    histos.push_back(hist);
  }
  fs["white"] >> white;
  fs["black"] >> black;
  fs.release();
  return true;
}

void saveHistogram(std::string filename, const std::vector<cv::Mat>& histos, float white, float black)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "histos" << "[";
  for (int i = 0; i < histos.size(); i++)
  {
    fs << histos[i];
  }
  fs << "]";
  fs << "white" << white;
  fs << "black" << black;
  fs.release();
}

cv::Mat generateRgba(cv::Mat rgbImg)
{
  if(rgbImg.channels() == 4)
    return rgbImg.clone();
  else
  {
    cv::Mat rgbaImg(rgbImg.rows, rgbImg.cols, CV_8UC4);
    for(int i = 0; i < rgbImg.rows; i++)
    {
      unsigned char *src = rgbImg.ptr<unsigned char>(i);
      unsigned char *dst = rgbaImg.ptr<unsigned char>(i);
      for (int j = 0; j < rgbImg.cols; j++)
      {
        unsigned char *rgb = src + j * 3;
        unsigned char *rgba = dst + j * 4;
        if (rgb[0] * rgb[0] + rgb[1] * rgb[1] + rgb[2] * rgb[2] < 10 * 10 * 3)
        {
          rgba[0] = 0;
          rgba[1] = 0;
          rgba[2] = 0;
          rgba[3] = 0;
        }
        else
        {
          rgba[0] = rgb[0];
          rgba[1] = rgb[1];
          rgba[2] = rgb[2];
          rgba[3] = 255;
        }
      }
    }
    return rgbaImg;
  }
}

cv::Mat cropBlack(cv::Mat img)
{
  int minX = img.cols-1, minY = img.rows-1, maxX = 0, maxY = 0;
  for(int i = 0; i < img.rows; i++)
  {
    unsigned char *data = img.ptr<unsigned char>(i);
    for(int j = 0; j < img.cols; j++)
    {
      if(data[j*img.channels()] > 0 || data[j*img.channels()+1] > 0 || data[j*img.channels()+2] > 0)
      {
        minX = std::min(minX, j);
        minY = std::min(minY, i);
        maxX = std::max(maxX, j);
        maxY = std::max(maxY, i);
      }
    }
  }
  return img(cv::Rect(minX, minY, maxX-minX+1, maxY-minY+1));
}

/**
 * Reads background removed pictures from disk and calulates histogram features.
 */
void ColorHistogramNode::init()
{
  long t_start = Helpers::timeNow();

  if (access(amazon_backgroundless_pic_base_folder_.c_str(), F_OK) != -1) // folder exists
  {
    // iterate over all sub folders where the item's background removed items are located
    if (indexResources(amazon_backgroundless_pic_base_folder_, ama_item_folders_, ama_images_in_folders_) > 0)
    {
      // check whether backup directory exists already, otherwise create.
      struct stat st = {0};
      if (stat(amazon_histogram_backup_folder_.c_str(), &st) == -1)
      {
        mkdir(amazon_histogram_backup_folder_.c_str(), 0755);
        ROS_DEBUG("Created backup folder %s", amazon_histogram_backup_folder_.c_str());
      }
    }
    else
    {
      node_state_ = ERROR;
      ROS_ERROR("Amazon data: No item raw data found to work on.");
    }
  }
  else
  {
    node_state_ = ERROR;
    ROS_ERROR("Amazon data: Couldn't find the bgr_item pic folder %s", amazon_backgroundless_pic_base_folder_.c_str());
  }

  if (access(own_backgroundless_pic_base_folder_.c_str(), F_OK) != -1) // folder exists
  {
    // iterate over all sub folders where the item's background removed items are located
    if (indexResources(own_backgroundless_pic_base_folder_, own_item_folders_, own_images_in_folders_) > 0)
    {
      // check whether backup directory exists already, otherwise create.
      struct stat st = {0};
      if (stat(own_histogram_backup_folder_.c_str(), &st) == -1)
      {
        mkdir(own_histogram_backup_folder_.c_str(), 0755);
        ROS_DEBUG("Created backup folder %s", own_histogram_backup_folder_.c_str());
      }
    }
    else
    {
      node_state_ = ERROR;
      ROS_ERROR("Own data: No item raw data found to work on.");
    }
  }
  else
  {
    node_state_ = ERROR;
    ROS_ERROR("Own data: Couldn't find the bgr_item pic folder %s", own_backgroundless_pic_base_folder_.c_str());
  }

  db_name_ =
  { "Amazon data", "Own data"};

  db_locations_ = vector<std::vector<std::vector<std::vector<cv::Mat> > > >(2); // histograms of the dbs
  db_locations_[0] = ama_histogram_DB_;
  db_locations_[1] = own_histogram_DB_;
  image_db_locations_ = vector<std::vector<std::vector<std::vector<cv::Mat> > > >(2); // miniature item images of the dbs
  image_db_locations_[0] = ama_image_DB_;
  image_db_locations_[1] = own_image_DB_;
  white_db_locations_ = std::vector<std::vector<std::vector<float> > >(2);
  black_db_locations_ = std::vector<std::vector<std::vector<float> > >(2);

  folder_trees_ = vector<std::vector<std::string> >(2); // folder (paths) of the db
  folder_trees_[0] = ama_item_folders_;
  folder_trees_[1] = own_item_folders_;
  image_trees_ = vector<vector<vector<string> > >(2); // picture contents (paths) of the folders
  image_trees_[0] = ama_images_in_folders_;
  image_trees_[1] = own_images_in_folders_;

  bgr_pic_folder_locations_ = vector<string>(2);
  bgr_pic_folder_locations_[0] = amazon_backgroundless_pic_base_folder_;
  bgr_pic_folder_locations_[1] = own_backgroundless_pic_base_folder_;

  histo_bck_folder_locations_ = vector<string>(2);
  histo_bck_folder_locations_[0] = amazon_histogram_backup_folder_;
  histo_bck_folder_locations_[1] = own_histogram_backup_folder_;

  for (int db = 0; db < db_locations_.size(); db++)
  {
    db_locations_[db].resize(folder_trees_[db].size());
    image_db_locations_[db].resize(folder_trees_[db].size());
    white_db_locations_[db].resize(folder_trees_[db].size());
    black_db_locations_[db].resize(folder_trees_[db].size());
    // load pictures, transform to (HSV),
    for (int i = 0; i < folder_trees_[db].size(); i++)
    {
      ROS_INFO("[%s] Load Histograms of %s ..", db_name_[db].c_str(), folder_trees_[db][i].c_str());
      db_locations_[db][i].resize(image_trees_[db][i].size());
      image_db_locations_[db][i].resize(image_trees_[db][i].size());
      white_db_locations_[db][i].resize(image_trees_[db][i].size());
      black_db_locations_[db][i].resize(image_trees_[db][i].size());
      for (int j = 0; j < image_trees_[db][i].size(); j++)
      {
        string pic_path = bgr_pic_folder_locations_[db] + folder_trees_[db][i] + "/" + image_trees_[db][i][j];
        string backup_file_path = histo_bck_folder_locations_[db] + image_trees_[db][i][j] + ".yml";
        string backup_file_path_img = histo_bck_folder_locations_[db] + image_trees_[db][i][j] + ".png";
        if (!(Helpers::file_exists(backup_file_path) && Helpers::file_exists(backup_file_path_img)))
        {
          cv::Mat img = generateRgba(cropBlack(cv::imread(pic_path, CV_LOAD_IMAGE_UNCHANGED)));

          if (img.empty())
          {
            ROS_WARN("[%s] Couldn`t read png %s", db_name_[db].c_str(), image_trees_[db][i][j].c_str());
          }
          else
          {
            ROS_DEBUG("[%s]... calculate new histograms on %s-%i", db_name_[db].c_str(), folder_trees_[db][i].c_str(),
                      j);
            std::vector<cv::Mat> rgba;
            cv::split(img, rgba);
            cv::Mat hsv_img;
            cv::cvtColor(img, hsv_img, CV_BGR2HSV);
            db_locations_[db][i][j].push_back(calculateHistograms1(hsv_img, rgba[3], 50, 60)); // [0]
            db_locations_[db][i][j].push_back(calculateHistograms2(hsv_img, rgba[3], 50, 60)); // [1]
            white_db_locations_[db][i][j] = whiteObjectProbability(hsv_img, rgba[3]);
            black_db_locations_[db][i][j] = blackObjectProbability(hsv_img, rgba[3]);
            ROS_DEBUG("[%s] Saving data to disk...", db_name_[db].c_str());
            saveHistogram(backup_file_path, db_locations_[db][i][j], white_db_locations_[db][i][j], black_db_locations_[db][i][j]);

            cv::Mat imgResized;
            int scale = 4;
            cv::resize(img, imgResized, cv::Size(256, 256 * img.rows / img.cols));
            image_db_locations_[db][i][j].push_back(imgResized);
            cv::imwrite(backup_file_path_img, imgResized);
          }
        }

        if(Helpers::file_exists(backup_file_path) && Helpers::file_exists(backup_file_path_img))
        {
          ROS_DEBUG("[%s]... %s-%i histogram from disk ", db_name_[db].c_str(), folder_trees_[db][i].c_str(), j);
          loadHistogram(backup_file_path, db_locations_[db][i][j], white_db_locations_[db][i][j], black_db_locations_[db][i][j]);
          image_db_locations_[db][i][j].push_back(cv::imread(backup_file_path_img));
        }
      }
    }
  }
  ama_histogram_DB_ = db_locations_[0];
  ama_image_DB_ = image_db_locations_[0];
  node_state_ = READY;

  ROS_DEBUG("Color histograms init time: %0.2lf sec", (((double )Helpers::timeNow() - t_start) / 1000));

  pub_myState_.publish(rosMessage(getTextForEnum(node_state_)));
}

/**
 * index files and folders in the given folder (expected to contain amazon data)
 * @param [in] baseFolder the folder to index
 * @param [out] item_folders - folder names resp. item names
 * @param [out] images_in_folders - file names of the images in the related folder (same index)
 * @return number of added folders, or -1 if not accessible address.
 */
int ColorHistogramNode::indexResources(const string& baseFolder, vector<string>& item_folders,
                                       std::vector<std::vector<std::string> >& images_in_folders)
{
  ROS_DEBUG("indexResources");

  if (access(baseFolder.c_str(), F_OK) == -1)
    return -1;

  // iterate over all subfolders where the item's background removed items are located
  DIR* pdir = opendir(baseFolder.c_str());
  struct dirent* entry = readdir(pdir);
  while (entry != NULL)
  {
    if (entry->d_type == DT_DIR && strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
    {
      string folder_Path = baseFolder + "/" + string(entry->d_name);
      vector<string> files = vector<string>();
      if (Helpers::getDir(folder_Path, files, "png") && files.size() > 0)
      {
        item_folders.push_back(string(entry->d_name));
        images_in_folders.push_back(files);
        ROS_INFO("Added item folder '%s' containing %lu images", entry->d_name, files.size());
      }
      else
      {
        ROS_WARN("Can`t open directory %s or it doesn't contain images.", folder_Path.c_str());
      }
    }
    entry = readdir(pdir); // iterate;
  }
  closedir(pdir);

  ROS_INFO("In total, added %lu item folders", item_folders.size());

  return item_folders.size();
}

/**
 * Calculates 3-channel histogram of an input picture
 * @param img the picture (in 3channels)
 * @return 3-channel histogram
 */
cv::Mat ColorHistogramNode::calculateHistograms1(cv::Mat & img, cv::Mat mask, int h_bins, int s_bins)
{
  ROS_DEBUG("calculateHistogram1");

  // calc histogram dimension-wise
  bool uniform = true;
  bool accumulate = false;

  // Using 50 bins for hue and 60 for saturation
  int histSize[] = {h_bins, s_bins};

  // hue varies from 0 to 179, saturation from 0 to 255
  float h_ranges[] = {0, 180};
  float s_ranges[] = {0, 256};
  const float* ranges[] = {h_ranges, s_ranges};

  // Use the o-th and 1-st channels
  int channels[] = {0, 1};

  // Compute the histograms:
  cv::Mat img_hist;
  cv::calcHist(&img, 1, channels, mask, img_hist, 2, histSize, ranges, uniform, accumulate);
  cv::normalize(img_hist, img_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  return img_hist;
}

cv::Mat ColorHistogramNode::calculateHistograms2(cv::Mat & img, cv::Mat mask, int h_bins, int s_bins)
{
  ROS_DEBUG("calculateHistogram2");
  
  // calc histogram dimension-wise
  bool uniform = true;
  bool accumulate = false;

  // Using 50 bins for hue and 60 for saturation
  int histSize[] = {h_bins, s_bins};

  // hue varies from 0 to 179, saturation from 0 to 255
  float h_ranges[] = {0, 180};
  float s_ranges[] = {0, 256};
  const float* ranges[] = {h_ranges, s_ranges};

  const int thresh = 40 * 255 / 100;
  cv::Mat img_hist = cv::Mat::zeros(h_bins, s_bins, CV_32F);
  for (int i = 0; i < img.rows; i++)
  {
    unsigned char *data = img.ptr<unsigned char>(i);
    unsigned char *maskData = mask.ptr<unsigned char>(i);

    for (int j = 0; j < img.cols; j++)
    {
      if (maskData[j] > 0)
      {
        unsigned char h = data[j * 3], s = data[j * 3 + 1], v = data[j * 3 + 2];
        int minSV = s < v ? s : v;
        float ratio = 1;
        if (minSV < thresh)
          ratio = ((float)minSV) / thresh;
        int hBin = h * h_bins / 180;
        int sBin = s * s_bins / 255;
        img_hist.at<float>(hBin, sBin) += ratio;
      }
    }
  }
  cv::normalize(img_hist, img_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  return img_hist;
}

void ColorHistogramNode::showIntermediateResults(cv::Mat &inputImg, cv::Mat &inputHist, cv::Mat &dbImg, cv::Mat &dbHist)
{
  vector<cv::Mat> histo_planes_input = calc_hue_sat_histogram(inputHist);
  vector<cv::Mat> histo_planes_db = calc_hue_sat_histogram(dbHist);

  // Using 50 bins for hue and 60 for saturation
  int num_hue_bins = 50;
  int num_sat_bins = 60;
  int histSize[] = {num_hue_bins, num_sat_bins};

  // Draw the histograms for H S and V
  cv::Mat input_hist_image = generate_1D_histogram(histo_planes_input);
  cv::Mat db_hist_image = generate_1D_histogram(histo_planes_db);

  // Normalize 2D histogram data to [ 0, input_hist_image.rows ]
  cv::normalize(inputHist, inputHist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
  cv::normalize(dbHist, dbHist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());

  cv::Size size(200, 200);
  cv::Mat resized_dbImg;
  cv::Mat resized_inputImg;
  cv::resize(inputImg, resized_inputImg, size);
  cv::resize(dbImg, resized_dbImg, size);

#ifndef competitionMode

  cv::imshow("Unknown object image", resized_inputImg);
  cv::imshow("Known object image", resized_dbImg);
  cv::imshow("Known object histogram2D", dbHist);
  cv::imshow("Known object histogram1D", db_hist_image);
  cv::imshow("Unknown object histogram2D", inputHist);
  cv::imshow("Unknown object histogram1D", input_hist_image);

  cv::waitKey(0);
#endif

}

/**
 * This method needs the data items list to reduce the items
 * @param req
 * @param res
 * @return
 */
bool ColorHistogramNode::setItemsInfo(tnp_feat_vision::set_items_info::Request &req,
                                      tnp_feat_vision::set_items_info::Response &res)
{
  // copying item data
  item_order_list_.clear();
  for (int i = 0; i < req.items_ids.size(); i++)
  {
    item_order_list_.push_back(req.items_ids[i].data);
  }
  sort(item_order_list_.begin(), item_order_list_.end());

  // verification on consistency
  if (ama_item_folders_.size() != item_order_list_.size())
    ROS_WARN("Amount of items doesn't match the amount of folders (%lu/ %lu)", ama_item_folders_.size(),
             item_order_list_.size());

  vector<string> tmp_order_list = item_order_list_;
  for (int i = 0; i < ama_item_folders_.size(); i++)
  {
    int pos_to_del = -1;
    for (int j = 0; j < tmp_order_list.size(); j++)
    {
      string folder = ama_item_folders_[i];
      std::transform(folder.begin(), folder.end(), folder.begin(), ::tolower);
      if (folder.compare(tmp_order_list[j]) == 0)
      {
        pos_to_del = j;
      }
    }
    tmp_order_list.erase(tmp_order_list.begin() + pos_to_del);
  }
  if (tmp_order_list.size() != 0)
  {
    ROS_ERROR("ERROR: I found no training data to %lu items", tmp_order_list.size());
    for (int i = 0; i < req.items_ids.size(); i++)
    {
      ROS_ERROR("Couldn't find images for %s", tmp_order_list[i].c_str());
    }
  }

  res.total_items.data = item_order_list_.size();
}

void mouseFuncReq(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    ColorHistogramNode *node = ((ColorHistogramNode*)userdata);
    int col = x/node->databaseViewerImgSize.width;
    node->setSelectedReqHistogram(col);
  }
}

void mouseFuncDatabase(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    ColorHistogramNode *node = ((ColorHistogramNode*)userdata);
    int row = y/node->databaseViewerImgSize.height;
    int col = x/node->databaseViewerImgSize.width;
    int item =  node->currentClassification[row + (col/node->databaseViewerNbPictureDrawn)*node->databaseViewerNbLine].second;
    int picture = col%node->databaseViewerNbPictureDrawn;
    node->setSelectedDatabaseHistogram(item, picture);
  }
}

void ColorHistogramNode::setSelectedDatabaseHistogram(int item, int picture)
{
  selectedDatabaseItem = item;
  selectedDatabasePicture = picture;
  ROS_DEBUG("redatabase item %d picture %d", item, picture);
  drawHistoDetail();
}

void ColorHistogramNode::setSelectedReqHistogram(int picture)
{
  selectedReqPicture = picture;
  ROS_DEBUG("req picture %d", picture);
  drawHistoDetail();
}

void ColorHistogramNode::drawHistoDetail()
{
  cv::Mat imgDb, histoDb, imgReq, histoReq;
  if(selectedDatabaseItem >= 0 && selectedDatabaseItem < currentDbImg.size()
      && selectedDatabasePicture >= 0 && selectedDatabasePicture < currentDbImg[selectedDatabaseItem].size())
  {
    imgDb  = currentDbImg[selectedDatabaseItem][selectedDatabasePicture][0];
    histoDb  = currentDbHisto[selectedDatabaseItem][selectedDatabasePicture][currentHistoType];
  }
  if(selectedReqPicture >= 0 && selectedReqPicture < currentReqImg.size())
  {
    imgReq  = currentReqImg[selectedReqPicture];
    histoReq  = currentReqHisto[selectedReqPicture];
  }

  cv::Size imgSize(200,200);
  int borderSize = 10;
  cv::Mat totalImg = cv::Mat::zeros(imgSize.height*3, imgSize.width*2+borderSize, CV_8UC3);
  cv::rectangle(totalImg, cv::Rect(imgSize.width,0,borderSize, totalImg.rows), cv::Scalar(255,255,255));

  if(!imgDb.empty())
  {
    cv::Mat img;
    cv::resize(imgDb, img, imgSize);
    img.copyTo(totalImg(cv::Rect(0,0,imgSize.width, imgSize.height)));
  }
  if(!imgReq.empty())
  {
    cv::Mat img;
    cv::resize(imgReq, img, imgSize);
    img.copyTo(totalImg(cv::Rect(imgSize.width+borderSize,0,imgSize.width, imgSize.height)));
  }
  if(!histoDb.empty())
  {
    cv::Mat img;
    cv::resize(histoDb, img, imgSize);
    cv::cvtColor(img, img, CV_GRAY2RGB);
    img.convertTo(img, CV_8UC3, 255);
    img = img.t();
    img.copyTo(totalImg(cv::Rect(0,imgSize.height,imgSize.width, imgSize.height)));

    std::vector<cv::Mat> histo_planes_input = calc_hue_sat_histogram(histoDb);
    cv::Mat hist_image = generate_1D_histogram(histo_planes_input);
    cv::resize(hist_image, img, imgSize);
    img.copyTo(totalImg(cv::Rect(0,2*imgSize.height,imgSize.width, imgSize.height)));
  }
  if(!histoReq.empty())
  {
    cv::Mat img;
    cv::resize(histoReq, img, imgSize);
    cv::cvtColor(img, img, CV_GRAY2RGB);
    img.convertTo(img, CV_8UC3, 255);
    img = img.t();
    img.copyTo(totalImg(cv::Rect(imgSize.width+borderSize,imgSize.height,imgSize.width, imgSize.height)));

    std::vector<cv::Mat> histo_planes_input = calc_hue_sat_histogram(histoReq);
    cv::Mat hist_image = generate_1D_histogram(histo_planes_input);
    cv::resize(hist_image, img, imgSize);
    img.copyTo(totalImg(cv::Rect(imgSize.width+borderSize,2*imgSize.height,imgSize.width, imgSize.height)));
  }

  cv::imshow("compare Histo", totalImg);
  cv::waitKey(100);
}

std::vector<int> clusterToList(const std::vector<std::pair<int, int> >& clusters, int id = -1)
{
  if(id == -1)
    id = clusters.size()-1;

  if(clusters[id].second == -1)
  {
    std::vector<int> list;
    list.push_back(clusters[id].first);
    return list;
  }
  std::vector<int> list = clusterToList(clusters, clusters[id].first);
  std::vector<int> list2 = clusterToList(clusters, clusters[id].second);
  for(int i = 0; i < list2.size(); i++)
    list.push_back(list2[i]);
  return list;
}

void ColorHistogramNode::clustering(int db, int histoType, int compareMethod)
{
  std::vector<std::vector<std::vector<cv::Mat> > >& dbHistoOrig = db_locations_[db];// [item][picture][histotype]-> histogram
  std::vector<std::vector<std::vector<cv::Mat> > >& dbImages = image_db_locations_[db];
  std::vector<std::vector<float> >& dbWhite = white_db_locations_[db];
  std::vector<std::vector<float> >& dbBlack = black_db_locations_[db];

  std::vector<float> dbWhiteMean;
  for(int i = 0; i < dbWhite.size(); i++)
  {
    float val = 0;
    for(int j = 0; j < dbWhite[i].size(); j++)
      val += dbWhite[i][j];
    val /= dbWhite[i].size();
    dbWhiteMean.push_back(val);
  }
  std::vector<float> dbBlackMean;
  for(int i = 0; i < dbBlack.size(); i++)
  {
    float val = 0;
    for(int j = 0; j < dbBlack[i].size(); j++)
      val += dbBlack[i][j];
    val /= dbBlack[i].size();
    dbBlackMean.push_back(val);
  }


  std::vector<std::vector<cv::Mat> > dbHisto(dbHistoOrig.size());
  for(int i = 0; i < dbHistoOrig.size(); i++)
  {
    dbHisto[i] = std::vector<cv::Mat>(dbHistoOrig[i].size());
    for(int j = 0; j < dbHistoOrig[i].size(); j++)
      dbHisto[i][j] = dbHistoOrig[i][j][histoType];
  }

  cv::Mat distance(2*dbHisto.size()+1, 2*dbHisto.size()+1, CV_32F);
  for(int i = 0; i < dbHisto.size(); i++)
  {
    std::vector<std::pair<double, int> > result = classify(dbHisto[i], dbHisto, compareMethod);
    for(int j = 0; j < result.size(); j++)
    {
      float white1 = dbWhiteMean[i];
      float white2 = dbWhiteMean[result[j].second];
      float black1 = dbBlackMean[i];
      float black2 = dbBlackMean[result[j].second];
      float alphaWhite = std::max(white1, white2);
      float alphaBlack = std::max(black1, black2);
      float score = result[j].first;
      score *= (1.0-alphaBlack) + alphaBlack*(1.0-std::fabs(black1-black2));
      score *= (1.0-alphaWhite) + alphaWhite*(1.0-std::fabs(white1-white2));
      distance.at<float>(i, result[j].second) = score;

    }
  }
  std::vector<bool> valid;
  std::vector<std::pair<int, int> > listClusters;
  for(int i = 0; i < dbHisto.size(); i++)
  {
    listClusters.push_back(std::make_pair(i,-1));
    valid.push_back(true);
  }

  ROS_DEBUG("clustering");
  while(true)
  {

    int maxId1 = -1, maxId2 = -1;
    float maxScore = -100000;
    for(int i = 0; i < listClusters.size(); i++)
    {
      if(valid[i])
      {
        for(int j = 0; j < listClusters.size(); j++)
        {
          if(i != j && valid[j])
          {
            float dist = distance.at<float>(i,j);
            if(dist > maxScore)
            {
              maxScore = dist;
              maxId1 = i;
              maxId2 = j;
            }
          }
        }
      }
    }
    if(maxId1 < 0)
      break;

    for(int i = 0; i < listClusters.size(); i++)
      distance.at<float>(listClusters.size(), i) = distance.at<float>(i, listClusters.size()) = (distance.at<float>(i, maxId1) + distance.at<float>(i, maxId2))/2;
    ROS_DEBUG("%d <-> %d\n", maxId1, maxId2);
    listClusters.push_back(std::make_pair(maxId1, maxId2));
    valid.push_back(true);
    valid[maxId1] = false;
    valid[maxId2] = false;
  }
  ROS_DEBUG("clustering end");
  std::vector<int> list = clusterToList(listClusters);

  ROS_DEBUG("draw classification");

  cv::Size imgSize(100, 100);
  cv::Mat totalImage = cv::Mat::zeros(imgSize.height, imgSize.width * 4, CV_8UC3);

  int nbResultDraw = 50 < list.size() ? 50 : list.size(), nbItemDraw = 4;
  int nbLine = 10;
  int nbCol = nbResultDraw / nbLine + (nbResultDraw % nbLine != 0 ? 1 : 0);
  cv::Mat totalImageDb = cv::Mat::zeros(imgSize.height * nbLine, imgSize.width * nbItemDraw * nbCol, CV_8UC3);

  ROS_DEBUG("draw classification");
  for (int id = 0; id < nbResultDraw && id < list.size(); id++)
  {
    ROS_DEBUG("draw classification %d", id);
    int col = id / nbLine;
    int row = id % nbLine;
    int j = list[id];
    for (int l = 0; l < nbItemDraw && l < dbImages[j].size(); l++)
    {
      cv::Mat imgdb;
      cv::resize(dbImages[j][l][0], imgdb, imgSize);
      imgdb.copyTo(
          totalImageDb(
              cv::Rect((l + col * nbItemDraw) * imgSize.width, row * imgSize.height, imgSize.width,
                       imgSize.height)));
    }
  }
  cv::imshow("clusters", totalImageDb);
#ifdef selfTestMode
  cv::waitKey(0);
#else
  cv::waitKey(100);
#endif

}

cv::Mat renderItemsBoard(const std::vector<std::vector<std::vector<cv::Mat> > >& dbImages, std::vector<std::pair<double, int> > results, int nbLine, cv::Size imgSize = cv::Size(100,100), int nbItemDraw = 4)
{
    int nbResultDraw = 50 < results.size() ? 50 : results.size();
    int nbCol = nbResultDraw / nbLine + (nbResultDraw % nbLine != 0 ? 1 : 0);
    cv::Mat totalImageDb = cv::Mat::zeros(imgSize.height * nbLine, imgSize.width * nbItemDraw * nbCol, CV_8UC3);

    ROS_DEBUG("draw classification");
    for (int id = 0; id < nbResultDraw && id < results.size(); id++)
    {
      ROS_DEBUG("draw classification %d", id);
      int col = id / nbLine;
      int row = id % nbLine;
      double score = results[id].first;
      int j = results[id].second;
      for (int l = 0; l < nbItemDraw && l < dbImages[j].size(); l++)
      {
        cv::Mat imgdb;
        cv::resize(dbImages[j][l][0], imgdb, imgSize);
        imgdb.copyTo(
            totalImageDb(
                cv::Rect((l + col * nbItemDraw) * imgSize.width, row * imgSize.height, imgSize.width,
                         imgSize.height)));
      }
    }

    ROS_DEBUG("draw");
    for (int j = 0; j < nbCol; j++)
    {
      for(int i = 0; i < nbLine; i++)
      {
        int id = i+j*nbLine;
        if(id >= results.size())
          continue;
        float score = results[id].first;
        cv::Mat colorMat;
        cv::applyColorMap(cv::Mat(int(score*255)*cv::Mat::ones(1,1,CV_8UC1)), colorMat, cv::COLORMAP_JET);
        unsigned char r = colorMat.ptr<unsigned char>(0)[0];
        unsigned char g = colorMat.ptr<unsigned char>(0)[1];
        unsigned char b = colorMat.ptr<unsigned char>(0)[2];
        cv::line(totalImageDb, cv::Point(j * imgSize.width * nbItemDraw, i*imgSize.height),
                 cv::Point(j * imgSize.width * nbItemDraw, (i+1)*imgSize.height), cv::Scalar(r, g, b), 2);
        cv::line(totalImageDb, cv::Point(j * imgSize.width * nbItemDraw, i*imgSize.height),
                         cv::Point((j+1) * imgSize.width * nbItemDraw, i*imgSize.height), cv::Scalar(r, g, b), 2);
      }
    }
    return totalImageDb;
}

std::vector<std::pair<double, int> > ColorHistogramNode::identifyItem(const std::vector<cv::Mat>& images, const std::vector<std::string>& considered_items, int db /*= 0*/, int histoType /*= 1*/, int compareMethod /*= 0*/)
{
    std::vector<std::vector<std::vector<cv::Mat> > >& dbHistoOrig = db_locations_[db];// [item][picture][histotype]-> histogram
    std::vector<std::vector<std::vector<cv::Mat> > >& dbImages = image_db_locations_[db];
    std::vector<std::vector<float> >& dbWhite = white_db_locations_[db];
    std::vector<std::vector<float> >& dbBlack = black_db_locations_[db];
    std::vector<bool> usedHisto(dbHistoOrig.size());

    for(int i = 0; i < folder_trees_[db].size(); i++)
    {
      usedHisto[i] = false;
      std::string name = folder_trees_[db][i];
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      for(int j = 0; j < considered_items.size(); j++)
      {
        if(name == considered_items[j])
        {
          usedHisto[i] = true;
          break;
        }
      }
    }

    std::vector<float> dbWhiteMean(dbWhite.size());
    for(int i = 0; i < dbWhite.size(); i++)
    {
      if(!usedHisto[i])
        continue;
      float val = 0;
      for(int j = 0; j < dbWhite[i].size(); j++)
        val += dbWhite[i][j];
      val /= dbWhite[i].size();
      dbWhiteMean[i] = val;
    }
    std::vector<float> dbBlackMean(dbBlack.size());
    for(int i = 0; i < dbBlack.size(); i++)
    {
      if(!usedHisto[i])
        continue;
      float val = 0;
      for(int j = 0; j < dbBlack[i].size(); j++)
        val += dbBlack[i][j];
      val /= dbBlack[i].size();
      dbBlackMean[i] = val;
    }


    ROS_DEBUG("Calculate histograms");
    vector<cv::Mat> recspace_histograms; // [histogram of pictures]
    printf("received %lu images\n", images.size());

    float whiteMean = 0, blackMean = 0;
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat img = images[i];
        if(img.empty())
        {
          ROS_ERROR("Empty input image");
          return std::vector<std::pair<double, int> >();
        }
        // ## computing rec space histograms
        std::vector<cv::Mat> rgba;
        cv::split(img, rgba);

        cv::Mat hsvImg;
        cv::cvtColor(img, hsvImg, CV_BGR2HSV);

        cv::Mat imgHisto;
        if(histoType == 0)
          imgHisto = calculateHistograms1(hsvImg, rgba[3], 50, 60);
        else imgHisto = calculateHistograms2(hsvImg, rgba[3], 50, 60);
        float blackProba = blackObjectProbability(hsvImg, rgba[3]);
        float whiteProba = whiteObjectProbability(hsvImg, rgba[3]);

        whiteMean += whiteProba;
        blackMean += blackProba;

        ROS_DEBUG("black proba : %f, white proba : %f", blackProba, whiteProba);
        recspace_histograms.push_back(imgHisto);
    }
    whiteMean /= images.size();
    blackMean /= images.size();

    std::vector<std::vector<cv::Mat> > dbHisto(dbHistoOrig.size());
    for(int i = 0; i < dbHistoOrig.size(); i++)
    {
      if(!usedHisto[i])
        continue;
      dbHisto[i] = std::vector<cv::Mat>(dbHistoOrig[i].size());
      for(int j = 0; j < dbHistoOrig[i].size(); j++)
        dbHisto[i][j] = dbHistoOrig[i][j][histoType];
    }

    ROS_DEBUG("classify");
    vector<pair<double, int> > results_classify = classify(recspace_histograms, dbHisto, compareMethod);

    for(int i = 0; i < results_classify.size(); i++)
    {
      float white1 = whiteMean;
      float white2 = dbWhiteMean[results_classify[i].second];
      float black1 = blackMean;
      float black2 = dbBlackMean[results_classify[i].second];
      float alphaWhite = std::max(white1, white2);
      float alphaBlack = std::max(black1, black2);
      double score = results_classify[i].first;
      score *= (1.0-alphaBlack) + alphaBlack*(1.0-std::fabs(black1-black2));
      score *= (1.0-alphaWhite) + alphaWhite*(1.0-std::fabs(white1-white2));
      results_classify[i].first = std::max(0.0, std::min(1.0, score));
    }

    std::sort(results_classify.begin(), results_classify.end(), [](const std::pair<double, int >& a, const std::pair<double, int >& b)
    {
      return a.first > b.first;
    });

    int nbItemDraw = 4, nbLine = 10;
    cv::Size imgSize(100, 100);
#ifndef competitionMode
    ROS_DEBUG("draw total image");
    cv::Mat totalImage = cv::Mat::zeros(imgSize.height, imgSize.width * 4, CV_8UC3);

    for (int id = 0; id < 4 && id < images.size(); id++)
    {
      cv::Mat img = images[id];

      cv::cvtColor(img, img, CV_BGRA2BGR);
      cv::resize(img, img, imgSize);
      img.copyTo(totalImage(cv::Rect(id * imgSize.width, 0, imgSize.width, imgSize.height)));
    }

    ROS_DEBUG("draw classification");
    cv::Mat totalImageDb = renderItemsBoard(dbImages, results_classify, nbLine, imgSize, nbItemDraw);
    
#endif

    databaseViewerNbPictureDrawn = nbItemDraw;
    databaseViewerNbLine = nbLine;
    databaseViewerImgSize = imgSize;

    selectedReqPicture = -1;
    selectedDatabaseItem = -1;
    selectedDatabasePicture = -1;

    currentDbHisto = dbHistoOrig;
    currentDbImg = dbImages;
    currentReqImg.clear();
    for(int i = 0; i < images.size(); i++)
    {
      cv::Mat img;
      cv::cvtColor(images[i], img, CV_RGBA2RGB);
      currentReqImg.push_back(img);
    }
    currentReqHisto = recspace_histograms;
    currentHistoType = histoType;
    currentClassification = results_classify;

#ifndef competitionMode

    cv::imshow("result unknown", totalImage);
    cv::setMouseCallback("result unknown", mouseFuncReq, this);
    cv::imshow("result known", totalImageDb);
    cv::setMouseCallback("result known", mouseFuncDatabase, this);
    #ifdef selfTestMode
        cv::waitKey(0);
    #else
        cv::waitKey(1000);
    #endif

#endif

    return results_classify;
}

/**
 * Providing some RGB pictures, images are HSV transformed and histogram is calculated.
 * Distance measures on single images and mean of item class is applied.
 * @param req rgb images and previous knowledge, if no previous knowledge provided, every item is compared.
 * @param res certainty.
 * @return error state
 */
bool ColorHistogramNode::identifyItemCallback(tnp_feat_vision::identify_item::Request &req,
                                              tnp_feat_vision::identify_item::Response &res)
{
  int db = 1;
  int histoType = 1;
  int compareMethod = 0;


  ROS_DEBUG("ColorHistogramNode identifyItemCallback received, received %lu images", req.cam_id.size());

  if (node_state_ == READY)
  {
    // # determine comparison items
    vector<string> considered_items;
    ROS_DEBUG("considered items :");
    if (req.requested_items_to_compare.size() > 0)
      for (int i = 0; i < req.requested_items_to_compare.size(); i++)
      {
        considered_items.push_back(req.requested_items_to_compare[i].data);
      }
    else
    {
      item_order_list_.clear();
      for(int i = 0; i < folder_trees_[db].size(); i++)
      {
        std::string name = folder_trees_[db][i];
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        ROS_DEBUG("%s", name.c_str());
        item_order_list_.push_back(name);
      }
      considered_items = item_order_list_; // no preference set - take all.
      ROS_DEBUG("no previous knowledge, use the full database");
    }
    sort(considered_items.begin(), considered_items.end());
    for(int i = 0; i < considered_items.size(); i++)
      ROS_DEBUG("%s", considered_items[i].c_str());

    // # calculate histograms
    ROS_DEBUG("Calculate histograms");
    vector<cv::Mat> recspace_histograms; // [histogram of pictures]
    printf("received %lu images\n", req.cam_id.size());

    std::vector<cv::Mat> images;
    for (int i = 0; i < req.cam_id.size(); i++)
    {
      cv::Mat img = Helpers::convert2OpenCV(req.rgb_data[i], sensor_msgs::image_encodings::BGRA8)->image;

      if (!img.empty())
        images.push_back(cropBlack(img));
      else
      {
        ROS_ERROR(" Faulty image from camera %s provided.", req.cam_id[i].data.c_str());
        return false;
      }
    }

    std::vector<std::pair<double, int> > results_classify = identifyItem(images, considered_items, db, histoType, compareMethod);

    if(results_classify.size() == 0)
    {
      return false;
    }

    for(int i = 0; i < results_classify.size(); i++)
    {
      std::string name = folder_trees_[db][results_classify[i].second];
      double score = results_classify[i].first;
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      ROS_DEBUG("[%d] %s : %lf", i, name.c_str(), score);

      std_msgs::Float64 ros_score;
      ros_score.data = score;
      res.confidences.push_back(ros_score);
      res.items_ids.push_back(rosMessage(name));
    }
#ifdef selfTestMode
    cv::waitKey(1000); // TODO: comment waitkey
#else
    cv::waitKey(100);
#endif
  }


  return true;
}


float compareHistWithHueShift(cv::Mat hist1, cv::Mat hist2, int method, int maxShift, double shiftPenalty)
{
  float best = 0;
  cv::Mat histMod(hist1.rows, hist1.cols, hist1.type());
  for(int i = -maxShift; i <= maxShift; i++)
  {
    if(i > 0)
    {
      hist1(cv::Rect(0,0,hist1.cols-i,hist1.rows)).copyTo(histMod(cv::Rect(i, 0, hist1.cols-i, hist1.rows)));
      hist1(cv::Rect(hist1.cols-i,0,i,hist1.rows)).copyTo(histMod(cv::Rect(0, 0, i, hist1.rows)));
    }
    else if(i < 0)
    {
      hist1(cv::Rect(-i, 0, hist1.cols+i, hist1.rows)).copyTo(histMod(cv::Rect(0,0,hist1.cols+i,hist1.rows)));
      hist1(cv::Rect(0, 0, -i, hist1.rows)).copyTo(histMod(cv::Rect(hist1.cols+i,0,-i, hist1.rows)));
    }
    else hist1.copyTo(histMod);

    float score = pow(1.0-shiftPenalty, fabs(i))*compareHist(histMod, hist2, method);
    if(score > best)
      best = score;
  }
  return best;
}


//dbHisto[item][picture]
vector<pair<double, int> > ColorHistogramNode::classify(const vector<cv::Mat> imgHisto,
                                                        const std::vector<std::vector<cv::Mat> > dbHisto, int compareMethod /*= 0*/)
{
  ROS_DEBUG("classify");
  printf("imgHisto %lu dbHisto %lu\n", imgHisto.size(), dbHisto.size());

  bool isHigherBetter = (compareMethod==CV_COMP_CORREL || compareMethod==CV_COMP_INTERSECT);

  std::vector<std::pair<double, int> > results;
  for (int i = 0; i < dbHisto.size(); i++)
  {
    if(dbHisto[i].size() == 0)
      continue;
    std::vector<double> scores;
    for (int id = 0; id < imgHisto.size(); id++)
    {
      for (int j = 0; j < dbHisto[i].size(); j++)
      {
        cv::Mat hue1 = calc_hue_sat_histogram(imgHisto[id])[0];
        cv::Mat hue2 = calc_hue_sat_histogram(dbHisto[i][j])[0];
        double distance = compareHistWithHueShift(hue1, hue2, compareMethod, hue1.cols/15, 0.02);//compareHist(hue1, hue2, compareMethod);
        scores.push_back(distance);
      }
    }
    if(isHigherBetter)
      std::sort(scores.begin(), scores.end());
    else std::sort(scores.begin(), scores.end(), std::greater<double>());
    double total = 0;
    scores.erase(scores.begin(), scores.begin() + scores.size() * 7 / 10);
    for (int j = 0; j < scores.size(); j++)
      total += scores[j];
    total /= scores.size();
    results.push_back(std::make_pair(total, i));
  }

  if(isHigherBetter)
  {
    std::sort(results.begin(), results.end(), [](const std::pair<double, int >& a, const std::pair<double, int >& b)
    {
      return a.first > b.first;
    });
  }
  else
  {
    std::sort(results.begin(), results.end(), [](const std::pair<double, int >& a, const std::pair<double, int >& b)
    {
       return a.first < b.first;
    });
  }

  return results;
}


bool ColorHistogramNode::identifyItemCallbackOLD(tnp_feat_vision::identify_item::Request &req,
                                              tnp_feat_vision::identify_item::Response &res)
{
  return true;
}


vector<pair<double, int> > ColorHistogramNode::classifyOLD(const vector<cv::Mat> imgHisto,
                                                        const std::vector<std::vector<std::vector<cv::Mat> > > dbHisto)
{
  ROS_DEBUG("classify");
  printf("imgHisto %lu dbHisto %lu\n", imgHisto.size(), dbHisto.size());

  std::vector<std::pair<double, int> > results;
  for (int i = 0; i < dbHisto.size(); i++)
  {
    std::vector<double> scores;
    for (int id = 0; id < imgHisto.size(); id++)
    {
      for (int j = 0; j < dbHisto[i].size(); j++)
      {
        double distance = compareHist(imgHisto[id], dbHisto[i][j][0], 0);
        scores.push_back(distance);
      }
    }
    std::sort(scores.begin(), scores.end());
    double total = 0;
    scores.erase(scores.begin(), scores.begin() + scores.size() * 7 / 10);
    for (int j = 0; j < scores.size(); j++)
      total += scores[j];
    total /= scores.size();
    results.push_back(std::make_pair(total, i));
  }

  std::sort(results.begin(), results.end(), [](const std::pair<double, int >& a, const std::pair<double, int >& b)
  {
    return a.first > b.first;
  });

  return results;
}

void ColorHistogramNode::selfTest()
{
  ROS_WARN("STARTING SELF TEST");

  string resourcePath = "/root/share/tnp_feature_vision/color_histogram/test_data";

  std::vector<std::string> test_item_folders = std::vector<std::string>();
  std::vector<std::vector<std::string> > test_images_in_folders = std::vector<std::vector<std::string> >();

  indexResources(resourcePath, test_item_folders, test_images_in_folders);


  float meanRank = 0;
  int nbMeasurement = 0;

  std::vector<int> listRank;

  for (int i = 0; i < test_item_folders.size(); i++)
  {
    tnp_feat_vision::identify_item ii;

    for (int j = 0; j < test_images_in_folders[i].size(); j++)
    {
      string pic_path = resourcePath + "/" + test_item_folders[i] + "/" + test_images_in_folders[i][j];
      cv::Mat img = cv::imread(pic_path, CV_LOAD_IMAGE_UNCHANGED);

      if (img.empty())
      {
        ROS_ERROR("Image read of file %s failed", pic_path.c_str());
      }
      else
      {
        if (ii.request.cam_id.size() < 4)
        {
          img = generateRgba(img);

          ii.request.cam_id.push_back(rosMessage(to_string(j)));
          ii.request.rgb_data.push_back(*cv_bridge::CvImage(std_msgs::Header(), "bgra8", img).toImageMsg());
        }
      }

      if (ii.request.cam_id.size() == 4)
      {
        std::string expectedResult = test_item_folders[i];
        std::transform(expectedResult.begin(), expectedResult.end(), expectedResult.begin(), ::tolower);

        ROS_DEBUG("Sending test data %s to colorHistogramCallback.", test_item_folders[i].c_str());
        identifyItemCallback(ii.request, ii.response);

        int rank = ii.response.items_ids.size();

        for (int k = 0; k < ii.response.items_ids.size(); k++)
        {
          ROS_INFO("RESULT: Distance for %s: %f", ii.response.items_ids[k].data.c_str(),
                   ii.response.confidences[k].data);
          if(ii.response.items_ids[k].data == expectedResult)
            rank = k;
        }

        if(rank < ii.response.items_ids.size())
        {
          listRank.push_back(rank);
          meanRank = (meanRank*nbMeasurement + rank)/(nbMeasurement+1);
          nbMeasurement++;
        }
        ROS_INFO("Expected result : %s, rank %d, meanRank %f", expectedResult.c_str(), rank, meanRank);
        float mean = 0, var = 0;
        for(int j = 0; j < listRank.size(); j++)
          mean += listRank[j];
        mean /= listRank.size();
        for(int j = 0; j < listRank.size(); j++)
          var += (listRank[j]-mean)*(listRank[j]-mean);
        var /= listRank.size();
        ROS_INFO("meanRank %f, var %f sqrtVar %f", mean, var, sqrt(var));

        ROS_DEBUG("selfTest7");
        ii = tnp_feat_vision::identify_item();
      }
    }
  }
}

std_msgs::String rosMessage(std::string txt)
{
  std_msgs::String msg;
  msg.data = txt;

  return msg;
}

static const char * EnumStrings[] = {"INIT", "READY", "ERROR"};
const char * getTextForEnum(int enumVal)
{
  return EnumStrings[enumVal];
}

int main(int argc, char **argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::init(argc, argv, "tnp_color_histogram");
  ros::NodeHandle nh;

  ColorHistogramNode colorHistogramN(nh);

#ifdef selfTestMode
  colorHistogramN.selfTest();
#endif

  ros::spin();
}
