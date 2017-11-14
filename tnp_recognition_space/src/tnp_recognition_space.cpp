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

#include "tnp_recognition_space.h"

using namespace std;

//Constructor
RecognitionSpace::RecognitionSpace(ros::NodeHandle &nh) :
    nh_(nh), it_(nh)
{

  // infrastructure check
  own_unknown_bbx_root_ = "/root/share/item_bounding_boxes/";
  own_known_bbx_root_ = "/root/share/own_bounding_boxes_known/";

  own_unknown_weights_root_ = "/root/share/item_weights/";
  own_known_weights_root_ = "/root/share/own_weights_known/";

  amazon_unknown_items_root_ = "/root/share/amazon_unknown/";
  amazon_known_items_root_ = "/root/share/amazon_data_known/";

  own_unknown_items_root_ = "/root/share/own_data_unknown/"; // will be overwritten by service callback parameter "input"
  own_known_items_root_ = "/root/share/own_data_known/";

  svm_augmented_items_own_root_ = "/root/share/tnp_svm/augmented_40items/";
  svm_target_folder_ = "/root/share/tnp_svm/for_training_pick/";

  svm_competition_items_own_root_ = "/root/share/tnp_svm/own_bgr/";
  histo_competition_items_own_root_ = "/root/share/tnp_feature_vision/color_histogram/own_data/bgr_img/";
  svm2_competition_weights_root_ = "/root/share/competition_weights/";
  svm2_competition_bbx_root_ = "/root/share/competition_bounding_boxes/";

  struct stat st = {0};
  if (stat(own_known_bbx_root_.c_str(), &st) == -1)
  {
    ROS_ERROR("Folder not found:  %s", own_known_bbx_root_.c_str());
  }

  st =
  { 0};
  if (stat(own_unknown_bbx_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(own_unknown_bbx_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(own_unknown_weights_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(own_unknown_weights_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(own_known_weights_root_.c_str(), &st) == -1)
  {
    ROS_ERROR("Folder not found:  %s", own_known_weights_root_.c_str());
  }

  st =
  { 0};
  if (stat(amazon_unknown_items_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(amazon_unknown_items_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(amazon_known_items_root_.c_str(), &st) == -1)
  {
    ROS_ERROR("Folder not found:  %s", amazon_known_items_root_.c_str());
  }

  st =
  { 0};
  if (stat(own_unknown_items_root_.c_str(), &st) == -1)
  {
    ROS_WARN("Folder not found:  %s", own_unknown_items_root_.c_str());
  }

  st =
  { 0};
  if (stat(own_known_items_root_.c_str(), &st) == -1)
  {
    ROS_ERROR("Folder not found:  %s", own_known_items_root_.c_str());
  }

  st =
  { 0};
  if (stat(svm_competition_items_own_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(svm_competition_items_own_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(svm_augmented_items_own_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(svm_augmented_items_own_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(histo_competition_items_own_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(histo_competition_items_own_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(svm2_competition_weights_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(svm2_competition_weights_root_.c_str(), 0755);
  }

  st =
  { 0};
  if (stat(svm2_competition_bbx_root_.c_str(), &st) == -1)
  {
    int ok;
    Helpers2::_mkdir(svm2_competition_bbx_root_.c_str(), 0755);
  }

  // ** publishers **
  pub_myState_ = nh_.advertise<std_msgs::String>("/tnp_recognition_space/state", 2);

  // ** subscriptions **
  sub_myState_ = nh_.subscribe("/tnp_recognition_space/state", 100, &RecognitionSpace::stateCallback, this);

  // camera L - left
  sub_cam_L_RGB_ = it_.subscribe("/camera_L/" + IMG_RGB, 1, &RecognitionSpace::camera_L_RGBcallback, this);
  sub_cam_L_Depth_ = it_.subscribe("/camera_L/" + IMG_D, 1, &RecognitionSpace::camera_L_DepthCallback, this);
  sub_cam_L_CamInfo_ = nh_.subscribe("/camera_L/" + IMG_INFO, 1, &RecognitionSpace::camera_L_CameraInfoCallback, this);
  sub_cam_L_DepthInfo_ = nh_.subscribe("/camera_L/" + DEPTH_INFO, 1, &RecognitionSpace::camera_L_DepthInfoCallback,
                                       this);
  // camera C - center
  sub_cam_C_RGB_ = it_.subscribe("/camera_C/" + IMG_RGB, 1, &RecognitionSpace::camera_C_RGBcallback, this);
  sub_cam_C_Depth_ = it_.subscribe("/camera_C/" + IMG_D, 1, &RecognitionSpace::camera_C_DepthCallback, this);
  sub_cam_C_CamInfo_ = nh_.subscribe("/camera_C/" + IMG_INFO, 1, &RecognitionSpace::camera_C_CameraInfoCallback, this);
  sub_cam_C_DepthInfo_ = nh_.subscribe("/camera_C/" + DEPTH_INFO, 1, &RecognitionSpace::camera_C_DepthInfoCallback,
                                       this);
  // camera R - right
  sub_cam_R_RGB_ = it_.subscribe("/camera_r/" + IMG_RGB, 1, &RecognitionSpace::camera_R_RGBcallback, this);
  sub_cam_R_Depth_ = it_.subscribe("/camera_r/" + IMG_D, 1, &RecognitionSpace::camera_R_DepthCallback, this);
  sub_cam_R_CamInfo_ = nh_.subscribe("/camera_r/" + IMG_INFO, 1, &RecognitionSpace::camera_R_CameraInfoCallback, this);
  sub_cam_R_DepthInfo_ = nh_.subscribe("/camera_r/" + DEPTH_INFO, 1, &RecognitionSpace::camera_R_DepthInfoCallback,
                                       this);
  // camera B - base
  sub_cam_B_RGB_ = it_.subscribe("/camera_B/" + IMG_RGB, 1, &RecognitionSpace::camera_B_RGBcallback, this);
  sub_cam_B_Depth_ = it_.subscribe("/camera_B/" + IMG_D, 1, &RecognitionSpace::camera_B_DepthCallback, this);
  sub_cam_B_CamInfo_ = nh_.subscribe("/camera_B/" + IMG_INFO, 1, &RecognitionSpace::camera_B_CameraInfoCallback, this);
  sub_cam_B_DepthInfo_ = nh_.subscribe("/camera_B/" + DEPTH_INFO, 1, &RecognitionSpace::camera_B_DepthInfoCallback,
                                       this);

  // camera E - end effector
  sub_cam_E_RGB_ = it_.subscribe("/camera/rgb/image_raw", 1, &RecognitionSpace::camera_E_RGBcallback, this);
  sub_cam_E_Depth_ = it_.subscribe("/camera/" + IMG_D, 1, &RecognitionSpace::camera_E_DepthCallback, this);
  sub_cam_E_CamInfo_ = nh_.subscribe("/camera/rgb/camera_info", 1, &RecognitionSpace::camera_E_CameraInfoCallback,
                                     this);
  sub_cam_E_DepthInfo_ = nh_.subscribe("/camera/" + DEPTH_INFO, 1, &RecognitionSpace::camera_E_DepthInfoCallback, this);

  // camera T - top view
  sub_cam_T_RGB_ = it_.subscribe("/camera_T/" + IMG_RGB, 1, &RecognitionSpace::camera_T_RGBcallback, this);
  sub_cam_T_Depth_ = it_.subscribe("/camera_T/" + IMG_D, 1, &RecognitionSpace::camera_T_DepthCallback, this);
  sub_cam_T_CamInfo_ = nh_.subscribe("/camera_T/" + IMG_INFO, 1, &RecognitionSpace::camera_T_CameraInfoCallback, this);
  sub_cam_T_DepthInfo_ = nh_.subscribe("/camera_T/" + DEPTH_INFO, 1, &RecognitionSpace::camera_T_DepthInfoCallback,
                                       this);
  // service servers
  srv_recognize_items_ = nh_.advertiseService("tnp_recognition_space/recognize_items",
                                              &RecognitionSpace::recognizeItemsCallback, this);
  srv_record_rec_space_ = nh_.advertiseService("tnp_recognition_space/record_scene",
                                               &RecognitionSpace::recordSceneCallback, this);
  srv_get_bounding_box_ = nh_.advertiseService("tnp_recognition_space/get_bounding_box",
                                               &RecognitionSpace::getBoundingBoxCallback, this);
  srv_record_empty_rs_ = nh_.advertiseService("tnp_recognition_space/record_empty_rs",
                                              &RecognitionSpace::recordEmptyRSCallback, this);
  srv_record_item_ = nh_.advertiseService("tnp_recognition_space/record_item", &RecognitionSpace::recordItemCallback,
                                          this);
  srv_recording_items_ = nh_.advertiseService("tnp_recognition_space/recording_items",
                                              &RecognitionSpace::recordingItemsCallback, this);
  srv_set_items_info_ = nh_.advertiseService("tnp_recognition_space/set_items_info",
                                             &RecognitionSpace::setItemsCallback, this);
  srv_prepare_competition_files_ = nh_.advertiseService("tnp_recognition_space/create_competition_files",
                                                        &RecognitionSpace::createCompetitionFiles, this);

  // service subscriptions
  srvc_background_removal_ = nh_.serviceClient<tnp_feat_vision::remove_background>(
      "/tnp_cloud_matching/remove_background");
  srvc_cloud_matching_ = nh_.serviceClient<tnp_feat_vision::cloud_matching>("/tnp_cloud_matching/identify_item");
  srvc_svm_matching_ = nh_.serviceClient<tnp_svm::IdentifyItem>("/tnp_svm/identify_item");
  srvc_color_histogram_ = nh_.serviceClient<tnp_feat_vision::identify_item>("tnp_color_histogram/identify_item");

  srvc_rgbd_recording_ = nh_.serviceClient<tnp_feat_vision::cloud_matching>("/tnp_record_rgbd_frames/save");
  srvc_set_rs_background_ = nh_.serviceClient<tnp_feat_vision::set_rs_background>(
      "/tnp_cloud_matching/set_rs_background");

  srvc_get_bounding_box_ = nh_.serviceClient<tnp_feat_vision::get_bounding_box>("/tnp_cloud_matching/get_bounding_box");

  srvc_shutter_ = nh_.serviceClient<tnp_end_effector::OpenShutterCloseTheRest>(
      "/tnp_shutters/open_shutter_close_the_rest");
  srvc_led_controller_set_ = nh_.serviceClient<tnp_led_control::set_led_intensity>(
      "/tnp_led_control/set_led_intensity");
  srvc_led_controller_get_ = nh_.serviceClient<tnp_led_control::get_led_intensity>(
      "/tnp_led_control/get_led_intensity");

  // ----- extended node communication for recording new items
  srvc_kuka_motion_rotation_ = nh_.serviceClient<tnp_kuka_motion::goToRecSpace>("/tnp_kuka_motion/goToRecSpace");
  srvc_kuka_led_ = nh_.serviceClient<iiwa_msgs::LED>("/iiwa/configuration/LED");
  srvc_suction_ = nh_.serviceClient<tnp_end_effector::Suction>("/tnp_end_effector/suction");
  srvc_suction_contact_ = nh_.serviceClient<tnp_end_effector::ItemSuctioned>("/tnp_end_effector/suction_status");
  srvc_suction_actuator_ = nh_.serviceClient<tnp_end_effector::LinActuatorSuction>(
      "/tnp_end_effector/lin_actuator_suction");
  srvc_w_sensors_start_ = nh_.serviceClient<tnp_weight_events::GetReadyForPick>(
      "/tnp_weight_events/get_ready_for_pick");
  srvc_w_sensors_end_ = nh_.serviceClient<tnp_optoforce::GetWeightChangeSinceReady>(
      "/optoforce_bin_A/get_weight_change_since_ready");

  previous_illumination_settings_ = vector<std_msgs::Int16>(7);
  tmp_illumination_settings_ = vector<std_msgs::Int16>(7);
  // set to illumination configuration, see in header

  // check if cameras are already online 
  num_cameras_online_ = camerasOnline();
  if (num_cameras_online_ == 4)
  {
    pub_myState_.publish(rosMessage("All cameras are present in recognition space. Fine"));
  }
  else
  {
    pub_myState_.publish(
        rosMessage(
            "Only " + to_string(num_cameras_online_)
                + " cameras are present in recognition space yet. Are we doomed?"));
    // TODO: Camera not available at node start. Here still acceptable?
  }

  // set Realsense cameras to manual exposure
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse resp;
  dynamic_reconfigure::IntParameter man_param;
  dynamic_reconfigure::IntParameter exp_param;
  dynamic_reconfigure::Config conf_on;

  man_param.name = "color_enable_auto_exposure";
  man_param.value = 0;
  exp_param.name = "color_exposure";
  exp_param.value = 1200;
  conf_on.ints.push_back(man_param);
  conf_on.ints.push_back(exp_param);
  req.config = conf_on;

  ros::service::call("/camera_L/driver/set_parameters", req, resp);
  ros::service::call("/camera_C/driver/set_parameters", req, resp);
  ros::service::call("/camera_r/driver/set_parameters", req, resp);
  ros::service::call("/camera_B/driver/set_parameters", req, resp);

  // default recognition space configuration at node start: end-effector
  IRProjectorOn("E");
}

/**
 * Init node, when required data is available. 
 * Executed code need to be stateless and reversible.
 * @return
 */
bool RecognitionSpace::init()
{
  // ####   sanity check
  // check wheter set_item_info was previously called.
  if (item_order_list_.size() == 0)
  {
    ROS_ERROR("item_order_list empty. Call initial_setup");
    return false;
  }
  // ###############################3

  // filter unknown items from items list
  unknown_items_list_.clear();
  for (int i = 0; i < item_order_list_.size(); i++)
  {
    if (item_order_list_[i].second == false)
    {
      unknown_items_list_.push_back(item_order_list_[i].first);
      ROS_INFO("Unknown item %i: %s", i, item_order_list_[i].first.c_str());
    }
  }

  return true;
}

/**
 * Check how many camera topics still have an active publisher
 * @return amount of active camera data publishers
 */
int RecognitionSpace::camerasOnline()
{
  int camL_online = (int)hasPublisher("/camera_L/" + IMG_INFO);
  ROS_INFO("Recognition space camera L status:%d", camL_online);
  int camC_online = (int)hasPublisher("/camera_C/" + IMG_INFO);
  ROS_INFO("Recognition space camera C status:%d", camC_online);
  int camR_online = (int)hasPublisher("/camera_r/" + IMG_INFO);
  ROS_INFO("Recognition space camera R status:%d", camR_online);
  int camB_online = (int)hasPublisher("/camera_B/" + IMG_INFO);
  ROS_INFO("Recognition space camera B status:%d", camB_online);

  return camL_online + camC_online + camR_online + camB_online; // + camT_online + camE_online;
}

/**
 * Switch IR projectors of a camera ON
 * @return successful submission of the reconfiguration
 */
bool RecognitionSpace::IRProjectorOn(std::string camera)
{
  tnp_end_effector::OpenShutterCloseTheRest osctr;
  osctr.request.camera_id = rosMessage(camera);

  return srvc_shutter_.call(osctr);
}

/**
 * Switch IR projector of a camera OFF
 * @return successful submission of the reconfiguration
 */
bool RecognitionSpace::IRProjectorOff(std::string camera)
{
  // Functionality was implemented in IRProjectorON
  return true;
}

/**
 * Clears the saved images in this instance to make sure we don't submit outdated data
 */
void RecognitionSpace::resetImageMemory(string cameraID)
{
  if (cameraID.compare("L") == 0)
  {
    camL_bool_rgb_raw_ = 0;
    camL_bool_d_raw_ = 0;
    camL_bool_rgb_info_ = 0;
    camL_bool_d_info_ = 0;
  }
  else if (cameraID.compare("C") == 0)
  {
    camC_bool_rgb_raw_ = 0;
    camC_bool_d_raw_ = 0;
    camC_bool_rgb_info_ = 0;
    camC_bool_d_info_ = 0;
  }
  else if (cameraID.compare("R") == 0)
  {
    camR_bool_rgb_raw_ = 0;
    camR_bool_d_raw_ = 0;
    camR_bool_rgb_info_ = 0;
    camR_bool_d_info_ = 0;
  }
  else if (cameraID.compare("B") == 0)
  {
    camB_bool_rgb_raw_ = 0;
    camB_bool_d_raw_ = 0;
    camB_bool_rgb_info_ = 0;
    camB_bool_d_info_ = 0;
  }
  else if (cameraID.compare("E") == 0)
  {
    camE_bool_rgb_raw_ = 0;
    camE_bool_d_raw_ = 0;
    camE_bool_rgb_info_ = 0;
    camE_bool_d_info_ = 0;
  }
  else if (cameraID.compare("T") == 0)
  {
    camT_bool_rgb_raw_ = 0;
    camT_bool_d_raw_ = 0;
    camT_bool_rgb_info_ = 0;
    camT_bool_d_info_ = 0;
  }
}

/**
 * Checks whether the rosnode has knowledge of active publishers to a topic
 */
bool RecognitionSpace::hasPublisher(string response_topic)
{
  std::string node_name = ros::this_node::getName();
  XmlRpc::XmlRpcValue request(node_name);
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  bool success = ros::master::execute("getSystemState", request, response, payload, false);

  ROS_ASSERT(payload.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(payload.size() == 3);

  bool published = false;
  bool subscribed = false;

  XmlRpc::XmlRpcValue & publishers = payload[0];
  ROS_ASSERT(publishers.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < publishers.size(); i++)
  {
    XmlRpc::XmlRpcValue & topic = publishers[i];
    ROS_ASSERT(topic.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(topic[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string topic_name = topic[0];
    if (topic_name == response_topic)
    {
      published = true;
    }
  }

  return published;
}

/*-------------------------------------------------------------------------------------*/
/// topics callbacks
/*-------------------------------------------------------------------------------------*/

// self state
void RecognitionSpace::stateCallback(const std_msgs::String &msg)
{
  ROS_INFO("rec_space_state: %s", msg.data.c_str());
}

bool RecognitionSpace::setItemsCallback(tnp_recognition_space::set_items_info::Request &req,
                                        tnp_recognition_space::set_items_info::Response &res)
{
  ROS_DEBUG("setItemsCallback - item list contains %lu items", req.items_ids.size());
  item_order_list_.clear();

  for (int i = 0; i < req.items_ids.size(); i++)
  {
    ROS_DEBUG("Pushback item list: %s, known: %s", req.items_ids[i].data.c_str(), boolToString(req.known[i]).c_str());
    item_order_list_.push_back(make_pair(req.items_ids[i].data, req.known[i]));
  }

  res.num_items_received.data = item_order_list_.size();

  init();

  return true;
}
/**
 * Selecting files for the competition classifiers. Requires previous knowledge of items_list and ideally unknown objects' data 
 * @param req file path of the unknown items
 * @param res
 * @return
 */
bool RecognitionSpace::createCompetitionFiles(tnp_recognition_space::prepare_competition_files::Request &req,
                                              tnp_recognition_space::prepare_competition_files::Response &res)
{
  ROS_INFO("createCompetitionFiles");

  // check whether set_item_info was called previously
  if (item_order_list_.size() == 0)
  {
    ROS_ERROR("item_order_list empty. Call initial_setup && recognizing objects");
    return false;
  }

  // set img source path from req input.
  if (!req.input.data.empty()) // used as source image root dir
  {
    if (access(req.input.data.c_str(), F_OK) == -1) // folder exists
    {
      ROS_ERROR("Given image source directory %s doesn't exist.", req.input.data.c_str());
      return false;
    }
    else
    {
      own_unknown_items_root_ = req.input.data;
    }
  }
  ROS_INFO("Using image source %s", own_unknown_items_root_.c_str());

  // copying known items
  vector<string> known_items = vector<string>();
  Helpers2::getDir(own_known_items_root_, known_items);

  ROS_DEBUG("Known items stored on disk: %lu", known_items.size());
  for (int i = 0; i < known_items.size(); i++) // iterate over all folder in destination
  {
    string known_toLower = known_items[i];
    bool found = false;
    bool inOrderList = false;

    // search in item list for matching folders to known items
    ROS_DEBUG("Item Order list size: %lu", item_order_list_.size());
    for (int k = 0; k < item_order_list_.size(); k++)
    {
      // in order list
      if (known_toLower.compare(item_order_list_[k].first) == 0)
      {
        ROS_DEBUG("FOUND Known Item in order list %s", known_toLower.c_str());
        inOrderList = true;
        break;
      }
    }

    if (inOrderList)
    {
      for (string unknown : unknown_items_list_) // comparing items from order list with unknown
      {
        string unknown_toLower = unknown;
        transform(known_toLower.begin(), known_toLower.end(), known_toLower.begin(), ::tolower);
        transform(unknown_toLower.begin(), unknown_toLower.end(), unknown_toLower.begin(), ::tolower);

        if (known_toLower.compare(unknown_toLower) == 0)
        {
          found = true;
          break;
        }
      }

      if (!found) // if item is not found on disk
      {
        int ok =
            system(
                (string("cp -R -f ") + own_known_items_root_ + known_items[i] + "* " + histo_competition_items_own_root_).c_str());
        if (ok != 0)
          ROS_WARN("not copied #5");
        else
          ROS_DEBUG("Histo: Copied previous knowledge of %s", known_toLower.c_str());

        ok = system(
            (string("cp -R -f ") + svm_augmented_items_own_root_ + known_toLower + "* " + svm_target_folder_).c_str());
        if (ok != 0)
          ROS_WARN("not copied #6");
        else
          ROS_DEBUG("SVM: Copied previous knowledge of %s", known_toLower.c_str());

        ok =
            system(
                (string("cp -R -f ") + own_known_weights_root_ + known_items[i] + "* " + svm2_competition_weights_root_).c_str());
        if (ok != 0)
          ROS_WARN("not copied #7");
        else
          ROS_DEBUG("SVM2: Copied previous knowledge of %s", known_toLower.c_str());

        ok = system(
            (string("cp -R -f ") + own_known_bbx_root_ + known_items[i] + "* " + svm2_competition_bbx_root_).c_str());
        if (ok != 0)
          ROS_WARN("not copied #8");
        else
          ROS_DEBUG("SVM2: Copied previous knowledge of %s", known_toLower.c_str());
      }
    }
  }

  // plain copying of new unknown data
  int ok = system((string("cp -R -f ") + own_unknown_items_root_ + "* " + histo_competition_items_own_root_).c_str());
  if (ok != 0)
    ROS_WARN("not copied #1");

  ok = system((string("cp -R -f ") + own_unknown_items_root_.c_str() + "* " + svm_competition_items_own_root_).c_str());
  if (ok != 0)
    ROS_WARN("not copied #2");

  ok = system(
      (string("cp -R -f ") + own_unknown_weights_root_.c_str() + "* " + svm2_competition_weights_root_).c_str());
  if (ok != 0)
    ROS_WARN("not copied #3");

  ok = system((string("cp -R -f ") + own_unknown_bbx_root_.c_str() + "* " + svm2_competition_bbx_root_).c_str());
  if (ok != 0)
    ROS_WARN("not copied #4");

  return true;
}

// camera left
void RecognitionSpace::camera_L_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camL_rgb_raw_ = msg;
  camL_bool_rgb_raw_++;
}
void RecognitionSpace::camera_L_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camL_d_raw_ = msg;
  camL_bool_d_raw_++;
}
void RecognitionSpace::camera_L_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camL_rgb_info_ = msg;
  camL_bool_rgb_info_++;
}
void RecognitionSpace::camera_L_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camL_depth_info_ = msg;
  camL_bool_d_info_++;
}

// camera center
void RecognitionSpace::camera_C_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camC_rgb_raw_ = msg;
  camC_bool_rgb_raw_++;
}
void RecognitionSpace::camera_C_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camC_d_raw_ = msg;
  camC_bool_d_raw_++;
}
void RecognitionSpace::camera_C_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camC_rgb_info_ = msg;
  camC_bool_rgb_info_++;
}
void RecognitionSpace::camera_C_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camC_depth_info_ = msg;
  camC_bool_d_info_++;
}
// camera right
void RecognitionSpace::camera_R_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camR_rgb_raw_ = msg;
  camR_bool_rgb_raw_++;
}
void RecognitionSpace::camera_R_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camR_d_raw_ = msg;
  camR_bool_d_raw_++;
}
void RecognitionSpace::camera_R_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camR_rgb_info_ = msg;
  camR_bool_rgb_info_++;
}
void RecognitionSpace::camera_R_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camR_depth_info_ = msg;
  camR_bool_d_info_++;
}
// camera base
void RecognitionSpace::camera_B_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camB_rgb_raw_ = msg;
  camB_bool_rgb_raw_++;
}
void RecognitionSpace::camera_B_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camB_d_raw_ = msg;
  camB_bool_d_raw_++;
}
void RecognitionSpace::camera_B_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camB_rgb_info_ = msg;
  camB_bool_rgb_info_++;
}
void RecognitionSpace::camera_B_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camB_depth_info_ = msg;
  camB_bool_d_info_++;
}

// camera base
void RecognitionSpace::camera_E_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camE_rgb_raw_ = msg;
  camE_bool_rgb_raw_++;
}
void RecognitionSpace::camera_E_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camE_d_raw_ = msg;
  camE_bool_d_raw_++;
}
void RecognitionSpace::camera_E_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camE_rgb_info_ = msg;
  camE_bool_rgb_info_++;
}
void RecognitionSpace::camera_E_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camE_depth_info_ = msg;
  camE_bool_d_info_++;
}

// camera top
void RecognitionSpace::camera_T_RGBcallback(const sensor_msgs::ImageConstPtr &msg)
{
  camT_rgb_raw_ = msg;
  camT_bool_rgb_raw_++;
}
void RecognitionSpace::camera_T_DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  camT_d_raw_ = msg;
  camT_bool_d_raw_++;
}
void RecognitionSpace::camera_T_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camT_rgb_info_ = msg;
  camT_bool_rgb_info_++;
}
void RecognitionSpace::camera_T_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  camT_depth_info_ = msg;
  camT_bool_d_info_++;
}

/**
 * Saving the current the LED lamps configuration
 * @return success if saving all 7 values was successful
 */
bool RecognitionSpace::saveCurrentIlluminationSettings()
{
  bool success = true;
  stringstream ss;
  tnp_led_control::get_led_intensity gli;

  for (int i = 1; i < 8; i++)
  {
    gli.request.target_led_num.data = i;

    if (srvc_led_controller_get_.call(gli))
    {
      previous_illumination_settings_[i - 1] = gli.response.current_intensity;
      ss << to_string(gli.response.current_intensity.data) << "; ";
    }
    else
    {
      std_msgs::Int16 unknown;
      unknown.data = -1;
      previous_illumination_settings_[i - 1] = unknown;
      ROS_WARN("Could not get current LED lamp %i's intensity setting. Set value to 'unknown'", i);
      success = false;
    }
  }

  ROS_ERROR("Saved current illumination state as : %s", ss.str().c_str());

  return success;
}

bool RecognitionSpace::setIllumination(vector<int> intensities_int)
{
  ROS_DEBUG("setIllumination _ int...");

  vector<std_msgs::Int16> intensities_ros;
  for (int i = 0; i < intensities_int.size(); i++)
  {
    std_msgs::Int16 tmp_int16;
    tmp_int16.data = (int16_t)intensities_int[i];
    intensities_ros.push_back(tmp_int16);
  }

  return setIllumination(intensities_ros);
}

/**
 * Setting the LED lamps. Providing only one value in the vector results in setting all LED to that value.
 * Providing more values results in setting cam[i_vector] to value[i_vector]
 * @param intensities in a vector from up to 7 values.
 * @return success
 */
bool RecognitionSpace::setIllumination(vector<std_msgs::Int16> intensities)
{

  ROS_DEBUG("setIllumination _ int16... (input size %lu)", intensities.size());
  stringstream ss;
  bool success = true;

  if (intensities.size() == 1)
    intensities.resize(7, intensities[0]); // if only one value provided, expand array to 7 times that value

  tnp_led_control::set_led_intensity sli;

  for (int i = 0; i < intensities.size(); i++)
  {
    ss << to_string(intensities[i].data);
    // only calls function for valid and changed values
    if (intensities[i].data != -1 && intensities[i].data != tmp_illumination_settings_[i].data)
    {
      sli.request.target_led_num.data = i + 1;
      sli.request.target_intensity = intensities[i];
      srvc_led_controller_set_.call(sli);

      if (!srvc_led_controller_set_.call(sli) || sli.response.succeeded == false)
      {
        ROS_WARN("Could not set LED lamp %i to intensity %s. Response service: %s", i + 1,
                 to_string((int )intensities[i].data).c_str(), boolToString(sli.response.succeeded).c_str());
        ss << "X";
        success = false;
      }
      else
      {
        ss << "!";
        tmp_illumination_settings_[i] = intensities[i];
      }
    }
    ss << ", ";
  }
  ROS_DEBUG("Set illumination to %s", ss.str().c_str());
  return success;
}

/**
 * Checks whether all 4 frames for the given camera has arrived yet
 * @param camID the camera
 * @param withDepth also considering depth
 * @return bool all frames present.
 */
bool RecognitionSpace::checkCompleteness(string camID, bool withDepth)
{

  int thres = 1; // the first picture doesn't contain depth.

  if (camID.compare("L") == 0)
  {
    if (camL_bool_rgb_raw_ > thres && camL_bool_rgb_info_ > thres
        && (!withDepth || (camL_bool_d_raw_ > thres && camL_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  else if (camID.compare("C") == 0)
  {
    if (camC_bool_rgb_raw_ > thres && camC_bool_rgb_info_ > thres
        && (!withDepth || (camC_bool_d_raw_ > thres && camC_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  else if (camID.compare("R") == 0)
  {
    if (camR_bool_rgb_raw_ > thres && camR_bool_rgb_info_ > thres
        && (!withDepth || (camR_bool_d_raw_ > thres && camR_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  else if (camID.compare("B") == 0)
  {
    if (camB_bool_rgb_raw_ > thres && camB_bool_rgb_info_ > thres
        && (!withDepth || (camB_bool_d_raw_ > thres && camB_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  else if (camID.compare("E") == 0)
  {
    if (camE_bool_rgb_raw_ > thres && camE_bool_rgb_info_ > thres
        && (!withDepth || (camE_bool_d_raw_ > thres && camE_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  else if (camID.compare("T") == 0)
  {
    if (camT_bool_rgb_raw_ > thres && camT_bool_rgb_info_ > thres
        && (!withDepth || (camT_bool_d_raw_ > thres && camT_bool_d_info_ > thres)))
    {
      return true;
    }
  }
  return false;
}

string boolToString(bool in)
{
  return in ? "true" : "false";
}

/**
 * waits until either all frames have arrived for that camera, or the time is up
 * @param cameraID the camera
 * @param seconds timeout time
 * @param withDepth waits also for depth stream frames to arrive
 */
bool RecognitionSpace::blockingWaitFor(string cameraID, double seconds, bool withDepth)
{
  long start = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
  long now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

  while (now - start < seconds * 1000)
  {
    ros::spinOnce();
    if (checkCompleteness(cameraID, withDepth))
    {
      ROS_INFO("Time to receive all frames for camera_%s: %s ms", cameraID.c_str(), to_string(now - start).c_str());
      return true;
    }

    now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
  }

  ROS_INFO("Timeout after %0.1lf seconds waiting for camera_%s`s frames", seconds, cameraID.c_str());
  return false;
}

/**
 * Collects an image from all cameras
 * Using an illumination setting
 * Using optional with shuttering the IR projectors
 * @param withDepth - activates shutters and delivers depth stream
 * @param withEEcam - also provides image material from the end effector and top camera
 */
void RecognitionSpace::retrieveRecognitionSpaceImages(bool withDepth, bool withEEcam,
                                                      vector<vector<int> > illuminationConfigurations)
{
  ROS_DEBUG("RetrievingRecognitionSpaceImages...");
  double timeOutTime = 0.7; // in seconds
  stringstream status;
  long t_start = timeNow();

  if (withEEcam) // optionally for recognition space registration
  {
    ROS_DEBUG(". using end effector and top cam");

    setIllumination(illuminationConfigurations[0]);
    if (withDepth)
      IRProjectorOn("E") ? status << "E->ON.." : status << "E->NOT_ON..";
    ros::Duration(0.3).sleep();

    resetImageMemory("E");
    if (blockingWaitFor("E", timeOutTime, withDepth))
    {
      tmp_cam_id_.push_back(rosMessage("E"));
      tmp_rgb_data_.push_back(*camE_rgb_raw_);
      tmp_rgb_info_data_.push_back(*camE_rgb_info_);

      if (withDepth)
      {
        tmp_depth_data_.push_back(*camE_d_raw_);
        tmp_depth_info_data_.push_back(*camE_depth_info_);
      }
    }

    if (withDepth)
      IRProjectorOn("T") ? status << "T->ON.." : status << "T->NOT_ON..";
    ros::Duration(0.3).sleep();

    resetImageMemory("T");
    if (blockingWaitFor("T", timeOutTime, withDepth))
    {
      tmp_cam_id_.push_back(rosMessage("T"));
      tmp_rgb_data_.push_back(*camT_rgb_raw_);
      tmp_rgb_info_data_.push_back(*camT_rgb_info_);

      if (withDepth)
      {
        tmp_depth_data_.push_back(*camT_d_raw_);
        tmp_depth_info_data_.push_back(*camT_depth_info_);
      }
    }
  }

// take frame from left camera
  setIllumination(illuminationConfigurations[1]);

  if (withDepth)
    IRProjectorOn("L") ? status << "L->ON.." : status << "L->NOT_ON..";
  ros::Duration(0.3).sleep();

  resetImageMemory("L");
  if (blockingWaitFor("L", timeOutTime, withDepth))
  {
    tmp_cam_id_.push_back(rosMessage("L"));
    tmp_rgb_data_.push_back(*camL_rgb_raw_);
    tmp_rgb_info_data_.push_back(*camL_rgb_info_);

    if (withDepth)
    {
      tmp_depth_data_.push_back(*camL_d_raw_);
      tmp_depth_info_data_.push_back(*camL_depth_info_);
    }
  }

// take frame from center camera
  setIllumination(illuminationConfigurations[2]);

  if (withDepth)
    IRProjectorOn("C") ? status << "C->ON.." : status << "C->NOT_ON..";
  ros::Duration(0.3).sleep();

  resetImageMemory("C");
  if (blockingWaitFor("C", timeOutTime, withDepth))
  {
    tmp_cam_id_.push_back(rosMessage("C"));
    tmp_rgb_data_.push_back(*camC_rgb_raw_);
    tmp_rgb_info_data_.push_back(*camC_rgb_info_);

    if (withDepth)
    {
      tmp_depth_data_.push_back(*camC_d_raw_);
      tmp_depth_info_data_.push_back(*camC_depth_info_);
    }
  }

// take frame from right camera
  setIllumination(illuminationConfigurations[3]);

  if (withDepth)
    IRProjectorOn("R") ? status << "R->ON.." : status << "R->NOT_ON..";
  ros::Duration(0.3).sleep();

  resetImageMemory("R");
  if (blockingWaitFor("R", timeOutTime, withDepth))
  {
    tmp_cam_id_.push_back(rosMessage("R"));
    tmp_rgb_data_.push_back(*camR_rgb_raw_);
    tmp_rgb_info_data_.push_back(*camR_rgb_info_);

    if (withDepth)
    {
      tmp_depth_data_.push_back(*camR_d_raw_);
      tmp_depth_info_data_.push_back(*camR_depth_info_);
    }
  }

// take frame from base camera
  setIllumination(illuminationConfigurations[4]);

  if (withDepth)
    IRProjectorOn("B") ? status << "B->ON.." : status << "B->NOT_ON..";
  ros::Duration(0.3).sleep();

  resetImageMemory("B");
  if (blockingWaitFor("B", timeOutTime, withDepth))
  {
    tmp_cam_id_.push_back(rosMessage("B"));
    tmp_rgb_data_.push_back(*camB_rgb_raw_);
    tmp_rgb_info_data_.push_back(*camB_rgb_info_);

    if (withDepth)
    {
      tmp_depth_data_.push_back(*camB_d_raw_);
      tmp_depth_info_data_.push_back(*camB_depth_info_);
    }
  }

// switch on EE camera projector again
  if (withDepth)
    IRProjectorOn("E") ? status << "EE->ON.." : status << "EE->NOT_ON.";

  pub_myState_.publish(rosMessage(status.str()));
// setIllumination(previous_illumination_settings_); // NOTE: Competition speed up

  ROS_DEBUG("Fetching recognition space images time: %.2lf sec", (((double )timeNow() - t_start) / 1000));
}

/**
 * Stores recognition space pictures to disk
 */
void RecognitionSpace::storingCurrentImages(bool withDepth, string item_name /*= ""*/)
{
// storing pictures as well
  if (!COMPETITION || (COMPETITION && item_name.empty()))
  {
    tnp_feat_vision::cloud_matching clm_srv;
    clm_srv.request.target_id.data = item_name;
    clm_srv.request.cam_id = tmp_cam_id_;
    clm_srv.request.rgb_data = tmp_rgb_data_;
    clm_srv.request.rgb_info_data = tmp_rgb_info_data_;

    if (withDepth)
    {
      clm_srv.request.depth_data = tmp_depth_data_;
      clm_srv.request.depth_info_data = tmp_depth_info_data_;
    }

    pub_myState_.publish(rosMessage("Forwarding data to RgbdRecording node..."));
    if (srvc_rgbd_recording_.call(clm_srv))
    {
      pub_myState_.publish(rosMessage("RgbdRecording returned true. Pictures saved."));
    }
    else
    {
      pub_myState_.publish(rosMessage("ERROR: Recording service returned false. Continuing"));
    }
  }
}

/*-------------------------------------------------------------------------------------*/
/// service callbacks
/*-------------------------------------------------------------------------------------*/
/**
* Processes recently taken and stored pictures from getBoundingBoxCallback 
* and distributes them among the classifiers for classification. 
**/
bool RecognitionSpace::recognizeItemsCallback(tnp_recognition_space::recognize_items::Request &req,
                                              tnp_recognition_space::recognize_items::Response &res)
{
  pub_myState_.publish(rosMessage("RecognizeItemsCallback was called."));

  int num_images = tmp_cam_id_.size();
  if (num_images == 0)
  {
    pub_myState_.publish(rosMessage("OFFLINE - No pictures in the pipeline. Request getBoundingBoxCallback first."));
    //TODO: Recovery plan anyone?
    return false;
  }
  else
  {

// background removal
    tnp_feat_vision::remove_background rb_srv;

    rb_srv.request.cam_id = tmp_cam_id_;
    rb_srv.request.rgb_data = tmp_rgb_data_;
    rb_srv.request.depth_data = tmp_depth_data_;
    rb_srv.request.rgb_info_data = tmp_rgb_info_data_;
    rb_srv.request.depth_info_data = tmp_depth_info_data_;

    pub_myState_.publish(rosMessage("Requesting background removal service..."));
    long t_startBR = timeNow();

    if (srvc_background_removal_.call(rb_srv))
    {
      pub_myState_.publish(rosMessage("Background removal returned true. Distribute the background-removed data..."));
    }
    else
    {
      pub_myState_.publish(rosMessage("Background removal returned FALSE. Use static masking and pray..."));
// overwrite response with fallback solution
      rb_srv.response.cam_id = tmp_cam_id_;
      rb_srv.response.rgb_data = tmp_rgb_data_;
    }
    ROS_DEBUG("Background removal calc time: %.2lf sec", (((double )timeNow() - t_startBR) / 1000));

    // ### distribute pictures
    int CRITICAL_NUM_CAMS_CLOUDM = 1;
    int CRITICAL_NUM_CAMS_SVM_AKAZE = 1;
    int CRITICAL_NUM_COLOR_HIST = 1;

    if (num_images >= CRITICAL_NUM_CAMS_SVM_AKAZE)
    {
      tnp_svm::IdentifyItem svm_srv;
      svm_srv.request.cam_id = rb_srv.response.cam_id;
      svm_srv.request.rgb_data = rb_srv.response.rgb_data;
      svm_srv.request.depth_data = tmp_depth_data_;
      svm_srv.request.rgb_info_data = tmp_rgb_info_data_;
      svm_srv.request.depth_info_data = tmp_depth_info_data_;

      pub_myState_.publish(rosMessage("Requesting SVM feature matching..."));
      long t_startSVM = timeNow();

      if (srvc_svm_matching_.call(svm_srv))
      {
        pub_myState_.publish(rosMessage("Got SVM feature service response. Append results to reply."));

        tnp_recognition_space::ClassifierResult cr;
        cr.classifierName = rosMessage("AKAZE SVM");
        cr.item_ids = svm_srv.response.items_ids;
        cr.confidences = svm_srv.response.confidences;
        res.classifier_results.push_back(cr); // push back AKAZE SVM results
      }
      else
      {
        pub_myState_.publish(rosMessage("ERROR: SVM feature service call returned false"));
      }
      ROS_DEBUG("AKAZE SVM calc  time: %.2lf sec", (((double )timeNow() - t_startSVM) / 1000));
    }
    else
    {
      pub_myState_.publish(
          rosMessage("ERROR: Sorry, too few images retrieved from cameras for SVM feature service call. Dropping."));
    }

    if (num_images >= CRITICAL_NUM_COLOR_HIST)
    {
      tnp_feat_vision::identify_item hist_srv;
      hist_srv.request.cam_id = rb_srv.response.cam_id;
      hist_srv.request.rgb_data = rb_srv.response.rgb_data;
      hist_srv.request.depth_data = tmp_depth_data_;
      hist_srv.request.rgb_info_data = tmp_rgb_info_data_;
      hist_srv.request.depth_info_data = tmp_depth_info_data_;

      pub_myState_.publish(rosMessage("Requesting color histogram..."));
      long t_startCH = timeNow();

      if (srvc_color_histogram_.call(hist_srv))
      {
        pub_myState_.publish(rosMessage("Got color histogram service response. Append results to reply."));

        tnp_recognition_space::ClassifierResult cr;
        cr.classifierName = rosMessage("Color histogram");
        cr.item_ids = hist_srv.response.items_ids;
        cr.confidences = hist_srv.response.confidences;
        res.classifier_results.push_back(cr); // push back color histo results
      }
      else
      {
        pub_myState_.publish(rosMessage("ERROR: Color histogram service call returned false"));
      }
      ROS_DEBUG("Color histogram calc  time: %.2lf sec", (((double )timeNow() - t_startCH) / 1000));
    }
    else
    {
      pub_myState_.publish(
          rosMessage(
              "ERROR: Sorry, too few images retrieved from cameras for color histogram service call. Dropping."));
    }

    // clear temporary stored images
    tmp_cam_id_.clear();
    tmp_rgb_data_.clear();
    tmp_depth_data_.clear();
    tmp_rgb_info_data_.clear();
    tmp_depth_info_data_.clear();

    return true;
  }
}
/**
* Takes scene pictures from recognition space and saves them to disk
**/
bool RecognitionSpace::recordSceneCallback(tnp_recognition_space::record_scene::Request &req,
                                           tnp_recognition_space::record_scene::Response &res)
{
  ROS_DEBUG("RecordSceneCallback...");
  pub_myState_.publish(rosMessage("RecordSceneCallback - taking snapshot of scenery..."));
  long t_start_rc = timeNow();

// save & set LED intensities, take pictures and load previous LED setting
  saveCurrentIlluminationSettings();
  retrieveRecognitionSpaceImages(true, true, intensityConfiguration_);
  setIllumination(previous_illumination_settings_);

  tnp_feat_vision::cloud_matching clm_srv;
  if (!req.item_name.data.empty())
    clm_srv.request.target_id = req.item_name;
  clm_srv.request.cam_id = tmp_cam_id_;
  clm_srv.request.rgb_data = tmp_rgb_data_;
  clm_srv.request.depth_data = tmp_depth_data_;
  clm_srv.request.rgb_info_data = tmp_rgb_info_data_;
  clm_srv.request.depth_info_data = tmp_depth_info_data_;

// clear temporary stored images
  tmp_cam_id_.clear();
  tmp_rgb_data_.clear();
  tmp_depth_data_.clear();
  tmp_rgb_info_data_.clear();
  tmp_depth_info_data_.clear();

  ROS_DEBUG("Recording recognition space time: %.2lf sec", (((double )timeNow() - t_start_rc) / 1000));
  pub_myState_.publish(rosMessage("Forwarding data to RgbdRecording node..."));

  if (srvc_rgbd_recording_.call(clm_srv))
  {
    res.successful.data = true;
    pub_myState_.publish(rosMessage("RgbdRecording node returned true, pictures saved to disk."));
    return true;
  }
  else
  {
    pub_myState_.publish(rosMessage("ERROR: Service returned false"));
    return false;
  }
}

/**
 * Taking some rgb images from empty recognition space for statistical background removal
 * @param req
 * @param res
 * @return
 */
bool RecognitionSpace::recordEmptyRSCallback(tnp_recognition_space::record_empty_rs::Request &req,
                                             tnp_recognition_space::record_empty_rs::Response &res)
{
  tnp_feat_vision::set_rs_background srsb;

  int num_imageSets = 0;
  int brightnessSteps = 3;
  saveCurrentIlluminationSettings();

  long t_start = timeNow();
  while (num_imageSets < req.num_images.data)
  {
    // prepare gradually increasing illumination
    vector<vector<int> > illu_for_backgroundSub(intensityConfiguration_.size(),
                                                vector<int>(intensityConfiguration_[0].size(), 0)); // size: [cameras][lamps]
    for (int i = 0; i < intensityConfiguration_.size(); i++)
    {
      for (int j = 0; j < intensityConfiguration_[i].size(); j++)
      {
        if (intensityConfiguration_[i][j] > 0)
        {
          int tmp_brghtnss = intensityConfiguration_[i][j]
              - (num_imageSets - req.num_images.data / 2) * brightnessSteps;
          illu_for_backgroundSub[i][j] = tmp_brghtnss > 0 ? tmp_brghtnss : 0; // no negative brightness
          // illumination around the target value of intensityConfiguration_
        }
        else
          illu_for_backgroundSub[i][j] = 0; // shut off
      }
    }

    // save & set LED intensities, take pictures and load previous LED setting
    retrieveRecognitionSpaceImages(true, false, illu_for_backgroundSub);
    // storing pictures as well
    storingCurrentImages(true);

    // background subtraction appending data
    srsb.request.cam_id.insert(srsb.request.cam_id.end(), tmp_cam_id_.begin(), tmp_cam_id_.end());
    srsb.request.rgb_data.insert(srsb.request.rgb_data.end(), tmp_rgb_data_.begin(), tmp_rgb_data_.end());
    srsb.request.rgb_info_data.insert(srsb.request.rgb_info_data.end(), tmp_rgb_info_data_.begin(),
                                      tmp_rgb_info_data_.end());

    std_msgs::Int16 numCam;
    numCam.data = tmp_cam_id_.size();
    srsb.request.num_camera_per_scene.push_back(numCam);

    // clear temporary stored images
    tmp_cam_id_.clear();
    tmp_rgb_data_.clear();
    tmp_depth_data_.clear();
    tmp_rgb_info_data_.clear();
    tmp_depth_info_data_.clear();

    num_imageSets++;
  }

  if (!srvc_set_rs_background_.call(srsb))
  {
    pub_myState_.publish(rosMessage("ERROR: Recording empty recognition space retrieval failed."));
  }

  setIllumination(previous_illumination_settings_);
  ROS_DEBUG("Recording images for empty background time: %.2lf sec", (((double )timeNow() - t_start) / 1000));

  return true;
}

/**
 * Calls bbox service only, pictures must be taken before.
 * @param res response object
 * @return successful service call
 */
bool RecognitionSpace::call_BBX_service(tnp_recognition_space::get_bounding_box::Response& res)
{
  tnp_feat_vision::get_bounding_box gbbv;
  gbbv.request.cam_id = tmp_cam_id_;
  gbbv.request.rgb_data = tmp_rgb_data_;
  gbbv.request.depth_data = tmp_depth_data_;
  gbbv.request.rgb_info_data = tmp_rgb_info_data_;
  gbbv.request.depth_info_data = tmp_depth_info_data_;
  if (srvc_get_bounding_box_.call(gbbv))
  {
    res.bb_pose = gbbv.response.bb_pose;
    res.bb_width = gbbv.response.bb_width;
    res.bb_length = gbbv.response.bb_length;
    res.bb_height = gbbv.response.bb_height;

    pub_myState_.publish(rosMessage("BoundingBox retrieval successful."));
    return true;
  }
  else
  {
    pub_myState_.publish(rosMessage("ERROR: BoundingBox retrieval failed."));
    return false;
  }
}
/**
* Triggers the recognition space and calculates the bounding box of an object in the recognition space
**/
bool RecognitionSpace::getBoundingBoxCallback(tnp_recognition_space::get_bounding_box::Request &req,
                                              tnp_recognition_space::get_bounding_box::Response &res)
{
  ROS_DEBUG("getBoundingBoxCallback");
  long t_start = timeNow();

// clear temporary stored images
  tmp_cam_id_.clear();
  tmp_rgb_data_.clear();
  tmp_depth_data_.clear();
  tmp_rgb_info_data_.clear();
  tmp_depth_info_data_.clear();

  num_cameras_online_ = camerasOnline();
  pub_myState_.publish(rosMessage("Currently are " + to_string(num_cameras_online_) + " cameras online"));
  pub_myState_.publish(rosMessage("NUM_CAMERA:" + to_string(num_cameras_online_)));

  if (num_cameras_online_ == 0)
  {
    pub_myState_.publish(rosMessage("OFFLINE - Try again in 10 seconds"));
    //TODO: Recovery plan?
    return false;
  }

  // save & set LED intensities, take pictures and load previous LED setting
  // saveCurrentIlluminationSettings(); // NOTE: Competition speedup
  retrieveRecognitionSpaceImages(true, false, intensityConfiguration_);
  // setIllumination(previous_illumination_settings_); // NOTE: Competition speedup

  // storingCurrentImages(true); // NOTE: Competition speedup

  int num_images = tmp_cam_id_.size();
  pub_myState_.publish(
      rosMessage("Cam online / Images received: " + to_string(num_cameras_online_) + "/" + to_string(num_images)));

  if (num_cameras_online_ > num_images)
    ROS_INFO("Info: We lost data from cameras on the way... (%i/%i)", num_cameras_online_, num_images);

  bool success = call_BBX_service(res);
  ROS_DEBUG("Get bounding box time: %.2lf sec", ((double )timeNow() - t_start) / 1000);
  return success;
}

/**
 * Interactive program in the shell for recording new items
 * @param req
 * @param res
 * @return
 */
bool RecognitionSpace::recordingItemsCallback(tnp_recognition_space::recording_items::Request &req,
                                              tnp_recognition_space::recording_items::Response &res)
{
  ROS_ERROR("Manual init");
  init();

  if (unknown_items_list_.empty())
  {
    string warning = "Recognition space was not previously initialized.";
    pub_myState_.publish(rosMessage(warning));
    ROS_ERROR(warning.c_str());
    return false;
  }

  setIllumination(intensityConfiguration_[0]);
  tnp_recognition_space::record_item ri;
  ROS_WARN("Hello and we1come - there are %lu unknown items to scan", unknown_items_list_.size());
  while (unknown_items_list_.size() > 0)
  {
    string current_item = unknown_items_list_.front();
    string inputString;

    cout << "Do you want to record " << current_item.c_str() << "? (d - default/s - special/n - no/manual)";

    std::getline(std::cin, inputString);
    std_msgs::String msg;
    if (inputString.compare("manual") == 0)
    {
      std::cout << "Manual naming:";
      std::getline(std::cin, inputString);

      ri.request.item_name.data = inputString;
      ri.request.suction_force = req.suction_force;
      ri.request.num_rotations = req.num_rotations;
      ri.request.num_repetitions = req.num_repetitions;
      ri.request.rotation_angle = req.rotation_angle;

      recordItemCallback(ri.request, ri.response);
    }
    else if (inputString.compare("n") == 0)
    {
      unknown_items_list_.pop_front();
      ROS_INFO("Skip.");
    }
    else if (inputString.compare("d") == 0)
    {
      ri.request.item_name.data = current_item;
      ri.request.suction_force = req.suction_force;
      ri.request.num_rotations = req.num_rotations;
      ri.request.num_repetitions = req.num_repetitions;
      ri.request.rotation_angle = req.rotation_angle;

      recordItemCallback(ri.request, ri.response);

      unknown_items_list_.pop_front();
    }
    else if (inputString.compare("s") == 0)
    {
// name
      std::cout << "Manual naming:";
      inputString.clear();
      while (inputString.empty())
      {
        std::getline(std::cin, inputString);
        ri.request.item_name.data = inputString;
      }

// suction
      std::cout << "Manual suction force [25-40], default:" << req.suction_force.data;
      inputString.clear();
      while (inputString.empty() || stoi(inputString) < 25 || stoi(inputString) < 40)
      {
        std::getline(std::cin, inputString);
        ri.request.suction_force.data = stoi(inputString);
      }

// num rotation
      std::cout << "Manual num of rotation, default:" << req.num_rotations.data;
      inputString.clear();
      while (inputString.empty() || stoi(inputString) < 0)
      {
        std::getline(std::cin, inputString);
        ri.request.num_rotations.data = stoi(inputString);
      }

//num repetitions
      std::cout << "Manual num of repetitions, default:" << req.num_repetitions.data;
      inputString.clear();
      while (inputString.empty() || stoi(inputString) < 0)
      {
        std::getline(std::cin, inputString);
        ri.request.num_repetitions.data = stoi(inputString);
      }

//num repetitions
      std::cout << "Manual rotation angle, default:" << req.rotation_angle.data;
      inputString.clear();
      while (inputString.empty() || stoi(inputString) < 0)
      {
        std::getline(std::cin, inputString);
        ri.request.rotation_angle.data = stoi(inputString);
      }

      recordItemCallback(ri.request, ri.response);
    }
    ros::spinOnce();
    ROS_WARN("%lu unknown items left.", unknown_items_list_.size());

  }
  return true;
}
/**
 * Recording sequence of an unknown item
 * @param req
 * @param res
 * @return
 */
bool RecognitionSpace::recordItemCallback(tnp_recognition_space::record_item::Request &req,
                                          tnp_recognition_space::record_item::Response &res)
{

  int su_force = req.suction_force.data > 0 ? req.suction_force.data : 30;
  double rotation_angle = req.rotation_angle.data != 0 ? M_PI / 180 * (double)req.rotation_angle.data : M_PI / 180 * 40;
  int n_rotations = req.num_rotations.data > 0 ? req.num_rotations.data : 1;
  int repetitions = req.num_repetitions.data > 0 ? req.num_repetitions.data : 1;
  string item_name = req.item_name.data;
  bool skipWeight = false;
  bool clockwiseTurn = true; // clockwise <-> anti-clockwise
  tnp_end_effector::ItemSuctioned is;

  ROS_DEBUG("recordItemCallback for %s", item_name.c_str());
  ROS_INFO("SETUP:");

  if (item_name.empty())
  {
    pub_myState_.publish(rosMessage("ERROR: Give me item_name!"));
    return false;
  }

// end effector initial rotation
  ROS_INFO("End effector go to rec space ...");
  tnp_kuka_motion::goToRecSpace gtrs;
  double tmp_angle = 0.001;
  gtrs.request.rotation_angle_z = tmp_angle;
  if (!srvc_kuka_motion_rotation_.call(gtrs))
    ROS_WARN("- kuka_motion returned false. Not in rec space position.");

// end effector extracting.
  ROS_INFO("Extract suction pipe ...");
  tnp_end_effector::LinActuatorSuction las;
  las.request.setLinActuatorState = true;
  if (!srvc_suction_actuator_.call(las))
    ROS_WARN("- End effector lin act service returned false.");

// suction off
  this_thread::sleep_for(std::chrono::milliseconds(500));
  suction(false, su_force);
  this_thread::sleep_for(std::chrono::milliseconds(500));

// zero weight sensors
  ROS_INFO("Zeroing weight sensors ...");
  tnp_weight_events::GetReadyForPick grfp;
  grfp.request.target_container.data = "bin_A";
  if (!srvc_w_sensors_start_.call(grfp))
    ROS_WARN("Weight sensors returned false");
  else
    ROS_INFO("Weight: Take the item out of the bin.");

// suction on
  suction(true, su_force);
  ROS_DEBUG("Ready, put item to suction head.");

  string inputString;
  inputString.clear();
  while (inputString.compare("y") != 0 && inputString.compare("r") != 0 && inputString.compare("o") != 0)
  {
    std::cout << "Start? ( y - yes , r - suction off, o - suction on, q - terminate )\n";
    std::getline(std::cin, inputString);
    if (inputString.compare("r") == 0)
    {
      suction(false, su_force);
      inputString.clear();
    }
    else if (inputString.compare("o") == 0)
    {
      suction(true, su_force);
      inputString.clear();
    }
    else if (inputString.compare("q") == 0)
    {
      ROS_INFO("Leave service.");
      suction(false, su_force);
      return false;
    }
  }

  if (skipWeight == false)
  {
    // measure weight
    ROS_DEBUG("Measuring weight ...");
    tnp_optoforce::GetWeightChangeSinceReady gwcsr;
    if (!srvc_w_sensors_end_.call(gwcsr))
      ROS_WARN("Measuring service failed.");
    else
    {
      ofstream bbx_file;
      bbx_file.open(own_unknown_weights_root_ + item_name + "_" + to_string(timeNow()) + ".csv");
      bbx_file << "item name, weight\n";
      bbx_file << req.item_name.data.c_str() << "," << gwcsr.response.weight << "\n";
      bbx_file.close();
      ROS_DEBUG("Saved weight of %s having %f g", item_name.c_str(), gwcsr.response.weight);
    }
  }
  else
  {
    ROS_WARN("Skip weight ...");
  }

// #####  END OF SETUP ######################
  long t_startRI = timeNow();

  ROS_INFO("Start recording %s...", item_name.c_str());

  int reps = 0;
  while (reps < repetitions)
  {
    ROS_DEBUG("Repetitions %i of %i", reps + 1, repetitions);

    int rot = 0;
    while (rot < n_rotations)
    {
      ROS_DEBUG("Rotation %i of %i", rot + 1, n_rotations);

      ROS_DEBUG("Take pictures...");
// take pictures
      retrieveRecognitionSpaceImages(true, false, intensityConfiguration_);
      ROS_ERROR("####################################################");
      this_thread::sleep_for(std::chrono::milliseconds(300));
      ROS_DEBUG("Storing pictures...");
// store pictures
      storingCurrentImages(true, item_name);

// calc bbx
      ROS_DEBUG("Get bounding box...");
      tnp_recognition_space::get_bounding_box gbbx;
      if (call_BBX_service(gbbx.response))
      {
        vector<double> dims;
        dims.push_back(gbbx.response.bb_width.data);
        dims.push_back(gbbx.response.bb_length.data);
        dims.push_back(gbbx.response.bb_height.data);

        ofstream bbx_file;
        bbx_file.open(own_unknown_bbx_root_ + item_name + "_" + to_string(timeNow()) + ".csv");
        bbx_file << "item name, x, y, z\n";
        bbx_file << req.item_name.data.c_str() << "," << dims[0] << "," << dims[1] << "," << dims[2] << "\n";
        bbx_file.close();
        ROS_DEBUG("Bounding box of %s: x: %f m, y: %f m, z: %f m", item_name.c_str(), dims[0], dims[1], dims[2]);
      }
      else
        ROS_WARN("Bounding box retrieval failed.");

// clear temporary stored images
      tmp_cam_id_.clear();
      tmp_rgb_data_.clear();
      tmp_depth_data_.clear();
      tmp_rgb_info_data_.clear();
      tmp_depth_info_data_.clear();

      if (rot < n_rotations - 1) // skip action in the last loop
      {
        // rotate
        ROS_DEBUG("Rotate to %f rad", tmp_angle);
        tnp_kuka_motion::goToRecSpace gtrs;
        if (clockwiseTurn)
          tmp_angle += rotation_angle;
        else
          tmp_angle -= rotation_angle;

        gtrs.request.rotation_angle_z = tmp_angle;
        if (!srvc_kuka_motion_rotation_.call(gtrs))
          ROS_WARN("- End effector lin act service returned false.");
      }
      else
        ROS_DEBUG("End of rotations");

      rot++;
    }

    if (reps < req.num_repetitions.data - 1) // continue with new pose
    {
      ROS_INFO("LED on, NEW POSE");

// suction off
      suction(false, su_force);

// toggle turning direction
      clockwiseTurn = (clockwiseTurn == true ? false : true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // suction off time

      suction(true, su_force);

      inputString.clear();
      is.response.suctioned = false;
      while (is.response.suctioned == false && inputString.compare("y") != 0)
      {
        srvc_suction_contact_.call(is);
        this_thread::sleep_for(std::chrono::milliseconds(100)); // wait until contact
      }

      inputString.clear();
      while (inputString.compare("y") != 0 && inputString.compare("r") != 0 && inputString.compare("o") != 0)
      {
        std::cout << "Contact. Start? (y - yes, r - suction off, o - suction on, q - terminate)\n";
        std::getline(std::cin, inputString);
        if (inputString.compare("r") == 0)
        {
          suction(false, su_force);
          inputString.clear();

        }
        else if (inputString.compare("o") == 0)
        {
          suction(true, su_force);
          inputString.clear();

        }
        else if (inputString.compare("q") == 0)
        {
          ROS_INFO("Leave service.");
          suction(false, su_force);

          return false;
        }
      }
    }
    else // end after blinking
    {
      suction(false, su_force);

//rotate to initial position
      tnp_kuka_motion::goToRecSpace gtrs;
      gtrs.request.rotation_angle_z = 0.001;
      srvc_kuka_motion_rotation_.call(gtrs);

      ROS_DEBUG("End round. New item?");
    }
    reps++;

  }

  ROS_DEBUG("Record new item time: %.2lf sec", ((double )timeNow() - t_startRI) / 1000);

}

/**
 * Turning on or off suction pump.
 * @param turn_on
 * @param force between [0, 40]
 * @return
 */
bool RecognitionSpace::suction(bool turn_on, const int& force)
{
  if (force < 0 || force > 40)
  {
    pub_myState_.publish(rosMessage("Suction value out of range."));
    return false;
  }

  tnp_end_effector::Suction srv;
  srv.request.setSuctionState = turn_on;
  srv.request.suction_force.data = force;
  if (srvc_suction_.call(srv))
  {
    pub_myState_.publish(rosMessage(string("Suction " + boolToString(turn_on))));
    return true;
  }
  else
  {
    pub_myState_.publish(rosMessage("Suction service returned false"));
    return false;
  }
}

/**
* creates a ros message
*/
static std_msgs::String rosMessage(std::string txt)
{
  std_msgs::String msg;
  msg.data = txt;

  return msg;
}

/**
 * Returns the current time in milliseconds
 */
long timeNow()
{
  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

/*-------------------------------------------------------------------------------------*/
/// main
/*-------------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_rec_space");
  ros::NodeHandle nh;

//Create an object of class RecognitionSpace that will take care of everything
  RecognitionSpace recSpace(nh);

  ROS_INFO("Recognition Space node started");
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
