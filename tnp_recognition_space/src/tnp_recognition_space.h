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

#ifndef SRC_TNP_RECOGNITION_SPACE_H_
#define SRC_TNP_RECOGNITION_SPACE_H_

#include "tnp_recognition_space/set_items_info.h"
#include "tnp_recognition_space/recognize_items.h"
#include "tnp_recognition_space/record_scene.h"
#include "tnp_recognition_space/ClassifierResult.h"
#include "tnp_recognition_space/get_bounding_box.h"
#include "tnp_recognition_space/record_empty_rs.h"
#include "tnp_recognition_space/record_item.h"
#include "tnp_recognition_space/recording_items.h"
#include "tnp_recognition_space/prepare_competition_files.h"
//
#include "tnp_end_effector/OpenShutterCloseTheRest.h"
#include "tnp_end_effector/ItemSuctioned.h"
//
#include "tnp_feat_vision/remove_background.h"
#include "tnp_feat_vision/cloud_matching.h"
#include "tnp_feat_vision/identify_item.h"
#include "tnp_feat_vision/get_bounding_box.h"
#include "tnp_feat_vision/set_rs_background.h"
//
#include "iiwa_msgs/LED.h"
//
#include "tnp_kuka_motion/goToRecSpace.h"
#include "tnp_end_effector/Suction.h"
#include "tnp_end_effector/LinActuatorSuction.h"
//
#include "tnp_led_control/set_led_intensity.h"
#include "tnp_led_control/get_led_intensity.h"
//
#include "tnp_svm/IdentifyItem.h"
//
#include "tnp_optoforce/GetWeightChangeSinceReady.h"
#include "tnp_optoforce/ZeroSensors.h"

#include "tnp_weight_events/GetReadyForPick.h"
//
#include "helpers2.h"
//
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <algorithm>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <list>
#include <sstream>
#include <sys/stat.h>
#include <sys/dir.h>
#include <thread>
#include <tuple>


class RecognitionSpace
{
public:
  // Constructor
  RecognitionSpace(ros::NodeHandle &nh);

  // Topic callbacks
  void stateCallback(const std_msgs::String &msg);
  void setSuctionSensor(const std_msgs::String &msg);

  // camera left
  void camera_L_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_L_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_L_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_L_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  // camera center
  void camera_C_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_C_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_C_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_C_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  // camera right
  void camera_R_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_R_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_R_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_R_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  // camera base
  void camera_B_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_B_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_B_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_B_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  // camera ee
  void camera_E_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_E_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_E_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_E_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  // camera top
  void camera_T_RGBcallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_T_DepthCallback(const sensor_msgs::ImageConstPtr &msg);
  void camera_T_CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void camera_T_DepthInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  // Service callback
  bool setItemsCallback(tnp_recognition_space::set_items_info::Request &req,
                        tnp_recognition_space::set_items_info::Response &res);
  bool recognizeItemsCallback(tnp_recognition_space::recognize_items::Request &req,
                              tnp_recognition_space::recognize_items::Response &res);
  bool recordSceneCallback(tnp_recognition_space::record_scene::Request &req,
                           tnp_recognition_space::record_scene::Response &res);
  bool recordEmptyRSCallback(tnp_recognition_space::record_empty_rs::Request &req,
                             tnp_recognition_space::record_empty_rs::Response &res);
  bool getBoundingBoxCallback(tnp_recognition_space::get_bounding_box::Request &req,
                              tnp_recognition_space::get_bounding_box::Response &res);
  bool recordItemCallback(tnp_recognition_space::record_item::Request &req,
                          tnp_recognition_space::record_item::Response &res);
  bool recordingItemsCallback(tnp_recognition_space::recording_items::Request &req,
                              tnp_recognition_space::recording_items::Response &res);
  bool createCompetitionFiles(tnp_recognition_space::prepare_competition_files::Request &req,
                              tnp_recognition_space::prepare_competition_files::Response &res);
private:
  // ********** TRAINING FUNCTIONS TOGGLE HERE
  bool COMPETITION = false;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Publisher pub_myState_;
  ros::Publisher pub_recogn_data_;
  ros::Subscriber sub_myState_;

  image_transport::Subscriber sub_cam_L_RGB_;
  image_transport::Subscriber sub_cam_L_Depth_;
  ros::Subscriber sub_cam_L_CamInfo_;
  ros::Subscriber sub_cam_L_DepthInfo_;

  image_transport::Subscriber sub_cam_C_RGB_;
  image_transport::Subscriber sub_cam_C_Depth_;
  ros::Subscriber sub_cam_C_CamInfo_;
  ros::Subscriber sub_cam_C_DepthInfo_;

  image_transport::Subscriber sub_cam_R_RGB_;
  image_transport::Subscriber sub_cam_R_Depth_;
  ros::Subscriber sub_cam_R_CamInfo_;
  ros::Subscriber sub_cam_R_DepthInfo_;

  image_transport::Subscriber sub_cam_B_RGB_;
  image_transport::Subscriber sub_cam_B_Depth_;
  ros::Subscriber sub_cam_B_CamInfo_;
  ros::Subscriber sub_cam_B_DepthInfo_;

  image_transport::Subscriber sub_cam_E_RGB_;
  image_transport::Subscriber sub_cam_E_Depth_;
  ros::Subscriber sub_cam_E_CamInfo_;
  ros::Subscriber sub_cam_E_DepthInfo_;

  image_transport::Subscriber sub_cam_T_RGB_;
  image_transport::Subscriber sub_cam_T_Depth_;
  ros::Subscriber sub_cam_T_CamInfo_;
  ros::Subscriber sub_cam_T_DepthInfo_;

  ros::ServiceClient srvc_shutter_;

  ros::ServiceClient srvc_background_removal_;
  ros::ServiceClient srvc_cloud_matching_;
  ros::ServiceClient srvc_svm_matching_;
  ros::ServiceClient srvc_color_histogram_;

  ros::ServiceClient srvc_rgbd_recording_;
  ros::ServiceClient srvc_get_bounding_box_;
  ros::ServiceClient srvc_set_rs_background_;
  ros::ServiceClient srvc_led_controller_set_;
  ros::ServiceClient srvc_led_controller_get_;
  ros::ServiceClient srvc_kuka_motion_rotation_;
  ros::ServiceClient srvc_suction_;
  ros::ServiceClient srvc_kuka_led_;
  ros::ServiceClient srvc_suction_contact_;
  ros::ServiceClient srvc_suction_actuator_;
  ros::ServiceClient srvc_w_sensors_start_;
  ros::ServiceClient srvc_w_sensors_end_;

  // services
  ros::ServiceServer srv_set_items_info_;
  ros::ServiceServer srv_recognize_items_;
  ros::ServiceServer srv_record_rec_space_;
  ros::ServiceServer srv_get_bounding_box_;
  ros::ServiceServer srv_record_empty_rs_;
  ros::ServiceServer srv_record_item_;
  ros::ServiceServer srv_recording_items_;
  ros::ServiceServer srv_prepare_competition_files_;

  // topics
  const std::string IMG_RGB = "color/image_raw";
  const std::string IMG_D = "depth/image_raw";
  const std::string IMG_INFO = "color/camera_info";
  const std::string DEPTH_INFO = "depth/camera_info";

  // pointer of the latest data
  sensor_msgs::ImageConstPtr camL_rgb_raw_;
  sensor_msgs::ImageConstPtr camC_rgb_raw_;
  sensor_msgs::ImageConstPtr camR_rgb_raw_;
  sensor_msgs::ImageConstPtr camB_rgb_raw_;
  sensor_msgs::ImageConstPtr camE_rgb_raw_;
  sensor_msgs::ImageConstPtr camT_rgb_raw_;

  sensor_msgs::ImageConstPtr camL_d_raw_;
  sensor_msgs::ImageConstPtr camC_d_raw_;
  sensor_msgs::ImageConstPtr camR_d_raw_;
  sensor_msgs::ImageConstPtr camB_d_raw_;
  sensor_msgs::ImageConstPtr camE_d_raw_;
  sensor_msgs::ImageConstPtr camT_d_raw_;

  sensor_msgs::CameraInfoConstPtr camL_rgb_info_;
  sensor_msgs::CameraInfoConstPtr camC_rgb_info_;
  sensor_msgs::CameraInfoConstPtr camR_rgb_info_;
  sensor_msgs::CameraInfoConstPtr camB_rgb_info_;
  sensor_msgs::CameraInfoConstPtr camE_rgb_info_;
  sensor_msgs::CameraInfoConstPtr camT_rgb_info_;

  sensor_msgs::CameraInfoConstPtr camL_depth_info_;
  sensor_msgs::CameraInfoConstPtr camC_depth_info_;
  sensor_msgs::CameraInfoConstPtr camR_depth_info_;
  sensor_msgs::CameraInfoConstPtr camB_depth_info_;
  sensor_msgs::CameraInfoConstPtr camE_depth_info_;
  sensor_msgs::CameraInfoConstPtr camT_depth_info_;

  // received data counter per stream
  int camL_bool_rgb_raw_ = 0;
  int camL_bool_rgb_info_ = 0;
  int camL_bool_d_raw_ = 0;
  int camL_bool_d_info_ = 0;
  int camC_bool_rgb_raw_ = 0;
  int camC_bool_rgb_info_ = 0;
  int camC_bool_d_raw_ = 0;
  int camC_bool_d_info_ = 0;
  int camR_bool_rgb_raw_ = 0;
  int camR_bool_rgb_info_ = 0;
  int camR_bool_d_raw_ = 0;
  int camR_bool_d_info_ = 0;
  int camB_bool_rgb_raw_ = 0;
  int camB_bool_rgb_info_ = 0;
  int camB_bool_d_raw_ = 0;
  int camB_bool_d_info_ = 0;
  int camE_bool_rgb_raw_ = 0;
  int camE_bool_rgb_info_ = 0;
  int camE_bool_d_raw_ = 0;
  int camE_bool_d_info_ = 0;
  int camT_bool_rgb_raw_ = 0;
  int camT_bool_rgb_info_ = 0;
  int camT_bool_d_raw_ = 0;
  int camT_bool_d_info_ = 0;

  int num_cameras_online_ = 0;

  // temporary saved data when recognition space data is collected
  std::vector<std_msgs::String> tmp_cam_id_;
  std::vector<sensor_msgs::Image> tmp_rgb_data_;
  std::vector<sensor_msgs::Image> tmp_depth_data_;
  std::vector<sensor_msgs::CameraInfo> tmp_rgb_info_data_;
  std::vector<sensor_msgs::CameraInfo> tmp_depth_info_data_;
  std::vector<std_msgs::Int16> previous_illumination_settings_; // before class was called
  std::vector<std_msgs::Int16> tmp_illumination_settings_; // within class
  const std::vector<std::vector<int>> intensityConfiguration_ = {
  {5, 25, 25, 25, 25, 25, 25}, // E
     {5, 25, 25, 25, 25, 25, 25}, // L
     {5, 25, 25, 25, 25, 25, 25}, // C
     {5, 25, 25, 25, 25, 25, 25}, // R
     {5, 25, 25, 25, 25, 25, 25}}; // B

  std::string amazon_resource_path_;

  std::string amazon_unknown_items_root_;
  std::string amazon_known_items_root_;

  std::string own_unknown_items_root_;
  std::string own_known_items_root_;

  std::string own_unknown_bbx_root_;
  std::string own_known_bbx_root_;

  std::string own_unknown_weights_root_;
  std::string own_known_weights_root_;

  std::string svm_augmented_items_own_root_;
  std::string svm_target_folder_;

  std::string svm_competition_items_own_root_;
  std::string histo_competition_items_own_root_;
  std::string svm2_competition_weights_root_;
  std::string svm2_competition_bbx_root_;

  std::vector<std::pair<std::string, bool>> item_order_list_;
  std::list<std::string> unknown_items_list_;

  bool init();
  bool IRProjectorOn(std::string camera);
  bool IRProjectorOff(std::string camera);
  bool setIllumination(std::vector<int> intensities);
  bool setIllumination(std::vector<std_msgs::Int16> intensities);
  bool saveCurrentIlluminationSettings();

  bool hasPublisher(std::string topic);
  int camerasOnline();
  bool blockingWaitFor(std::string cameraID, double seconds, bool usingDepth);
  bool checkCompleteness(std::string cameraID, bool withDepth);
  void resetImageMemory(std::string cameraID);
  void retrieveRecognitionSpaceImages(bool withDepth, bool withEEcam,
                                      std::vector<std::vector<int> > illuminationConfigurations);
  void storingCurrentImages(bool withDepth, std::string item_name = "");
  bool call_BBX_service(tnp_recognition_space::get_bounding_box::Response& res);
  bool suction(bool turn_on, const int& force/*from 20 (min) to 40 (max)*/);

};
//End of class RecognitionSpace

static std_msgs::String rosMessage(std::string txt);
std::string boolToString(bool in);
long timeNow();

#endif /* SRC_TNP_RECOGNITION_SPACE_H_ */
