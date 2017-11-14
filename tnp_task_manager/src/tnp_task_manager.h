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

#ifndef TNP_TASK_MANAGER_H
#define TNP_TASK_MANAGER_H

// End effector
#include "tnp_end_effector/Blower.h"
#include "tnp_end_effector/Gripper.h"
#include "tnp_end_effector/LinActuatorGripper.h"
#include "tnp_end_effector/LinActuatorSuction.h"
#include "tnp_end_effector/Suction.h"
#include "tnp_end_effector/Drawer.h"
#include "tnp_end_effector/OpenShutterCloseTheRest.h"
// kuka motion
#include "tnp_kuka_motion/goToAmnesty.h"
#include "tnp_kuka_motion/goToBin.h"
#include "tnp_kuka_motion/goToBox.h"
#include "tnp_kuka_motion/goToHome.h"
#include "tnp_kuka_motion/goToLookIntoBin.h"
#include "tnp_kuka_motion/goToLookIntoTote.h"
#include "tnp_kuka_motion/goToRecSpace.h"
#include "tnp_kuka_motion/goToTote.h"
#include "tnp_kuka_motion/graspItem.h"
#include "tnp_kuka_motion/moveToJointAnglesPTP.h"
#include "tnp_kuka_motion/moveToJointAnglesPTPAction.h"
#include "tnp_kuka_motion/putItemIntoBin.h"
#include "tnp_kuka_motion/putItemIntoBox.h"
#include "tnp_kuka_motion/putItemIntoTote.h"
#include "tnp_kuka_motion/putItemIntoAmnesty.h"
#include "tnp_kuka_motion/putItemIntoContainer.h"
#include "tnp_kuka_motion/suckItem.h"
#include "tnp_kuka_motion/sweepToteHorizontal.h"
#include "tnp_kuka_motion/goToLookIntoContainer.h"
#include "tnp_kuka_motion/goToContainer.h"
#include "tnp_moveit_planner/isEEPoseReachable.h"
// task manager
#include "tnp_task_manager/pickTask.h"
#include "tnp_task_manager/stowTask.h"
#include "tnp_task_manager/testMotions.h"
#include "tnp_task_manager/setupBeforeRound.h"
#include "tnp_task_manager/initialSetup.h"
#include "tnp_task_manager/transform2poses.h"
// Weight events
#include "tnp_weight_events/RecognizeItems.h"
#include "tnp_weight_events/GetReadyForPick.h"
#include "tnp_weight_events/SetItemsInfo.h"
#include "tnp_weight_events/SetItemsLocation.h"
// Recognition Space
#include "tnp_recognition_space/get_bounding_box.h"
#include "tnp_recognition_space/recognize_items.h"
#include "tnp_recognition_space/record_empty_rs.h"
#include "tnp_recognition_space/record_scene.h"
#include "tnp_recognition_space/ClassifierResult.h"
#include "tnp_recognition_space/get_bounding_boxResponse.h"
#include "tnp_deep_vision/recognize_items.h"
#include "tnp_recognition_space/set_items_info.h"
#include "tnp_recognition_space/set_items_infoRequest.h"
// Grasp Planner
#include "tnp_grasp_planner/getGripCandidates_container.h"
#include "tnp_grasp_planner/getSuctionCandidates_bb.h"
#include "tnp_grasp_planner/getGripCandidates_bb.h"
#include "tnp_grasp_planner/updateOccupancyMap.h"
#include "tnp_grasp_planner/getPoseToPlaceItem.h"
#include "tnp_grasp_planner/getSuctionCandidates_container_fromOctomap.h"
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
// LED control
#include "tnp_led_control/set_led_intensity.h"
#include "tnp_led_control/get_led_intensity.h"
// Visualization (tnp_monitor)
#include <tnp_msgs/ShowPose.h>
// SVM
#include <tnp_svm/IdentifyItem.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tnp_msgs/TaskList.h"
#include "tnp_msgs/StateInfo.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "tf/transform_listener.h"
#include "Database.h" //JSON Library

#include <chrono>
#include <termios.h>	// non-blocking getchar
#include <thread>	// To wait for time_to_destination
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <mutex>
#include <fstream>
#include <cmath>

struct TargetItemStruct{
  std::string DL_tag;
  int order;
  geometry_msgs::PoseStamped suck_pose;
  TargetItemStruct() : DL_tag(""),
                       order(-1)
  {
    suck_pose.pose.orientation.w = 1.0;
  }
};

struct RetrievalCandidateStruct{
  // suckable surface
  geometry_msgs::PoseStamped suck_pose;
  std::vector<std::string> suck_pose_DL_tags;
  std::vector<float> suck_pose_DL_confidences;
  std::vector<float> suck_pose_DL_bb_center_z;
  // deep learning
  std::string dl_item_id;
  float dl_confidence;
  float dl_item_center_x;
  float dl_item_center_y;
  float dl_item_width;
  float dl_item_height;
  geometry_msgs::PoseStamped dl_item_center_poses;
  geometry_msgs::PoseStamped dl_grasp_center_poses;
  geometry_msgs::PoseStamped dl_grasp_line_pt1_poses;
  geometry_msgs::PoseStamped dl_grasp_line_pt2_poses;
  // weight
  std::vector<std::string> weight_item_ids;
  std::vector<float> weight_confidences;
  RetrievalCandidateStruct()
  {
    suck_pose.pose.orientation.w = 1.0;
    dl_item_id = "";
    dl_confidence = 0.0;
    dl_item_center_x = 0.0;
    dl_item_center_y = 0.0;
    dl_item_width = 0.0;
    dl_item_height = 0.0;
    dl_item_center_poses.pose.orientation.w = 1.0;
    dl_grasp_center_poses.pose.orientation.w = 1.0;
    dl_grasp_line_pt1_poses.pose.orientation.w = 1.0;
    dl_grasp_line_pt2_poses.pose.orientation.w = 1.0;
  }
};

/**
 * TaskMangerNode, controls the decision process
 */
class TaskManagerNode
{
public:
  TaskManagerNode();

  bool setupROSConfiguration();

  //Helpers
  void publishItem(const std::string& target_item, const geometry_msgs::Pose& target_pose);
  void publishContainerList(const std::string& container_id);
  void publishRunningStatus(const bool& running_status);
  void publishStateInfo(const std::string current_state, std::vector<std::string> next_states);
  // KUKa motion related
  bool GoToHome();
  bool GoToBin(const std::string& bin_id);
  bool GoToRecSpace();
  bool GoToBox(const int& box_id);
  bool GoToTote();
  bool GoToContainer(const std::string& container_id, bool keep_A7_fixed = false);
  bool EEPoseIsReachable(geometry_msgs::Pose EE_pose, bool use_gripper_EE);

  /**
   * Places the end effector camera to pre-established points of view to increase the DL recognition success
   * @param point_of_view int point of view for the camera (defined in the tnp_kuka_motion)
   */
  bool GoToLookIntoTote(const int& point_of_view);

  bool GoToAmnesty();
  bool moveToJointAnglesPTP(const double& j1, const double& j2, const double& j3, const double& j4, const double& j5,
                            const double& j6, const double& j7);

  /**
   * Call KUKA service to retrieve item through suction method
   * @param itemName contains the id of the item to be retrieved
   * @param itemPose contains the pose coordinates of the location of the item center
   */
  bool SuckItem(const std::string& itemName, geometry_msgs::PoseStamped itemPose, const int& force, bool fuzzyMode = false, std::string container_name = "");

  bool SweepToteHorizontal(double height, bool sweep_to_left = false);

  /**
   * Call KUKA service to retrieve item through grasping method
   * @param itemName contains the id of the item to be retrieved
   * @param itemPose contains the pose coordinates of the location of the item center
   */
  bool GraspItem(const std::string &itemName, geometry_msgs::PoseStamped itemPose,
                 geometry_msgs::PoseStamped graspLinePt1, geometry_msgs::PoseStamped graspLinePt2,
                 const int& force, bool fuzzyMode = false);

  /**
   * Call KUKA service to deliver the item currently in the end effector to a specified box,
   * internally it determines in which box to deliver the item throught the order_id received
   * @param order_id contains the id of the order containing the item to deliver
   */
  bool PutItemIntoBox(const int& order_id);

  /**
   * Call KUKA service to deliver an item to the specified bin
   * @param bin_id contains the id of the target bin to deliver the item
   */
  bool PutItemIntoBin(const std::string& bin_id);

  bool PutItemIntoTote();

  bool PutItemIntoAmnesty();

  bool PutItemIntoContainer( std::string container_name, bool use_gripper_EE,
                                            geometry_msgs::PoseStamped target_pose_in_world );

  bool GoToLookIntoContainer( const std::string& container_name, const int& height );

  // Deep learning
  /**
   * Call DL Service to recognize items under the end effector camera vision range
   */
  geometry_msgs::PoseStamped Get3DPoseFrom2DCoord(float x, float y);
  bool DLRecognizeItems(bool use_stow = false);

  // Weight events
  /**
   * Notify Weight service that a picking will be soon made to start detecting changes
   * @param target_id contains the id of the item to be picked up
   * @param target_container contains the id of the container from which the item will be retrieved
   */
  bool WeightGetReadyToPick(const std::string& target_id, const std::string& target_container);

  /**
   * Call the Weight service to retrieve the list of posible items recognized throught the weight of
   * the picked item
   * @param target_id contains the id of the item to be picked up
   * @param target_container contains the id of the container from which the item will be retrieved
   */
  bool WeightRecognizeItems(const std::string& target_id, const std::string& target_container);

  bool WeightSetItemsInfo();
  bool WeightSetItemsLocation();

  // Recognition Space
  /**
   * Call the recognition space service to determine a list of items detected through the item positioned
   * in the recognition space
   * @param target_item_id contains the id of the item to be recognized
   */
  bool RecSpaceRecognizeItems(const std::string& target_item_id, std::vector<std::string> possible_item_ids);
  bool RecSpaceRecordEmptyRS(const int& num_images);
//  bool RecSpaceRecordScene(const std::vector<std_msgs::Int16>& led_intensities);
  bool RecSpaceGetBoundingBox();
  bool RecSpaceSetItemsInfo();

  bool GoToLookIntoBin(const std::string& bin_id, const int& point_of_view);

  // LED control
  /**
   * Call the tnp_led_control node to set/get the LEDs intensity
   * @param target_led_num contains the id of the item to be recognized
   */
   bool SetLedIntensity(const int& led_num, const int& intensity);
   void RestoreLedInitialState();

  // Task Manager
  /**
   *
   * @param target_id contains the id of the item to be picked up
   * @param target_container contains the id of the container from which the item will be retrieved
   */
  bool LoadFiles(std::string task_name);
  bool LoadBoxesAndOrders();
  bool LoadParameters();
  bool OrderListCompleted();
  bool OrderItemsInBin(const std::string &bin_id,  const std::string& search_criterion);
  bool ItemInOrder(const std::string &item_id, int &order_id);
  bool ItemInTote(const std::string& item_id);
  bool ItemInBin(const std::string& bin_id, const std::string& item_id);
  bool ItemIsKnown(const std::string &item_id);
  bool GetBoxInfo(const int &order_id, int& box_id);
  int  GetBoxIDOfItem(const std::string &item_id);
  bool GetRetrieveMethod(const std::string &item_id, int& retrieve_method, int& retrieve_force);
  bool DetermineBinBySize(const std::string& item_id, std::string& bin_id);
  bool UpdateItemLocationInMemory(const std::string &item_id, const std::string &origin_container,
                                const std::string &destination_container);
  bool UpdateOrderLocationInMemory(const int &order_id, const std::string &item_id,
                                 const std::string &location_type, const std::string &location_id);
  void PublishOrdersUnfulfilledItems();
  void PublishTaskMessages(std::string msg, bool append);
  void DumpDecisionDataToTnpMonitor(std::vector<std::pair<std::string, float>> weight_combined_vectors, std::vector<std::pair<std::string, float>> svm_combined_vectors, std::vector<std::string> vec_cloud_matching_items_ids, std::vector<float> vec_cloud_matching_confs,std::vector<std::pair<std::string, float>> akaze_svm_combined_vectors, std::vector<std::pair<std::string, float>> color_histogram_combined_vectors, bool display_the_weight = false);
  bool TimeLimitReached(const ros::Time& begin_time);
  void DrawerTimeLimit(const ros::Time& begin_time);
  bool RetrieveItem(const int& item_retrieve_method, const std::string& item_id, geometry_msgs::PoseStamped item_pose,
                    geometry_msgs::PoseStamped graspLinePt1, geometry_msgs::PoseStamped graspLinePt2,
                    const int& retrieve_force, bool fuzzyMode = false, std::string container_name = "");
  bool DLInitializeVectors();
  bool GetOrderItemsPerBin();
  std::vector<std::string> GetContainerItems( const std::string container_id, std::string type_of_list = "" );
  bool GetCandidatesFromSVM(const float x, const float y, const float z, const float weight);


  // Tasks versions
  bool decide_item_identity_from_all_votes(std::string& belief_item_id, float& belief_confidence, std::string dl_target_item, float dl_item_conf, bool ignore_DL);
  bool decide_item_identity_from_all_votes_stow(std::vector<std::string>& possible_items_ids, std::vector<float>& possible_items_confidences, 
    std::string dl_target_item, float dl_item_conf );
  bool DeliverHeldItemToBox(std::string belief_item_id, bool use_gripper_EE);
  bool SelectCandidateFromDL(int& dl_item_idx, std::string bin_id, int looking_for_known_item, std::vector<geometry_msgs::PoseStamped> vec_poses_blacklist, bool filter_for_sections = false, bool section_is_A1 = false);
  bool SelectCandidateFromSuckableSurfaces(int& suck_item_idx, int looking_for_known_item, std::vector<geometry_msgs::PoseStamped> vec_poses_blacklist, bool filter_for_sections = false, bool section_is_A1 = false);
  bool RecognizeItemsInContainer(std::string container);
  bool SearchingPhaseAllBins(int& looking_for_known_item, 
        std::string& dl_target_item, float& dl_item_conf, 
        geometry_msgs::PoseStamped& dl_item_pose_in_world, 
        geometry_msgs::PoseStamped& dl_grasp_line_pt1_pose_in_world, 
        geometry_msgs::PoseStamped& dl_grasp_line_pt2_pose_in_world, 
        std::string& bin_id, std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist);
  bool SearchInSingleBin(int& looking_for_known_item, 
        std::string& dl_target_item, float& dl_item_conf, 
        geometry_msgs::PoseStamped& dl_item_pose_in_world, 
        geometry_msgs::PoseStamped& dl_grasp_line_pt1_pose_in_world, 
        geometry_msgs::PoseStamped& dl_grasp_line_pt2_pose_in_world, 
        std::string& bin_id_including_section, std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist,
        bool filter_for_sections = false, bool section_is_A1 = false);
  bool RetrievalPhase(bool& weight_service_ready, std::string target_item, float item_conf,
        geometry_msgs::PoseStamped item_pose_in_world, 
        geometry_msgs::PoseStamped grasp_line_pt1_pose_in_world, 
        geometry_msgs::PoseStamped grasp_line_pt2_pose_in_world, std::string bin_id,
        bool fuzzyMode = false);
  bool DemocracyPhase(std::string& belief_item_id, float& belief_confidence, 
        bool& weight_service_ready, bool& item_in_order, std::string dl_target_item, 
        float dl_item_conf, geometry_msgs::PoseStamped dl_item_pose_in_world, 
        std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist,
        std::string bin_id, int& looking_for_known_item);
  bool RegisterItemAsDelivered(std::string item_id, std::string origin_bin_id);

  void MoveToRefusalBin(std::string item_id, std::string origin_bin_id, std::string refusal_bin, int item_retrieve_method);
  void MoveToBinAndUpdateOccupancyMap(std::string bin_id_with_section);
  void TakeItemsOutOfBinUntil(std::string bin_id_including_section, std::string breakout_check, std::string refusal_bin);
  bool CheckByWeightIfItemIsFeasible(std::string bin_id);

  // Service declarations
  bool TestMotionsCallback(tnp_task_manager::testMotions::Request &req, tnp_task_manager::testMotions::Response &res);
  bool PickTaskCallback(tnp_task_manager::pickTask::Request &req, tnp_task_manager::pickTask::Response &res);
  bool RapidStowTaskCallback(tnp_task_manager::stowTask::Request &req, tnp_task_manager::stowTask::Response &res);
  bool SetupBeforeRoundCallback(tnp_task_manager::setupBeforeRound::Request &req,
                                tnp_task_manager::setupBeforeRound::Response &res);
  bool InitialSetupCallback(tnp_task_manager::initialSetup::Request &req,
                                tnp_task_manager::initialSetup::Response &res);
  bool Transform2PosesCallback(tnp_task_manager::transform2poses::Request &req,
                                tnp_task_manager::transform2poses::Response &res);

  std::vector<std::string> GetTagsForSuctionCandidatesUsingDLInfo();

  // End Effector
  void ItemIsSuctionedCallback(const std_msgs::Int16::ConstPtr &msg);
  void ItemIsGraspedCallback(const std_msgs::Int16::ConstPtr &msg);
  bool StartSuction();
  bool StopSuction();
  bool DeploySuction();
  bool RetractSuction();
  bool CloseGripper();
  bool OpenGripper();
  bool DeployGripper();
  bool RetractGripper();
  bool SetDrawerAction(const int& drawer_action);
  bool OpenDrawer(); //open
  bool CloseDrawer(); //close the drawer and if stucked shake it and retry close (iteratively)
  bool ShakeDrawer(); //shake the drawer (stays open)
  bool OpenEEShutterCloseTheRest();

  // Vision
  bool VisionSetItemsInfo();
  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

  // Grasp Planner
  bool UpdateOccupancyMap(const std::string& target_container);
  bool GetPoseToPlaceItem(const std::string& target_container, const geometry_msgs::PoseStamped& bounding_box_center_pose,
                          const float& item_width, const float& item_length, const float& item_height);
  bool GetSuctionCandidatesContainerFromOctomap(const std::string& target_container, const int& section_num);

  // Visualization (tnp_monitor)
  void publishPoseMarker(const geometry_msgs::Pose& pose, const std::string& name);

  // Status messages function
  void GenerateStatusMessage(std::string current_state, std::string next_state0, std::string next_state1, std::string next_state2, std::string next_state3);

private:
  ros::NodeHandle n_, n_tmp_;
  //publishers
  ros::Publisher target_item_name_pub_;
  ros::Publisher target_item_point_pub_;
  ros::Publisher target_container_items_pub_;
  ros::Publisher unfulfilled_order_items_pub_;
  ros::Publisher state_machine_msg_pub_;
  ros::Publisher running_status_pub_;
  ros::Publisher current_task_msgs_pub_;
  //subscribers
  ros::Subscriber item_pose_sub_;
  ros::Subscriber confidence_sub_;
  ros::Subscriber sub_item_is_suctioned_;
  ros::Subscriber sub_item_is_grasped_;
  ros::Subscriber sub_camera_info_; // this is the ee camera info

  // services provided
  ros::ServiceServer test_motions_service_;
  ros::ServiceServer pick_task_service_;
  ros::ServiceServer stow_task_service_;
  ros::ServiceServer rapid_stow_task_service_;
  ros::ServiceServer final_task_service_;
  ros::ServiceServer setup_before_round_service_;
  ros::ServiceServer initial_setup_service_;
  ros::ServiceServer transform_2_poses_service_;

  // service clients
  // KUKA motion
  ros::ServiceClient goToHomeClient;
  ros::ServiceClient goToBinClient;
  ros::ServiceClient goToRecSpaceClient;
  ros::ServiceClient goToBoxClient;
  ros::ServiceClient goToToteClient;
  ros::ServiceClient goToLookIntoToteClient;
  ros::ServiceClient goToAmnestyClient;
  ros::ServiceClient moveToJointAnglesPTPClient;
  ros::ServiceClient suckItemClient;
  ros::ServiceClient graspItemClient;
  ros::ServiceClient sweepToteHorizontalClient;
  ros::ServiceClient goToLookIntoBinClient;
  ros::ServiceClient putItemIntoBinClient;
  ros::ServiceClient putItemIntoBoxClient;
  ros::ServiceClient putItemIntoToteClient;
  ros::ServiceClient putItemIntoAmnestyClient;
  ros::ServiceClient putItemIntoContainerClient;
  ros::ServiceClient goToLookIntoContainerClient;
  ros::ServiceClient goToContainerClient;
  ros::ServiceClient EEPoseIsReachableClient;
  // End effector
  ros::ServiceClient suctionClient;
  ros::ServiceClient LinActuatorSuctionClient;
  ros::ServiceClient gripperClient;
  ros::ServiceClient LinActuatorGripperClient;
  ros::ServiceClient BlowerClient;
  ros::ServiceClient DrawerClient;
  ros::ServiceClient shutters_client_;
  // Deep learning
  ros::ServiceClient dl_recognize_items_client_;
  // Weight
  ros::ServiceClient weight_recognize_items_client_;
  ros::ServiceClient get_ready_for_pick_client_;
  ros::ServiceClient weight_set_items_info_client_;
  ros::ServiceClient weight_set_items_location_client_;
  // Recognition Space
  ros::ServiceClient rec_space_recognize_items_client_;
  ros::ServiceClient rec_space_record_empty_rs_client_;
  ros::ServiceClient rec_space_record_scene_client_;
  ros::ServiceClient rec_space_get_bounding_box_client_;
  ros::ServiceClient rec_space_set_items_info_client_;
  // Vision
  ros::ServiceClient vision_set_items_info_client_;
  // Grasp Planner
  ros::ServiceClient get_suction_candidate_C_client_;
  ros::ServiceClient get_grip_candidate_C_client_;
  ros::ServiceClient get_suction_candidate_BB_client_;
  ros::ServiceClient get_grip_candidate_BB_client_;
  ros::ServiceClient update_occupancy_map_client_;
  ros::ServiceClient get_pose_to_place_item_client_;
  ros::ServiceClient get_suction_candidates_container_from_octomap_client_;
  // LED control
  ros::ServiceClient set_led_intensity_client_;
  // Visualization (tnp_monitor)
  ros::ServiceClient show_pose_client_;
  // Actions
  actionlib::SimpleActionClient<tnp_kuka_motion::moveToJointAnglesPTPAction> move_to_joint_angles_PTP_action_client_;
  // SVM
  ros::ServiceClient svm_second_identify_items_client_;

  // members
  ARCItemDatabase database_;
  ARCBoxSizeList box_size_list_;
  ARCLocationList location_list_;
  ARCLocationList initial_location_list_;
  ARCOrderList order_list_;
  ARCBoxLocationList box_location_;
  ARCRetrieveMethodList retrieve_list_;

  // Deep learning
  std::vector<std::string> vec_dl_item_ids_;
  std::vector<float> vec_dl_confidences_;
  std::vector<float> vec_dl_item_center_x_;
  std::vector<float> vec_dl_item_center_y_;
  std::vector<float> vec_dl_item_width_;
  std::vector<float> vec_dl_item_height_;
 
  // Deep learning with the pose in 0,0,0
  std::vector<std::string> vec_unreliable_dl_item_ids_;
  std::vector<geometry_msgs::PoseStamped> vec_unreliable_dl_center_poses_;

  std::vector<geometry_msgs::PoseStamped> vec_dl_item_center_poses_in_cam_;
  //std::vector<geometry_msgs::PoseStamped> vec_dl_item_center_poses_in_world_;
  std::vector<geometry_msgs::PoseStamped> vec_dl_grasp_center_poses_;
  std::vector<geometry_msgs::PoseStamped> vec_dl_grasp_line_pt1_poses_;
  std::vector<geometry_msgs::PoseStamped> vec_dl_grasp_line_pt2_poses_;
  sensor_msgs::CameraInfo cam_info_;

  // Grasp planner
  // suction candidates from grasp planner
  std::vector<geometry_msgs::PoseStamped> vec_suction_candidate_poses_in_world_;
  std::vector<geometry_msgs::PoseStamped> vec_suction_candidate_poses_in_camera_frame_;
  bool free_spot_found_;
  double blacklist_distance_stow_ = .05;

  // Weight node
  std::vector<std_msgs::String> vec_weight_items_ids_;
  std::vector<std_msgs::Float64> vec_weight_items_confs_;
  double measured_weight_;

  //////////////////////////////////////////////////////////////////////////////////////
  /// Recognition Space
  //////////////////////////////////////////////////////////////////////////////////////
  std::ofstream ofs_data_gathering_;
  // Cloud matching
  std::vector<std::string> vec_cloud_matching_items_ids_;
  std::vector<float> vec_cloud_matching_confs_;
  // AKAZE SVM
  std::vector<std::string> vec_akaze_svm_items_ids_;
  std::vector<float> vec_akaze_svm_confs_;
  // Color histogram
  std::vector<std::string> vec_color_histogram_items_ids_;
  std::vector<float> vec_color_histogram_confs_;
  // Bounding box
  tnp_recognition_space::get_bounding_boxResponse bounding_box_information_;
  // SVM Candidates
  std::vector<std::string> vec_svm_items_ids_;
  std::vector<float> vec_svm_confs_;

  // Variables for Services Priority Percentage
  float dl_vote_share_;
  float cloud_matching_vote_share_;
  float akaze_svm_vote_share_;
  float color_histogram_vote_share_;
  float weight_vote_share_;
  float svm_vote_share_;

  float item_is_identified_threshold_;

  // Parameter for debug
  bool debug_on_;

  // Variable for time limit of the functions;
  ros::Duration time_limit_;

  // Variable for the different positions of the points of view once placed upon a tote or bin
  int height_positions_;

  // Variable to store the values from the end effector to determine if item is been sucked or not
  bool item_is_suctioned_;
  bool item_is_grasped_;

  // Variable for max number of attempts to retrieve item
  int max_retrieve_attempts_;

  // Variable to store drawer status (bin_C)
  bool drawer_is_open_;

  // Variables to determine the force to retrieve an item depending on its weight
  float lower_weight_limit_;
  float higher_weight_limit_;
  float suction_force_light_;
  float suction_force_medium_;
  float suction_force_heavy_;
  float gripper_force_light_;
  float gripper_force_medium_;
  float gripper_force_heavy_;

  bool DRAWER_IS_CLOSED_ = false;

  // tf
  tf::TransformListener tf_listener;

  //Grasp planner
  geometry_msgs::PoseStamped pose_to_place_item_in_world_coords_;

  // Variable to define the current and next states to be displayed;
  std::string state_machine_current_state;
  std::vector<std::string> state_machine_next_states;

  // Variable to define the related messages for the current task to be displayed
  tnp_msgs::TaskList task_msgs_;

  // Variable to store led intensities
  int led_intensities_[8];

  bool is_drawer_open_;
  ros::Time round_time_;
};

#endif
