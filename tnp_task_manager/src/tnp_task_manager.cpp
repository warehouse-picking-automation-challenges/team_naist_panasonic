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

#include "tnp_task_manager.h"

#include "tnp_task_manager_helpers.hpp"

using namespace std;

/**
 * This is the constructor of tnp_task_manager
 */
TaskManagerNode::TaskManagerNode() :
    move_to_joint_angles_PTP_action_client_("tnp_kuka_motion/moveToJointAnglesPTPAction", true)
{
  //default init

  // data gathering file
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );
  stringstream ss;

  ss << (now->tm_year + 1900) 
     << (now->tm_mon + 1) 
     <<  now->tm_mday << '_'
     << now->tm_hour << now->tm_min << now->tm_sec;
  ofs_data_gathering_.open("/root/share/"+ss.str()+".txt");

  // Initialization -- everything there what should be there? (1) internal components (2) external components

  // Setup -- stage 2
  // ?

  // Get operative
  // Establish ros services
  setupROSConfiguration();
}

int getch()
{
  ROS_INFO("Press any key to continue...");
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON); // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

// (c) Salvo Virga, sankyu~~
geometry_msgs::PoseStamped transform_pose_now(geometry_msgs::PoseStamped& pose, const std::string& referenceFrame, const tf::TransformListener& listener, 
  const bool& use_now=true)
{
  // Check if the frames are different
  if (pose.header.frame_id != referenceFrame )
  {
    bool success = false;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped result_pose;

    while (!success) {
      try {
        ros::Time t = ros::Time::now();
        if (use_now) {
          pose.header.stamp = t;
        }
        else {
          t = pose.header.stamp;
        }
        listener.waitForTransform(pose.header.frame_id, referenceFrame, t, ros::Duration(3.0));
        listener.lookupTransform(pose.header.frame_id, referenceFrame, t, transform);
        listener.transformPose(referenceFrame, pose, result_pose);
        success = true;
        return result_pose;
      }
      catch (tf::ExtrapolationException &e) {
        ROS_ERROR_STREAM(e.what());
      }
      sleep(0.1);
    }
  }
  return pose;
}

/**
 * sets up services to offer topics and subscribes to main topics
 * @return true, if every subscription worked.
 */
bool TaskManagerNode::setupROSConfiguration()
{
  //Topic you want to publish
  target_item_name_pub_ = n_.advertise<std_msgs::String>("/tnp_task_manager/TargetItemName", 500);
  target_item_point_pub_ = n_.advertise<geometry_msgs::Point>("/tnp_task_manager/TargetItemPoint", 500);
  target_container_items_pub_ = n_.advertise<tnp_msgs::TaskList>("/tnp_task_manager/target_container_items_list", 500);
  unfulfilled_order_items_pub_ = n_.advertise<tnp_msgs::TaskList>("/tnp_task_manger/open_order_list", 500);
  state_machine_msg_pub_ = n_.advertise<tnp_msgs::StateInfo>("/tnp_task_manager/state_machine_info", 500);
  running_status_pub_ = n_.advertise<std_msgs::Bool>("/tnp_task_manager/running_status",500);
  current_task_msgs_pub_ = n_.advertise<tnp_msgs::TaskList>("tnp_task_manager/task_msgs", 500);
  //Topic you want to subscribe
  sub_item_is_suctioned_ = n_.subscribe("/tnp_end_effector/itemIsSuctioned", 1,
                                        &TaskManagerNode::ItemIsSuctionedCallback, this);
  sub_item_is_grasped_ = n_.subscribe("/tnp_end_effector/itemIsGripped", 1,
                                        &TaskManagerNode::ItemIsGraspedCallback, this);
  sub_camera_info_ = n_.subscribe("/camera/depth_registered/sw_registered/camera_info", 1, &TaskManagerNode::cameraInfoCallback, this); //ee camera

  // services you want to advertise
  test_motions_service_ = n_.advertiseService("tnp_task_manager/testMotions", &TaskManagerNode::TestMotionsCallback,
                                              this);
  pick_task_service_ = n_.advertiseService("tnp_task_manager/pickTask", &TaskManagerNode::PickTaskCallback, this);
  rapid_stow_task_service_ = n_.advertiseService("tnp_task_manager/rapidStowTask",
                                                &TaskManagerNode::RapidStowTaskCallback, this);
  setup_before_round_service_ = n_.advertiseService("tnp_task_manager/setup_before_round",
                                                    &TaskManagerNode::SetupBeforeRoundCallback, this);
  initial_setup_service_ = n_.advertiseService("tnp_task_manager/initial_setup",
                                                &TaskManagerNode::InitialSetupCallback, this);
  transform_2_poses_service_ = n_.advertiseService("tnp_task_manager/transform_2_poses",
                                                &TaskManagerNode::Transform2PosesCallback, this);

  // services you want to subscribe to
  // KUKA motion
  goToHomeClient = n_.serviceClient<tnp_kuka_motion::goToHome>("tnp_kuka_motion/goToHome");
  goToBinClient = n_.serviceClient<tnp_kuka_motion::goToBin>("tnp_kuka_motion/goToBin");
  goToRecSpaceClient = n_.serviceClient<tnp_kuka_motion::goToRecSpace>("tnp_kuka_motion/goToRecSpace");
  goToBoxClient = n_.serviceClient<tnp_kuka_motion::goToBox>("tnp_kuka_motion/goToBox");
  goToToteClient = n_.serviceClient<tnp_kuka_motion::goToTote>("tnp_kuka_motion/goToTote");
  goToLookIntoToteClient = n_.serviceClient<tnp_kuka_motion::goToLookIntoTote>("tnp_kuka_motion/goToLookIntoTote");
  goToAmnestyClient = n_.serviceClient<tnp_kuka_motion::goToAmnesty>("tnp_kuka_motion/goToAmnesty");
  goToContainerClient = n_.serviceClient<tnp_kuka_motion::goToContainer>("tnp_kuka_motion/goToContainer");
  moveToJointAnglesPTPClient = n_.serviceClient<tnp_kuka_motion::moveToJointAnglesPTP>(
      "tnp_kuka_motion/moveToJointAnglesPTP");
  goToLookIntoContainerClient = n_.serviceClient<tnp_kuka_motion::goToLookIntoContainer>(
      "tnp_kuka_motion/goToLookIntoContainer");
  suckItemClient = n_.serviceClient<tnp_kuka_motion::suckItem>("tnp_kuka_motion/suckItem");
  graspItemClient = n_.serviceClient<tnp_kuka_motion::graspItem>("tnp_kuka_motion/graspItem");
  sweepToteHorizontalClient = n_.serviceClient<tnp_kuka_motion::sweepToteHorizontal>("tnp_kuka_motion/sweepToteHorizontal");
  goToLookIntoBinClient = n_.serviceClient<tnp_kuka_motion::goToLookIntoBin>("tnp_kuka_motion/goToLookIntoBin");
  putItemIntoBoxClient = n_.serviceClient<tnp_kuka_motion::putItemIntoBox>("tnp_kuka_motion/putItemIntoBox");
  putItemIntoBinClient = n_.serviceClient<tnp_kuka_motion::putItemIntoBin>("tnp_kuka_motion/putItemIntoBin");
  putItemIntoToteClient = n_.serviceClient<tnp_kuka_motion::putItemIntoTote>("tnp_kuka_motion/putItemIntoTote");
  putItemIntoAmnestyClient = n_.serviceClient<tnp_kuka_motion::putItemIntoAmnesty>("tnp_kuka_motion/putItemIntoAmnesty");
  putItemIntoContainerClient = n_.serviceClient<tnp_kuka_motion::putItemIntoContainer>("tnp_kuka_motion/putItemIntoContainer");
  EEPoseIsReachableClient = n_.serviceClient<tnp_moveit_planner::isEEPoseReachable>("tnp_moveit_planner/isEEPoseReachable");
  // End effector
  suctionClient = n_.serviceClient<tnp_end_effector::Suction>("tnp_end_effector/suction");
  LinActuatorSuctionClient = n_.serviceClient<tnp_end_effector::LinActuatorSuction>(
      "tnp_end_effector/lin_actuator_suction");
  gripperClient = n_.serviceClient<tnp_end_effector::Gripper>("tnp_end_effector/gripper");
  LinActuatorGripperClient = n_.serviceClient<tnp_end_effector::LinActuatorGripper>(
      "tnp_end_effector/lin_actuator_gripper");
  BlowerClient = n_.serviceClient<tnp_end_effector::Blower>("tnp_end_effector/blower");
  DrawerClient = n_.serviceClient<tnp_end_effector::Drawer>("tnp_end_effector/drawer");
  shutters_client_ = n_.serviceClient<tnp_end_effector::OpenShutterCloseTheRest>("/tnp_shutters/open_shutter_close_the_rest");
  // tnp_deep_vision (deep learning)
  dl_recognize_items_client_ = n_.serviceClient<tnp_deep_vision::recognize_items>("tnp_deep_vision/recognize_items");
  // tnp_rec_space  // recognition
  rec_space_recognize_items_client_ = n_.serviceClient<tnp_recognition_space::recognize_items>(
      "tnp_recognition_space/recognize_items");
  rec_space_record_scene_client_ = n_.serviceClient<tnp_recognition_space::record_scene>("tnp_recognition_space/record_scene");
  rec_space_record_empty_rs_client_ = n_.serviceClient<tnp_recognition_space::record_empty_rs>("tnp_recognition_space/record_empty_rs");
  rec_space_get_bounding_box_client_ = n_.serviceClient<tnp_recognition_space::get_bounding_box>("tnp_recognition_space/get_bounding_box");
  rec_space_set_items_info_client_ = n_.serviceClient<tnp_recognition_space::set_items_info>("tnp_recognition_space/set_items_info");
  // weight
  get_ready_for_pick_client_ = n_.serviceClient<tnp_weight_events::GetReadyForPick>(
      "tnp_weight_events/get_ready_for_pick");
  weight_recognize_items_client_ = n_.serviceClient<tnp_weight_events::RecognizeItems>(
      "tnp_weight_events/recognize_items");
  weight_set_items_info_client_ = n_.serviceClient<tnp_weight_events::SetItemsInfo>("tnp_weight_events/set_items_info");
  weight_set_items_location_client_ = n_.serviceClient<tnp_weight_events::SetItemsLocation>(
      "tnp_weight_events/set_items_location");
  // Grasp Planner 
  get_grip_candidate_C_client_ = n_.serviceClient<tnp_grasp_planner::getGripCandidates_container>(
    "tnp_grasp_planner/get_grip_candidates_container");
  get_suction_candidate_BB_client_ = n_.serviceClient<tnp_grasp_planner::getSuctionCandidates_bb>(
    "tnp_grasp_planner/get_suction_candidates_bb");
  get_grip_candidate_BB_client_ = n_.serviceClient<tnp_grasp_planner::getGripCandidates_bb>(
    "tnp_grasp_planner/get_grip_candidates_bb");
  update_occupancy_map_client_ = n_.serviceClient<tnp_grasp_planner::updateOccupancyMap>(
      "tnp_grasp_planner/update_occupancy_map");
  get_pose_to_place_item_client_ = n_.serviceClient<tnp_grasp_planner::getPoseToPlaceItem>(
      "tnp_grasp_planner/get_pose_to_place_item");
  get_suction_candidates_container_from_octomap_client_ = n_.serviceClient<tnp_grasp_planner::getSuctionCandidates_container_fromOctomap>(
    "tnp_grasp_planner/get_suction_candidates_container_from_octomap");
  // tnp_led_control
  set_led_intensity_client_ = n_.serviceClient<tnp_led_control::set_led_intensity>(
    "tnp_led_control/set_led_intensity");
  // Visualization (tnp_monitor)
  show_pose_client_ = n_.serviceClient<tnp_msgs::ShowPose>("/tnp_monitor/showPose");

  svm_second_identify_items_client_ = n_.serviceClient<tnp_svm::IdentifyItem>("/tnp_svm_second/identify_item");

  // variables updated from services and topics
  item_is_suctioned_ = false;
  free_spot_found_ = false;

  return true;
}

bool TaskManagerNode::moveToJointAnglesPTP(const double& j1, const double& j2, const double& j3, const double& j4,
                                           const double& j5, const double& j6, const double& j7)
{
  ROS_INFO("Doing the PTP test motions");

  /// Try to execute this via an action.
  tnp_kuka_motion::moveToJointAnglesPTPGoal goal; // This has to be ...PTPGoal, not ActionGoal.
  goal.a1 = j1;
  goal.a2 = j2;
  goal.a3 = j3;
  goal.a4 = j4;
  goal.a5 = j5;
  goal.a6 = j6;
  goal.a7 = j7;
  move_to_joint_angles_PTP_action_client_.sendGoal(goal); // Leaving it like this causes a crazy error

  //wait for the action to return
  bool finished_before_timeout = move_to_joint_angles_PTP_action_client_.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = move_to_joint_angles_PTP_action_client_.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return true;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// KUKA motion
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::PutItemIntoBox(const int& box_id)
{
  tnp_kuka_motion::putItemIntoBox srv;

  ROS_WARN("[TNP_STATE L2] Calling tnp_kuka_motion/putItemIntoBox Calling the service");

  srv.request.box_id = box_id;
  ROS_INFO("Putting item into box num %d", srv.request.box_id);
  ROS_WARN("[TNP_STATE L1] Start tnp_kuka_motion/putItemIntoBox Start putting item in box: %d", srv.request.box_id);
  if (!putItemIntoBoxClient.call(srv))
  {
    ROS_ERROR("Failed to call service /tnp_kuka_motion/putItemIntoBox");
    return false;
  }

  ROS_DEBUG("[TNP_STATE L2] Done tnp_kuka_motion/putItemIntoBox Called successfully");

  return true;
}

bool TaskManagerNode::PutItemIntoBin(const std::string& bin_id)
{
  tnp_kuka_motion::putItemIntoBin srv;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/putItemIntoBin Start calling the service");
  std_msgs::String bin_id_msg;
  bin_id_msg.data = bin_id;
  srv.request.bin_id = bin_id_msg;
  if (!putItemIntoBinClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/putItemIntoBin There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/putItemIntoBin Called successfully");

  return true;
}

bool TaskManagerNode::PutItemIntoTote()
{
  tnp_kuka_motion::putItemIntoTote srv;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/putItemIntoTote Start calling the service");

  if (!putItemIntoToteClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/putItemIntoTote There was an error calling the service");
    return false;
  }

  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/putItemIntoAmnesty Service called successfully");

  return true;
}

bool TaskManagerNode::PutItemIntoAmnesty()
{
  tnp_kuka_motion::putItemIntoAmnesty srv;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/putItemIntoAmnesty Start calling the service");

  if( !putItemIntoAmnestyClient.call( srv ) )
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/putItemIntoAmnesty There was an error calling the service");
    return false;
  }

  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/putItemIntoAmnesty Service called successfully");

  return true;
}

bool TaskManagerNode::PutItemIntoContainer( std::string container_name, bool use_gripper_EE, geometry_msgs::PoseStamped target_pose_in_world )
{
  tnp_kuka_motion::putItemIntoContainer srv;
  srv.request.container_name = container_name;
  srv.request.use_gripper_EE = use_gripper_EE;
  srv.request.target_pose_in_world = target_pose_in_world.pose;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/putItemIntoContainer Start calling the service");

  if( !putItemIntoContainerClient.call( srv ) )
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/putItemIntoContainer There was an error calling the service");
    return false;
  }

    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/putItemIntoContainer There was an error calling the service");
  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/ Service called successfully");

  return true;
}

bool TaskManagerNode::GoToLookIntoContainer( const std::string& container_name, const int& height )
{
  tnp_kuka_motion::goToLookIntoContainer srv;
  srv.request.container_name = container_name;
  srv.request.height = height;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/goToLookIntoContainer Start calling the service");

  if( !goToLookIntoContainerClient.call( srv ) )
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/goToLookIntoContainer There was an error calling the service");
    return false;
  }

  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/goToLookIntoContainer Service called successfully");

  return true;
}

bool TaskManagerNode::GoToLookIntoBin(const std::string& bin_id, const int& point_of_view)
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToLookIntoBin Calling service for going to look into bin %s",
           bin_id.c_str());
  tnp_kuka_motion::goToLookIntoBin srv;
  std_msgs::String bin_id_msg;
  bin_id_msg.data = bin_id;
  srv.request.bin_id = bin_id_msg;
  srv.request.height = point_of_view;
  if (goToLookIntoBinClient.call(srv))
  {
    ROS_INFO("/tnp_kuka_motion/goToLookIntoBin is being called");
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToLookIntoBin Executing the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error /tnp_kuka_motion/goToLookIntoBin There was an error calling the service");
    ROS_ERROR("Failed to call service /tnp_kuka_motion/goToLookIntoBin");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToLookIntoBin Called successfully");
  return true;
}

void TaskManagerNode::publishItem(const std::string& target_item, const geometry_msgs::Pose& target_pose)
{
  std_msgs::String target_item_msg;
  target_item_msg.data = target_item;
  target_item_name_pub_.publish(target_item_msg);
  target_item_point_pub_.publish(target_pose.position);
}

void TaskManagerNode::publishContainerList(const std::string& container_id)
{
  tnp_msgs::TaskList tmp_list;
  if( container_id.compare( "A" ) == 0 )
  {
    tmp_list.info = location_list_.bins[0].content;
  }
  else if( container_id.compare( "B" ) == 0 )
  {
    tmp_list.info = location_list_.bins[1].content;
  }
  else if( container_id.compare( "C" ) == 0 )
  {
    tmp_list.info = location_list_.bins[2].content;
  }
  else if( container_id.compare( "tote" ) == 0 )
  {
    tmp_list.info = location_list_.tote.content;
  }

  target_container_items_pub_.publish( tmp_list );
}

void TaskManagerNode::publishStateInfo(std::string current_state, std::vector<std::string> next_states)
{
  tnp_msgs::StateInfo state_info;
  state_info.current_state = current_state;
  state_info.next_states = next_states;

  state_machine_msg_pub_.publish( state_info );
}

void TaskManagerNode::publishRunningStatus(const bool& running_status){
  std_msgs::Bool running_info;
  running_info.data = running_status;
  running_status_pub_.publish( running_info );

}

bool TaskManagerNode::GoToHome()
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToHome Calling service for going home");
  ROS_INFO("Moving to home configuration");
  tnp_kuka_motion::goToHome srv;
  if (goToHomeClient.call(srv))
  {
    ROS_INFO("/tnp_kuka_motion/goToHome is being called");
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToHome Executing the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToHome There was an error calling the service");
    ROS_ERROR("Failed to call service /tnp_kuka_motion/goToHome");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToHome Service called successfully");
  return true;
}

bool TaskManagerNode::GoToBin(const std::string& bin_id)
{
  tnp_kuka_motion::goToBin srv;
  std_msgs::String bin_id_msg;
  bin_id_msg.data = bin_id;
  srv.request.bin_id = bin_id_msg;
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToBin Calling service for going to bin %s", bin_id.c_str());
  if (goToBinClient.call(srv))
  {
    ROS_INFO("/tnp_kuka_motion/goToBin is being called");
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToBin Executing the service");
  }
  else
  {
    ROS_ERROR("Failed to call service /tnp_kuka_motion/goToBin");
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToBin There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToBin Called successfully");
  return true;
}

bool TaskManagerNode::GoToRecSpace()
{
  ROS_INFO("Moving to RecSpace configuration");
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToRecSpace Start calling service");
  tnp_kuka_motion::goToRecSpace srv;
  if (goToRecSpaceClient.call(srv))
  {
    ROS_INFO("/tnp_kuka_motion/goToRecSpace is being called");
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToRecSpace Executing the service");
  }
  else
  {
    ROS_ERROR("Failed to call service /tnp_kuka_motion/goToRecSpace");
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToRecSpace Error when calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToRecSpace Service called successfully");
  return true;
}

bool TaskManagerNode::GoToBox(const int& box_id)
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToBox Calling service for going to box %d", box_id);
  ROS_INFO("Moving above the box number %d", box_id);
  tnp_kuka_motion::goToBox srv;
  srv.request.box_id = box_id;
  if (goToBoxClient.call(srv))
  {
    ROS_INFO("/tnp_kuka_motion/goToBox is being called");
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToBox Executing the service");
  }
  else
  {
    ROS_ERROR("Failed to call service /tnp_kuka_motion/goToBox");
    ROS_WARN("[TNP_STATE L2] Error /tnp_kuka_motion/goToBox There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToBox Service called successfully");
  return true;
}

bool TaskManagerNode::GoToTote()
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToTote Moving KUKA to tote");
  tnp_kuka_motion::goToTote srv;

  if (goToToteClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToTote Calling the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToTote There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToTote Service called successfully");
  return true;
}

bool TaskManagerNode::GoToContainer(const std::string& container_id, bool keep_A7_fixed)
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToContainer Moving KUKA to %s", container_id.c_str() );
  tnp_kuka_motion::goToContainer srv;
  srv.request.container_name = container_id;
  srv.request.keep_A7_fixed = keep_A7_fixed;

  if (goToContainerClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToContainer Calling the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToContainer There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToContainer Service called successfully");
  return true;
}

bool TaskManagerNode::GoToAmnesty()
{
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToAmnesty Moving KUKA to tote");
  tnp_kuka_motion::goToAmnesty srv;

  if (goToAmnestyClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing /tnp_kuka_motion/goToAmnesty Calling the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToAmnesty There was an error calling the service");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToAmnesty Service called successfully");
  return true;
}

bool TaskManagerNode::GoToLookIntoTote(const int& point_of_view)
{
  ROS_INFO("Attempting to recognize items on tote for point of view %d", point_of_view);
  ROS_WARN("[TNP_STATE L1] Calling /tnp_kuka_motion/goToLookIntoTote Moving KUKA to tote");
  tnp_kuka_motion::goToLookIntoTote srv;
  srv.request.point_of_view.data = point_of_view;

  if (goToLookIntoToteClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Done /tnp_kuka_motion/goToLookIntoTote Called successfully");
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/goToLookIntoTote There was an error calling the service");
    return false;
  }

  return true;
}

bool TaskManagerNode::SuckItem(const std::string& itemName, geometry_msgs::PoseStamped item_pose_in_world, const int& force, bool fuzzyMode, std::string container_name)
{
  bool result = false;

  ROS_WARN("[TNP_STATE L1] Preparing /tnp_kuka_motion/suckItem Start to retrieve item %s", itemName.c_str());

  tnp_kuka_motion::suckItem srv;
  srv.request.target_pose = item_pose_in_world.pose;
  srv.request.force.data = force;
  srv.request.fuzzyMode = fuzzyMode;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/suckItem Start calling the service");
  if (suckItemClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing tnp_kuka_motion/suckItem Executing the service");
    result = true;
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/suckItem There was an error calling the service");
    ROS_ERROR("Failed to call service /tnp_kuka_motion/suckItem");
  }

  return result;
}

bool TaskManagerNode::GraspItem(const std::string &itemName, geometry_msgs::PoseStamped itemPose,
                                geometry_msgs::PoseStamped graspLinePt1, geometry_msgs::PoseStamped graspLinePt2,
                                const int& force, bool fuzzyMode)
{
  bool result = true;

  ROS_WARN("[TNP_STATE L1] Preparing /tnp_kuka_motion/graspItem to retrieve item %s", itemName.c_str());

  ROS_INFO_STREAM("Starting graspItem Function for item: " << itemName);

  geometry_msgs::PoseStamped target_pose_in_world; // grasp center
  geometry_msgs::PoseStamped graspLinePt1_in_world; // where one of the gripper finger should go
  geometry_msgs::PoseStamped graspLinePt2_in_world; // where the other gripper finger should go
  try
  {
    ROS_INFO_STREAM("target_pose.position: " <<itemPose.pose.position);
    ROS_INFO_STREAM("graspLinePt1.position: " <<graspLinePt1.pose.position);
    ROS_INFO_STREAM("graspLinePt2.position: " <<graspLinePt2.pose.position);

    if( (abs(graspLinePt1.pose.position.x) < 1e-6) &&
        (abs(graspLinePt1.pose.position.y) < 1e-6) &&
        (abs(graspLinePt1.pose.position.z) < 1e-6) )
    {
      ROS_WARN("Grasp line points are 0!!!! Adjusting to a default grasp angle.");
      graspLinePt1.pose = itemPose.pose;
      graspLinePt2.pose = itemPose.pose;
      graspLinePt1.pose.position.x += .1;   // Move one of the points to create a line (angle is unimportant)
      graspLinePt1.pose.position.y += .01;
    }

    if (graspLinePt2.header.frame_id.compare("") == 0)
    {
      ROS_ERROR("Frame IDs are empty on the grasp poses!!!! Changing to camera.");
      graspLinePt1.header.frame_id = "/tnp_ee_camera_frame";
      graspLinePt2.header.frame_id = "/tnp_ee_camera_frame";
    }

    target_pose_in_world = transform_pose_now(itemPose,"/iiwa_link_0", tf_listener);
    graspLinePt1_in_world = transform_pose_now(graspLinePt1,"/iiwa_link_0", tf_listener);
    graspLinePt2_in_world = transform_pose_now(graspLinePt2,"/iiwa_link_0", tf_listener);

    ROS_INFO_STREAM("target_pose_in_world " <<target_pose_in_world.pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[TNP_STATE L2] Error tf_listener/transformPose Error calling the service");
    ROS_WARN("[TNP_STATE L1] Error /tnp_kuka_motion/graspItem Error transforming Pose Coordinates");
    ROS_ERROR("Received an exception trying to transform Pose Coordinates: %s", ex.what());
    result = false;
  }

  if (result)
  {
    tnp_kuka_motion::graspItem srv;
    srv.request.target_pose = target_pose_in_world.pose;
    srv.request.pad_point_1 = graspLinePt1_in_world.pose.position;
    srv.request.pad_point_2 = graspLinePt2_in_world.pose.position;
    srv.request.force.data = force;
    srv.request.fuzzyMode = fuzzyMode;
    ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/graspItem Start calling the service");
    if (graspItemClient.call(srv))
    {
      ROS_WARN("[TNP_STATE L1] Executing tnp_kuka_motion/graspItem Executing the service");
    }
    else
    {
      ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/graspItem There was an error calling the service");
      ROS_ERROR("Failed to call service /tnp_kuka_motion/graspItem");
      result = false;
    }
  }
  ROS_WARN("[TNP_STATE L1] Done tnp_kuka_motion/graspItem Service called successfully");
  return result;
}

bool TaskManagerNode::SweepToteHorizontal(double height, bool sweep_to_left)
{
  bool result = false;

  ROS_WARN("[TNP_STATE L1] Preparing /tnp_kuka_motion/sweepToteHorizontal");

  tnp_kuka_motion::sweepToteHorizontal srv;
  srv.request.height = height;
  srv.request.sweepToLeft = sweep_to_left;
  srv.request.palpation_mode = false;
  ROS_WARN("[TNP_STATE L1] Calling tnp_kuka_motion/sweepToteHorizontal Start calling the service");
  if (sweepToteHorizontalClient.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing tnp_kuka_motion/sweepToteHorizontal Executing the service");
    result = true;
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_kuka_motion/sweepToteHorizontal There was an error calling the service");
    ROS_ERROR("Failed to call service /tnp_kuka_motion/sweepToteHorizontal");
  }

  return result;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// End effector
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::StartSuction()
{
  ROS_INFO("Starting suction");
  tnp_end_effector::Suction srv;
  srv.request.setSuctionState = true;
  if (suctionClient.call(srv))
  {
    ROS_INFO("SuctionService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionService");
    return false;
  }
  return true;
}

bool TaskManagerNode::StopSuction()
{
  ROS_INFO("Stopping suction");
  tnp_end_effector::Suction srv;
  srv.request.setSuctionState = false;
  if (suctionClient.call(srv))
  {
    ROS_INFO("SuctionService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/SuctionService");
    return false;
  }
  return true;
}

bool TaskManagerNode::DeploySuction()
{
  ROS_INFO("Deploying Suction");
  tnp_end_effector::LinActuatorSuction srv;
  srv.request.setLinActuatorState = true;
  if (LinActuatorSuctionClient.call(srv))
  {
    ROS_INFO("LinActuatorSuctionService is being called from task manager");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorSuctionService");
    return false;
  }
  return true;
}

bool TaskManagerNode::RetractSuction()
{
  ROS_INFO("Retracting Suction");
  tnp_end_effector::LinActuatorSuction srv;
  srv.request.setLinActuatorState = false;
  if (LinActuatorSuctionClient.call(srv))
  {
    ROS_INFO("LinActuatorSuctionService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorSuctionService");
    return false;
  }
  return true;
}

bool TaskManagerNode::OpenGripper()
{
  ROS_INFO("Opening gripper");
  tnp_end_effector::Gripper srv;

  std_msgs::String gripper_control_method;
  std_msgs::Int16 gripper_control_parameter;
  gripper_control_method.data = "position_control";
  gripper_control_parameter.data = 110;


  srv.request.gripper_control_method = gripper_control_method;
  srv.request.gripper_control_parameter = gripper_control_parameter;
  if (gripperClient.call(srv))
  {
    ROS_INFO("GripperService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/GripperService");
    return false;
  }

  return true;
}

bool TaskManagerNode::DeployGripper()
{
  ROS_INFO("Deploying gripper");
  tnp_end_effector::LinActuatorGripper srv;
  srv.request.setLinActuatorState = true;
  if (LinActuatorGripperClient.call(srv))
  {
    ROS_INFO("LinActuatorGripperService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorGripperService");
    return false;
  }
  return true;
}

bool TaskManagerNode::RetractGripper()
{
  ROS_INFO("Retracting gripper");
  tnp_end_effector::LinActuatorGripper srv;
  srv.request.setLinActuatorState = false;
  if (LinActuatorGripperClient.call(srv))
  {
    ROS_INFO("LinActuatorGripperService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/LinActuatorGripperService");
    return false;
  }
  return true;
}


bool TaskManagerNode::SetDrawerAction(const int& drawer_action) //0:close, 1:open, 2:shake
{
  std::string drawer_action_description = "close";
  int valid_drawer_action = drawer_action;
  drawer_is_open_ = drawer_action == 0 ? false : true;
  if (drawer_action == 0) drawer_action_description = "close"; // closes and if stuck then shakes and keeps trying to close it
  else if (drawer_action == 1) drawer_action_description = "open"; // open the drawer
  else if (drawer_action == 2) drawer_action_description = "shake"; // stays open after shaking
  else
  {
    ROS_ERROR_STREAM("TaskManagerNode::SetDrawerAction: Invalid drawer action: " << drawer_action);
    valid_drawer_action = 0; // close the drawer as default action
    ROS_ERROR_STREAM("Setting the drawer action to default: " << drawer_action_description);
  }

  ROS_INFO_STREAM("Set drawer action to: " << drawer_action_description);
  ROS_DEBUG_STREAM("[TNP_STATE L1] Calling /tnp_end_effector/drawer Setting drawer action to " << drawer_action_description);

  //call the service
  tnp_end_effector::Drawer srv;
  srv.request.setDrawerAction.data = drawer_action;
  if (DrawerClient.call(srv))
  {
    ROS_INFO("DrawerClient is being called");
    ROS_DEBUG_STREAM("[TNP_STATE L1] Executing /tnp_end_effector/drawer Drawer action: " << drawer_action_description);
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/drawer");
    ROS_DEBUG("[TNP_STATE L1] Error /tnp_end_effector/drawer Error when calling the service");
    return false;
  }
  return true;
}

bool TaskManagerNode::CloseDrawer()
{
  SetDrawerAction(0); //0:close, 1:open, 2:shake
  return true;
}

bool TaskManagerNode::OpenDrawer()
{
  SetDrawerAction(1); //0:close, 1:open, 2:shake
  return true;
}

bool TaskManagerNode::ShakeDrawer()
{
  SetDrawerAction(2); //0:close, 1:open, 2:shake
  return true;
}

bool TaskManagerNode::OpenEEShutterCloseTheRest()
{
  ROS_INFO("Opening EE shutter, closign the rest");
  tnp_end_effector::OpenShutterCloseTheRest srv;
  std_msgs::String msg;
  msg.data = "E"; //end effector camera shutter
  srv.request.camera_id = msg;
  if (shutters_client_.call(srv))
  {
    ROS_INFO("OpenShutterCloseTheRestService is being called");
  }
  else
  {
    ROS_ERROR("Failed to call service tnp_end_effector/OpenShutterCloseTheRestService");
    return false;
  }
  return true;
}

bool TaskManagerNode::EEPoseIsReachable(geometry_msgs::Pose EE_pose, bool use_gripper_EE)
{
  tnp_moveit_planner::isEEPoseReachable srv;
  srv.request.EE_target_pose = EE_pose;
  srv.request.use_gripper_EE = use_gripper_EE;
  if (EEPoseIsReachableClient.call(srv))
  {
    ROS_DEBUG("[TNP_STATE L3] Calling tnp_moveit_planner/isEEPoseReachable");
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L3] Error tnp_moveit_planner/isEEPoseReachable");
    ROS_ERROR("Failed to call service tnp_moveit_planner/isEEPoseReachable");
    return false;
  }
  ROS_INFO_STREAM("isEEPoseReachable was called. Result: " << srv.response.pose_is_reachable);
  return srv.response.pose_is_reachable;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Task Manager
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::LoadFiles(std::string task_name)
{
  bool result = true;

  ROS_INFO("Loading Items Location");
  location_list_.loadJson("/root/share/PutAmazonDataHere/item_location_file.json");

  ROS_INFO("Loading Items Information");
  database_.loadItems("/root/share/PutAmazonDataHere", "/root/share/PutAmazonDataHere/our_prior_knowledge.json");

  initial_location_list_ = location_list_;

  return result;
}

bool TaskManagerNode::LoadBoxesAndOrders()
{
  bool result = true;
  ROS_INFO("Getting box location from parameters");

  ROS_INFO("Loading Box Sizes");
  box_size_list_.loadJson("/root/share/PutAmazonDataHere/box_sizes.json");
  
  for (int i = 0; i < 3; ++i)
  {
    char tmp[50] = "";
    sprintf(tmp, "/tnp_environment/box_%d_size", i + 1);
    if (!ros::param::has(tmp))
    {
      result = false;
      break;
    }
  }


  if (result)
  {
    for (int i = 0; i < 3; ++i)
    {
      ARCBoxLocation boxLocation;
      char tmp[50] = "";

      boxLocation.boxId = i + 1;

      sprintf(tmp, "/tnp_environment/box_%d_size", i + 1);
      ros::param::get(tmp, boxLocation.sizeId);

      boxLocation.orderId = -1;
      box_location_.boxLocationList.push_back(boxLocation);
    }

    ROS_INFO("Loading Orders Information");
    order_list_.loadJson("/root/share/PutAmazonDataHere/order_file.json");
    for (int i = 0; i < order_list_.listOrders.size(); ++i)
    {
      for (int j = 0; j < box_location_.boxLocationList.size(); ++j)
      {
        if (order_list_.listOrders[i].sizeId.compare(box_location_.boxLocationList[j].sizeId) == 0
            && box_location_.boxLocationList[j].orderId == -1)
        {
          box_location_.boxLocationList[j].orderId = i;
          break;
        }
      }

      for (int j = 0; j < order_list_.listOrders[i].content.size(); ++j)
      {
        location_list_.getLocation(order_list_.listOrders[i].content[j], order_list_.listOrders[i].locationType[j],
                                   order_list_.listOrders[i].locationId[j]);
      }
    }

    for( int i = 0; i < 3; ++i )
    {
      location_list_.boxes[i].sizeId = box_location_.boxLocationList[i].sizeId;
    }
  }
  else
  {
    ROS_ERROR("/tnp_environment/box_n_size parameter were not found cannot continue");
  }

  return result;
}

bool TaskManagerNode::LoadParameters()
{
  bool result = true;

  bool vote_share_params_found = false;
  if( ros::param::has( "/tnp_task_manager/dl_vote_share" ) )
  {
    if( ros::param::has( "/tnp_task_manager/weight_vote_share" ) )
    {
      if( ros::param::has( "/tnp_task_manager/cloud_matching_vote_share" ) )
      {
        if( ros::param::has( "/tnp_task_manager/akaze_svm_vote_share") )
        {
          if( ros::param::has( "/tnp_task_manager/color_histogram_vote_share" ) )
          {
            if( ros::param::has( "/tnp_task_manager/svm_vote_share" ) )
            {
              ros::param::get( "/tnp_task_manager/dl_vote_share", dl_vote_share_ );
              ros::param::get( "/tnp_task_manager/cloud_matching_vote_share", cloud_matching_vote_share_ );
              ros::param::get( "/tnp_task_manager/akaze_svm_vote_share", akaze_svm_vote_share_ );
              ros::param::get( "/tnp_task_manager/color_histogram_vote_share",color_histogram_vote_share_ );
              ros::param::get( "/tnp_task_manager/weight_vote_share", weight_vote_share_ );
              ros::param::get( "/tnp_task_manager/svm_vote_share", svm_vote_share_ );
              vote_share_params_found = true;
            }
            else
            {
              ROS_ERROR( "Missing /tnp_task_manager/svm_vote_share parameter, setting defaults" );
            }
          }
          else
          {
            ROS_ERROR( "Missing /tnp_task_manager/color_histogram_vote_share parameter, setting defaults" );
          }
        }
        else
        {
          ROS_ERROR( "Missing /tnp_task_manager/akaze_svm_vote_share parameter, setting defaults" );
        }
      }
      else
      {
        ROS_ERROR( "Missing /tnp_task_manager/cloud_matching_vote_share parameter, setting defaults" );
      }
    }
    else
    {
      ROS_ERROR( "Missing /tnp_task_manager/weight_vote_share parameter, setting defaults" );
    }
  }
  else
  {
    ROS_ERROR( "Missing /tnp_task_manager/dl_vote_share parameter, setting defaults" );
  }

  if( !vote_share_params_found )
  {
    dl_vote_share_ = 0.3;
    cloud_matching_vote_share_ = 0.1;
    akaze_svm_vote_share_ = 0.1;
    color_histogram_vote_share_ = 0.1;
    weight_vote_share_ = 0.3;
    svm_vote_share_ = 0.1;
  }

  ROS_INFO_STREAM( "dl_vote_share: " << dl_vote_share_ );
  ROS_INFO_STREAM( "weight_vote_share: " << weight_vote_share_ );
  ROS_INFO_STREAM( "svm_vote_share: " << svm_vote_share_ );
  ROS_INFO_STREAM( "cloud_matching_vote_share: " << cloud_matching_vote_share_ );
  ROS_INFO_STREAM( "akaze_svm_vote_share: " << akaze_svm_vote_share_ );
  ROS_INFO_STREAM( "color_histogram_vote_share: " << color_histogram_vote_share_ );

  if( ros::param::has( "/tnp_task_manager/item_is_identified_threshold" ) )
  {
    ros::param::get( "/tnp_task_manager/item_is_identified_threshold", item_is_identified_threshold_ );
  }
  else
  {
    ROS_ERROR( "Missing /tnp_task_manager/item_is_identified_threshold parameter, setting default" );
    item_is_identified_threshold_ = 0.4;
  }
  ROS_INFO_STREAM( "total_confidence_threshold: " << item_is_identified_threshold_ );

  if( ros::param::has( "/tnp_task_manager/time_limit" ) )
  {
    float tmp_time;
    ros::param::get( "/tnp_task_manager/time_limit", tmp_time );
    time_limit_ = ros::Duration( tmp_time );
  }
  else
  {
    ROS_ERROR( "Missing /tnp_task_manager/time_limit parameter, setting default" );
    time_limit_ = ros::Duration( 900.0 );
  }
  ROS_INFO_STREAM( "time_limit: " << std::setprecision(2) << time_limit_ );

  if( ros::param::has( "/tnp_task_manager/height_positions" ) )
  {
    ros::param::get( "/tnp_task_manager/height_positions", height_positions_ );
  }
  else
  {
    height_positions_ = 3;
  }
  ROS_INFO_STREAM( "height_positions: " << height_positions_ );

  if( ros::param::has( "/tnp_task_manager/debug" ) )
  {
    ros::param::get( "/tnp_task_manager/debug", debug_on_ );
  }
  else
  {
    debug_on_ = false;
  }
  ROS_INFO( "Debug flag set to: %s", debug_on_ ? "True" : "False" );

  if( ros::param::has( "/tnp_task_manager/max_attempts" ) )
  {
    ros::param::get( "/tnp_task_manager/max_attempts", max_retrieve_attempts_ );
  }
  else
  {
    max_retrieve_attempts_ = 3;
  }
  ROS_INFO( "Max number of attempts: %d", max_retrieve_attempts_ );

  if( ros::param::has( "/tnp_task_manager/lower_weight_limit" ) )
  {
    ros::param::get( "/tnp_task_manager/lower_weight_limit", lower_weight_limit_ );
  }
  else
  {
    lower_weight_limit_ = 0.5;
  }
  ROS_INFO( "Lower weight limit: %.2f", lower_weight_limit_ );

  if( ros::param::has( "/tnp_task_manager/higher_weight_limit" ) )
  {
    ros::param::get( "/tnp_task_manager/higher_weight_limit", higher_weight_limit_ );
  }
  else
  {
    higher_weight_limit_ = 0.75;
  }
  ROS_INFO( "Higher weight limit: %.2f", higher_weight_limit_ );

  if( ros::param::has( "/tnp_task_manager/suction_force_light" ) )
  {
    ros::param::get( "/tnp_task_manager/suction_force_light", suction_force_light_ );
  }
  else
  {
    suction_force_light_ = 25;
  }
  ROS_INFO( "Suction force light: %.2f", suction_force_light_ );

  if( ros::param::has( "/tnp_task_manager/suction_force_medium" ) )
  {
    ros::param::get( "/tnp_task_manager/suction_force_medium", suction_force_medium_ );
  }
  else
  {
    suction_force_medium_ = 30;
  }
  ROS_INFO( "Suction force medium: %.2f", suction_force_medium_ );

  if( ros::param::has( "/tnp_task_manager/suction_force_heavy" ) )
  {
    ros::param::get( "/tnp_task_manager/suction_force_heavy", suction_force_heavy_ );
  }
  else
  {
    suction_force_heavy_ = 40;
  }
  ROS_INFO( "Suction force heavy: %.2f", suction_force_heavy_ );

  if( ros::param::has( "/tnp_task_manager/gripper_force_light" ) )
  {
    ros::param::get( "/tnp_task_manager/gripper_force_light", gripper_force_light_ );
  }
  else
  {
    gripper_force_light_ = 55;
  }
  ROS_INFO( "Grip force light: %.2f", gripper_force_light_ );

  if( ros::param::has( "/tnp_task_manager/gripper_force_medium" ) )
  {
    ros::param::get( "/tnp_task_manager/gripper_force_medium", gripper_force_medium_ );
  }
  else
  {
    gripper_force_medium_ = 65;
  }
  ROS_INFO( "Grip force medium: %.2f", gripper_force_medium_ );

  if( ros::param::has( "/tnp_task_manager/gripper_force_heavy" ) )
  {
    ros::param::get( "/tnp_task_manager/gripper_force_heavy", gripper_force_heavy_ );
  }
  else
  {
    gripper_force_heavy_ = 75;
  }
  ROS_INFO( "Grip force heavy: %.2f", gripper_force_heavy_ );

  for( int i = 0; i < 8; ++i )
  {
    if( ros::param::has( "/tnp_task_manager/led_" + std::to_string(i+1) + "_intensity" ) )
    {
      ros::param::get( "/tnp_task_manager/led_" + std::to_string(i+1) + "_intensity", led_intensities_[i] );
    }
    else
    {
      led_intensities_[i] = 20;
    }
    ROS_INFO( "Led %d intensity: %d", i, led_intensities_[i] );
  }

  return result;
}

bool TaskManagerNode::OrderListCompleted()
{
  bool result = true;

  for (int i = 0; i < order_list_.listOrders.size(); ++i)
  {
    if (order_list_.listOrders[i].orderProcessed == 0)
    {
      result = false;
      break;
    }
  }

  return result;
}

bool TaskManagerNode::ItemIsKnown(const std::string &item_id)
{
  bool result = false;
  for( int i_database_index = 0; i_database_index < database_.listItems.size(); ++i_database_index )
  {
    if( database_.listItems[i_database_index].item_id.compare( item_id ) == 0 )
    {
      result = database_.listItems[i_database_index].known;
      break;
    }
  }

  return result;
}

bool TaskManagerNode::OrderItemsInBin(const std::string &bin_id, const std::string& search_criterion)
{
  bool result = false;
  int match = 0;

  for (int i = 0; i < order_list_.listOrders.size(); ++i)
  {
    for (int j = 0; j < order_list_.listOrders[i].content.size() && order_list_.listOrders[i].orderProcessed == 0; ++j)
    {
      if (order_list_.listOrders[i].itemProcessed[j] == 0
          && order_list_.listOrders[i].locationType[j].compare("bin") == 0)
      {
        if (order_list_.listOrders[i].locationId[j].compare(bin_id) == 0)
        {
          if( search_criterion.compare( "known" ) == 0 )
          {
            if( ItemIsKnown(order_list_.listOrders[i].content[j]) )
            {
              match = 1;
              break;
            }
          }
          else if( search_criterion.compare( "unknown" ) == 0 )
          {
            if( !ItemIsKnown(order_list_.listOrders[i].content[j]) )
            {
              match = 1;
              break;
            }
          }
          else
          {
            match = 1;
            break;
          }
        }
      }
    }

    if (match == 1)
    {
      break;
    }
  }

  if (match == 1)
  {
    result = true;
  }
  else
  {
    result = false;
  }

  return result;
}

bool TaskManagerNode::ItemInOrder(const std::string &item_id, int &order_id)
{
  bool result = true;
  int match = 0;
  int dl_item_idx;
  std::vector<std_msgs::String> vec_bin_id;
  std::vector<int> item_count;

  for (int i = 0; i < order_list_.listOrders.size(); ++i)
  {
    for (int j = 0; j < order_list_.listOrders[i].content.size() && order_list_.listOrders[i].orderProcessed == 0; ++j)
    {
      if (order_list_.listOrders[i].itemProcessed[j] == 0
          && order_list_.listOrders[i].content[j].compare(item_id) == 0 && match == 0)
      {
        order_id = i;
        match = 1;
        break;
      }
    }
  }

  if (match == 0)
  {
    result = false;
  }

  return result;
}

bool TaskManagerNode::ItemInTote(const std::string& item_id)
{
  bool result = true;
  int match = 0;
  int dl_item_idx;
  std::vector<std_msgs::String> vec_bin_id;
  std::vector<int> item_count;

  for (int i = 0; i < location_list_.tote.content.size(); ++i)
  {
    if (item_id.compare(location_list_.tote.content[i]) == 0)
    {
      match = 1;
      break;
    }
  }

  if (match == 0)
  {
    result = false;
  }

  return result;
}

bool TaskManagerNode::ItemInBin(const std::string& bin_id, const std::string& item_id)
{
  bool result = true;
  int match = 0;
  int dl_item_idx;

  for (int i = 0; i < location_list_.bins.size(); ++i)
  {
    if (bin_id.compare(location_list_.bins[i].id) == 0)
    {
      for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
      {
        if (item_id.compare(location_list_.bins[i].content[j]) == 0)
        {
          match = 1;
          break;
        }
      }
    }
  }

  if (match == 0)
  {
    result = false;
  }

  return result;
}

bool TaskManagerNode::GetBoxInfo(const int &order_id, int& box_id)
{
  bool result = false;

  for (int i = 0; i < box_location_.boxLocationList.size(); ++i)
  {
    if (box_location_.boxLocationList[i].orderId == order_id)
    {
      box_id = box_location_.boxLocationList[i].boxId;
      result = true;
    }
  }

  return result;
}

bool TaskManagerNode::GetRetrieveMethod(const std::string &item_id, int& retrieve_method, int& retrieve_force)
{
  bool result = false;

  for( int i = 0; i < database_.listItems.size(); ++i )
  {
    if( database_.listItems[i].item_id.compare( item_id ) == 0 )
    {
      retrieve_method = database_.listItems[i].retrieve_method;
      if( database_.listItems[i].force > 0 )
      {
        retrieve_force = database_.listItems[i].force;
      }
      else
      {
        if( database_.listItems[i].weight < lower_weight_limit_ )
        {
          retrieve_force = retrieve_method == 0 ? suction_force_light_ : gripper_force_light_;
        }
        else if( database_.listItems[i].weight < higher_weight_limit_ )
        {
          retrieve_force = retrieve_method == 0 ? suction_force_medium_ : gripper_force_medium_;
        }
        else
        {
          retrieve_force = retrieve_method == 0 ? suction_force_heavy_ : gripper_force_heavy_;
        }
      }

      if( retrieve_force > 0 )
      {
        result = true;
      }

      break;
    }
  }

  return result;
}

bool TaskManagerNode::DetermineBinBySize(const std::string& item_id, std::string& bin_id)
{
  bool result = true;

  for (int i = 0; i < database_.listItems.size(); ++i)
  {
    if (item_id.compare(database_.listItems[i].item_id) == 0)
    {
      float tmp_x, tmp_y, tmp_z, tmp_vol;
      std::vector<std::string> forbidden_bins;
      tmp_x = database_.listItems[i].dim_x * 100;
      tmp_y = database_.listItems[i].dim_y * 100;
      tmp_z = database_.listItems[i].dim_z * 100;
      tmp_vol = tmp_x * tmp_y * tmp_z;
      forbidden_bins = database_.listItems[i].forbidden_bins;

      if( tmp_x < 15. && tmp_y < 15. && tmp_z < 15. && tmp_vol < 650. && forbidden_bins[0].compare( "B" ) != 0  )
      {
        if( tmp_x < 9. && tmp_y < 9. && tmp_z < 9. && forbidden_bins[0].compare( "C" ) != 0 &&
          forbidden_bins[1].compare( "C" ) != 0 )
        {
          bin_id = "C";
        }
        else
        {
          bin_id = "B";
        }
      }
      else
      {
        bin_id = "A";
      }
    }
  }

  return result;
}

bool TaskManagerNode::UpdateItemLocationInMemory(const std::string &item_id, const std::string &origin_container,
                                               const std::string &destination_container)
{
  bool result = true;
  int located = 0;
  int tmp_i = 0;

  if (origin_container.substr(0, 3).compare("bin") == 0)
  {
    for (int i = 0; i < location_list_.bins.size() && located == 0; ++i)
    {
      if (origin_container.compare("bin_"+location_list_.bins[i].id) == 0)
      {
        for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
        {
          if (item_id.compare(location_list_.bins[i].content[j]) == 0)
          {
            location_list_.bins[i].content.erase(location_list_.bins[i].content.begin() + j);
            tmp_i = i;
            located = 1;
            break;
          }
        }
      }
    }
  }
  else if (origin_container.substr(0, 3).compare("box") == 0)
  {
    for (int i = 0; i < location_list_.boxes.size() && located == 0; ++i)
    {
      if (origin_container.compare("box_"+location_list_.boxes[i].id) == 0)
      {
        for (int j = 0; j < location_list_.boxes[i].content.size(); ++j)
        {
          if (item_id.compare(location_list_.boxes[i].content[j]) == 0)
          {
            location_list_.boxes[i].content.erase(location_list_.boxes[i].content.begin() + j);
            tmp_i = i;
            located = 1;
            break;
          }
        }
      }
    }
  }
  else if (origin_container.substr(0, 4).compare("tote") == 0)
  {
    for (int i = 0; i < location_list_.tote.content.size(); ++i)
    {
      if (item_id.compare(location_list_.tote.content[i]) == 0)
      {
        location_list_.tote.content.erase(location_list_.tote.content.begin() + i);
        located = 1;
        break;
      }
    }
  }

  if (located == 1)
  {
    located = 0;
    if (destination_container.substr(0, 3).compare("bin") == 0)
    {
      for (int i = 0; i < location_list_.bins.size(); ++i)
      {
        if (destination_container.compare("bin_"+location_list_.bins[i].id) == 0)
        {
          location_list_.bins[i].content.push_back(item_id);
          located = 1;
          break;
        }
      }
    }
    else if (destination_container.substr(0, 3).compare("box") == 0)
    {
      for (int i = 0; i < location_list_.boxes.size(); ++i)
      {
        if (destination_container.compare("box_"+location_list_.boxes[i].id) == 0)
        {
          location_list_.boxes[i].content.push_back(item_id);
          located = 1;
          break;
        }
      }
    }
    else if (destination_container.substr(0, 4).compare("tote") == 0)
    {
      location_list_.tote.content.push_back(item_id);
      located = 1;
    }

    if (located == 0)
    {
      //Failed to update, location id not found
      ROS_ERROR("Failed to update location list, destination container [%s] not found",
                destination_container.c_str());
      if (origin_container.substr(0, 3).compare("bin") == 0)
      {
        location_list_.bins[tmp_i].content.push_back(item_id);
      }
      else if (origin_container.substr(0, 3).compare("box") == 0)
      {
        location_list_.boxes[tmp_i].content.push_back(item_id);
      }
      else if (origin_container.substr(0, 4).compare("tote") == 0)
      {
        location_list_.tote.content.push_back(item_id);
      }

      result = false;
    }
  }
  else
  {
    //Failed to update, item not found
    ROS_ERROR("Failed to update location list, item id [%s] not found", item_id.c_str());
    result = false;
  }

  return result;
}

void TaskManagerNode::PublishOrdersUnfulfilledItems()
{
  tnp_msgs::TaskList unfulfilled_order_items_vec;
  char tmp[50] = "";

  for( int i = 0; i < order_list_.listOrders.size(); ++i )
  {
    if( i != 0 )
    {
      sprintf( tmp, " " );
      unfulfilled_order_items_vec.info.push_back( tmp );
    }

    if( order_list_.listOrders[i].orderProcessed == 1 )
    {
      sprintf( tmp, "Order %d, completed!", i + 1 );
      unfulfilled_order_items_vec.info.push_back( tmp );
      continue;
    }
    else
    {
      sprintf( tmp, "Remaining items in order %d:", i + 1 );
      unfulfilled_order_items_vec.info.push_back( tmp );
    }

    for( int j = 0; j < order_list_.listOrders[i].content.size(); ++j )
    {
      if( order_list_.listOrders[i].itemProcessed[j] == 0 )
      {
        sprintf( tmp, "   - %s", order_list_.listOrders[i].content[j].c_str() );
        unfulfilled_order_items_vec.info.push_back( tmp );
      }
    }
  }

  unfulfilled_order_items_pub_.publish(unfulfilled_order_items_vec);
}

void TaskManagerNode::PublishTaskMessages(std::string msg, bool append)
{
  try{
    if( !append )
    {
      task_msgs_.info.clear();
    }

    task_msgs_.info.push_back( msg );
    current_task_msgs_pub_.publish( task_msgs_ );
  }
  catch(...)
  {
    //avoid stalling the application due to visualization errors
  }
}

bool TaskManagerNode::UpdateOrderLocationInMemory(const int &order_id, const std::string &item_id,
                                                const std::string &location_type, const std::string &location_id)
{
  bool result = false;
  bool is_item_in_order = false;

  for (int j = 0; j < order_list_.listOrders[order_id].content.size(); ++j)
  {
    if (order_list_.listOrders[order_id].content[j].compare(item_id) == 0)
    {
      order_list_.listOrders[order_id].itemProcessed[j] = 1;
      order_list_.listOrders[order_id].locationType[j] = location_type;
      order_list_.listOrders[order_id].locationId[j] = location_id;
      is_item_in_order = true;
      break;
    }
  }

  if (is_item_in_order)
  {
    bool all_items_processed = true;
    for (int i = 0; i < order_list_.listOrders[order_id].content.size(); ++i)
    {
      if (order_list_.listOrders[order_id].itemProcessed[i] == 0)
      {
        all_items_processed = false;
        break;
      }
    }

    if (all_items_processed)
    {
      order_list_.listOrders[order_id].orderProcessed = 1;
    }
  }
  else
  {
    //Failed to update, item not found
    ROS_ERROR("Failed to update order list, item id [%s] not found", item_id.c_str());
  }

  return result;
}

bool TaskManagerNode::TimeLimitReached(const ros::Time& begin_time)
{
  bool result = false;
  ros::Time now = ros::Time::now();
  ros::Duration time_diff = now - begin_time;

  if( time_diff.toSec() >= time_limit_.toSec() )
  {
    ROS_INFO("Time limit reached");
    result = true;
  }
  else
  {
    float j = ceil( time_diff.toSec() / 60.0 );
    float k = 60.0 * ( j - 1 );

    ROS_INFO( "%.0f minutes and %.0f seconds remaining",
      ( time_limit_.toSec() / 60.0 ) - j,
      60.0 - ( ceil( time_diff.toSec() ) - k ) );
  }

  return result;
}

void TaskManagerNode::DrawerTimeLimit(const ros::Time& begin_time)
{
  ros::Time now = ros::Time::now();
  ros::Duration time_diff = now - begin_time;

  while( time_diff.toSec() < time_limit_.toSec() - 30.0 )
  {
    ros::Duration(1.0).sleep();
    now = ros::Time::now();
    time_diff = now - begin_time;
  }

  ROS_INFO( "Time limit for drawer reached" );
  CloseDrawer();
}

bool TaskManagerNode::RetrieveItem(const int& item_retrieve_method, const std::string& item_id, geometry_msgs::PoseStamped item_pose,
                                   geometry_msgs::PoseStamped graspLinePt1, geometry_msgs::PoseStamped graspLinePt2,
                                   const int& retrieve_force, bool fuzzyMode, std::string container_name)
{
  bool result = true;

  if (item_retrieve_method == 0) //suction
  {

    if (!SuckItem(item_id, item_pose, retrieve_force, fuzzyMode, container_name))
    {
      result = false;
    }
  }
  else if(item_retrieve_method == 1) //gripper
  {
    if (!GraspItem(item_id, item_pose, graspLinePt1, graspLinePt2, retrieve_force, fuzzyMode))
    {
      result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unknown item_retrieve_method "<< item_retrieve_method << ". Using suction");
    if (!SuckItem(item_id, item_pose, retrieve_force, fuzzyMode, container_name))
    {
      result = false;
    }
  }

  return result;
}

bool TaskManagerNode::DLInitializeVectors()
{
  vec_dl_item_ids_.clear();
  vec_dl_item_center_poses_in_cam_.clear();
  vec_dl_confidences_.clear();
  vec_dl_item_center_x_.clear();
  vec_dl_item_center_y_.clear();
  vec_dl_item_width_.clear();
  vec_dl_item_height_.clear();

  vec_weight_items_confs_.clear();
  vec_weight_items_ids_.clear();

  return true;
}

std::vector<std::string> TaskManagerNode::GetContainerItems( const std::string container_id, std::string type_of_list )
{
  std::vector<std::string> vec_container_items;

  if( type_of_list.compare( "initial" ) == 0 )
  {
    if (container_id.substr(0, 3).compare("bin") == 0)
    {
      for (int i = 0; i < initial_location_list_.bins.size(); ++i)
      {
        if (container_id.compare("bin_"+initial_location_list_.bins[i].id) == 0)
        {
          for (int j = 0; j < initial_location_list_.bins[i].content.size(); ++j)
          {
            vec_container_items.push_back( initial_location_list_.bins[i].content[j] );
          }
        }
      }
    }
    else if (container_id.substr(0, 3).compare("box") == 0)
    {
      for (int i = 0; i < initial_location_list_.boxes.size(); ++i)
      {
        if (container_id.compare("box_"+initial_location_list_.boxes[i].id) == 0)
        {
          for (int j = 0; j < initial_location_list_.boxes[i].content.size(); ++j)
          {
            vec_container_items.push_back( initial_location_list_.boxes[i].content[j] );
          }
        }
      }
    }
    else if( container_id.substr( 0, 4 ).compare( "tote" ) == 0 )
    {
      for( int i = 0; i < initial_location_list_.tote.content.size(); ++i)
      {
        vec_container_items.push_back( initial_location_list_.tote.content[i] );
      }
    }
  }
  else
  {
    if (container_id.substr(0, 3).compare("bin") == 0)
    {
      for (int i = 0; i < location_list_.bins.size(); ++i)
      {
        if (container_id.compare("bin_"+location_list_.bins[i].id) == 0)
        {
          for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
          {
            vec_container_items.push_back( location_list_.bins[i].content[j] );
          }
        }
      }
    }
    else if (container_id.substr(0, 3).compare("box") == 0)
    {
      for (int i = 0; i < location_list_.boxes.size(); ++i)
      {
        if (container_id.compare("box_"+location_list_.boxes[i].id) == 0)
        {
          for (int j = 0; j < location_list_.boxes[i].content.size(); ++j)
          {
            vec_container_items.push_back( location_list_.boxes[i].content[j] );
          }
        }
      }
    }
    else if( container_id.substr( 0, 4 ).compare( "tote" ) == 0 )
    {
      for( int i = 0; i < location_list_.tote.content.size(); ++i)
      {
        vec_container_items.push_back( location_list_.tote.content[i] );
      }
    }
  }

  return vec_container_items;
}

bool TaskManagerNode::GetCandidatesFromSVM(const float x, const float y, const float z, const float weight)
{
  vec_svm_items_ids_.clear();
  vec_svm_confs_.clear();

  tnp_svm::IdentifyItem srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.z = z;
  srv.request.weight = weight;

  if( svm_second_identify_items_client_.call( srv ) )
  {
    ROS_WARN_STREAM("[TNP_STATE L1] Executing /tnp_svm_second/identify_item Executing the service with values x,y,z,weight: " << x << ", " << y << ", " << z << ", " << weight );
    vec_svm_items_ids_ = convert_msgstring_to_string_vec(srv.response.items_ids);
    vec_svm_confs_ = convert_msgfloat_to_float_vec(srv.response.confidences);
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error /tnp_svm_second/identify_item There was an error calling the service");
    ROS_ERROR("Failed to call service /tnp_svm_second/identify_item");
    return false;
  }

  ROS_WARN("[TNP_STATE L1] Done /tnp_svm_second/identify_item Service called successfully");

  return true;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Deep learning
/*-------------------------------------------------------------------------------------*/
geometry_msgs::PoseStamped TaskManagerNode::Get3DPoseFrom2DCoord(float x, float y)
{
  // Use image_geometry package to transform from spatial coordinates to pixels.
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info_);

  const cv::Point2d point2d(x, y);
  cv::Point3d point3d = cam_model.projectPixelTo3dRay(point2d) * 0.70;

  geometry_msgs::PoseStamped pose_in_camera;
  pose_in_camera.header.stamp = ros::Time::now();
  pose_in_camera.header.frame_id = "tnp_ee_camera_frame";
  pose_in_camera.pose.position.x = point3d.x;
  pose_in_camera.pose.position.y = point3d.y;
  pose_in_camera.pose.position.z = point3d.z;
  pose_in_camera.pose.orientation.w = 1.0;

  return pose_in_camera;
}



inline bool orderDL(tnp_deep_vision::BoundingBox i,tnp_deep_vision::BoundingBox j)
{
  return (i.confidence > j.confidence);
}

bool TaskManagerNode::DLRecognizeItems(bool use_stow)
{
  // NOTE all poses are in camera frame
  ROS_WARN("[TNP_STATE L1] Calling tnp_deep_vision/recognize_items Start the recognition of the items");
  ROS_INFO("Recognizing items");
  tnp_deep_vision::recognize_items srv;

  vec_unreliable_dl_item_ids_.clear();
  vec_unreliable_dl_center_poses_.clear();

  if (dl_recognize_items_client_.call(srv))
  {
    ROS_WARN("[TNP_STATE L1] Executing tnp_deep_vision/recognize_items Executing the service");
    vector<tnp_deep_vision::BoundingBox> lotsOfBoundingBoxes = srv.response.boundingBoxes;
    if(srv.response.boundingBoxes.size() > 0)
    {
      // sorter by higher confidence
      std::sort (lotsOfBoundingBoxes.begin(), lotsOfBoundingBoxes.end(), orderDL );

      //clear the vector that save the DL results
      DLInitializeVectors();

      for( int num_of_items = 0; num_of_items < lotsOfBoundingBoxes.size(); ++num_of_items )
      {
        // Ignore those without valid coordinates (i.e., != 0 0 0)
        if( lotsOfBoundingBoxes[num_of_items].poseCenter.pose.position.x == 0 &&
            lotsOfBoundingBoxes[num_of_items].poseCenter.pose.position.y == 0 &&
            lotsOfBoundingBoxes[num_of_items].poseCenter.pose.position.z == 0 )
        {
          // Do not discard unreliable poses if stow
          if( use_stow )
          {
            vec_dl_item_ids_.push_back( "unknown_depth" );
            vec_dl_confidences_.push_back( 0.0 );
            //bounding boxes
            vec_dl_item_center_x_.push_back( 0.0 );
            vec_dl_item_center_y_.push_back( 0.0 );
            vec_dl_item_width_.push_back( 0.0 );
            vec_dl_item_height_.push_back( 0.0 );
            geometry_msgs::PoseStamped guessed_center_point = Get3DPoseFrom2DCoord( lotsOfBoundingBoxes[num_of_items].center_x, lotsOfBoundingBoxes[num_of_items].center_y );
            vec_dl_item_center_poses_in_cam_.push_back( guessed_center_point );
            
            // grasping points
            vec_dl_grasp_center_poses_.push_back( guessed_center_point ); //center of the grasp line
            geometry_msgs::PoseStamped guessed_graspLinePt1 = guessed_center_point;
            geometry_msgs::PoseStamped guessed_graspLinePt2 = guessed_center_point;
            guessed_graspLinePt1.pose.position.x += .1;   // Move one of the points to create a line (angle is unimportant)
            vec_dl_grasp_line_pt1_poses_.push_back( guessed_graspLinePt1 ); //first grasp line extremity using depth of the grasp point plane
            vec_dl_grasp_line_pt2_poses_.push_back( guessed_graspLinePt2 ); //second grasp line extremity using depth of the grasp point plane

          }
          continue;
        }

        // Valid dl poses
        //classification
        vec_dl_item_ids_.push_back( lotsOfBoundingBoxes[num_of_items].item_id);
        vec_dl_confidences_.push_back( lotsOfBoundingBoxes[num_of_items].confidence );
        //bounding boxes
        vec_dl_item_center_x_.push_back( lotsOfBoundingBoxes[num_of_items].center_x );
        vec_dl_item_center_y_.push_back( lotsOfBoundingBoxes[num_of_items].center_y );
        vec_dl_item_width_.push_back( lotsOfBoundingBoxes[num_of_items].width );
        vec_dl_item_height_.push_back( lotsOfBoundingBoxes[num_of_items].height );
        vec_dl_item_center_poses_in_cam_.push_back( lotsOfBoundingBoxes[num_of_items].poseCenter );
        // grasping points
        vec_dl_grasp_center_poses_.push_back(lotsOfBoundingBoxes[num_of_items].poseGrasp); //center of the grasp line
        vec_dl_grasp_line_pt1_poses_.push_back(lotsOfBoundingBoxes[num_of_items].graspLinePt1); //first grasp line extremity using depth of the grasp point plane
        vec_dl_grasp_line_pt2_poses_.push_back(lotsOfBoundingBoxes[num_of_items].graspLinePt2); //second grasp line extremity using depth of the grasp point plane

        // Doublecheck if the grasp pose is 0 for some reason, correct it if it is
        if( vec_dl_grasp_line_pt1_poses_[vec_dl_grasp_line_pt1_poses_.size()-1].pose.position.x == 0 &&
            vec_dl_grasp_line_pt1_poses_[vec_dl_grasp_line_pt1_poses_.size()-1].pose.position.y == 0 &&
            vec_dl_grasp_line_pt1_poses_[vec_dl_grasp_line_pt1_poses_.size()-1].pose.position.z == 0 )
        {
          ROS_WARN("Grasp pose is 0!! Adjusting to center of bbox.");
          geometry_msgs::PoseStamped guessed_center_point = Get3DPoseFrom2DCoord( lotsOfBoundingBoxes[num_of_items].center_x, lotsOfBoundingBoxes[num_of_items].center_y );
          geometry_msgs::PoseStamped guessed_graspLinePt1 = guessed_center_point;
          geometry_msgs::PoseStamped guessed_graspLinePt2 = guessed_center_point;
          guessed_graspLinePt1.pose.position.x += .1;   // Move one of the points to create a line (angle is unimportant)

          vec_dl_grasp_center_poses_[vec_dl_grasp_center_poses_.size()-1] = guessed_center_point; //center of the grasp line
          vec_dl_grasp_line_pt1_poses_[vec_dl_grasp_line_pt1_poses_.size()-1] = guessed_graspLinePt1; //first grasp line extremity using depth of the grasp point plane
          vec_dl_grasp_line_pt2_poses_[vec_dl_grasp_line_pt2_poses_.size()-1] = guessed_graspLinePt2; //second grasp line extremity using depth of the grasp point plane
        }
      }
    }
    else {/*we keep the latest results (better than nothing)*/}

    ROS_INFO("Items recognized by deep learning");
    for(size_t iDLitems = 0; iDLitems<vec_dl_item_ids_.size(); ++iDLitems)
    {
      ROS_INFO_STREAM("item: " << vec_dl_item_ids_[iDLitems] << " confidence " << vec_dl_confidences_[iDLitems]);
    }
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_vision/recognize_items There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_vision/DLRecognizeItemsService");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done tnp_vision/recognize_items Service called successfully");
  return true;
}

///Get the camera info
void TaskManagerNode::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  cam_info_ = msg;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Weight Events
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::WeightGetReadyToPick(const std::string& target_id, const std::string& target_container)
{
  bool result = true;
  ROS_INFO("Calling get_ready_for_pick service");
  tnp_weight_events::GetReadyForPick srv;
  ROS_WARN("[TNP_STATE L1] Calling tnp_weight_events/get_ready_for_pick Start calling the service");

  srv.request.target_id.data = target_id;
  srv.request.target_container.data = target_container;
  if (get_ready_for_pick_client_.call(srv))
  {
    ROS_WARN_STREAM("[TNP_STATE L1] Executing tnp_weight_events/get_ready_for_pick Executing the service with target id " << target_id);
    result = srv.response.ready;
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_weight_events/get_ready_for_pick There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_weight_events/get_ready_for_pick");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done tnp_vision/get_ready_for_pick Service called successfully");
  return result;
}

bool TaskManagerNode::WeightRecognizeItems(const std::string& target_id, const std::string& target_container)
{

  tnp_weight_events::RecognizeItems srv;
  ROS_WARN("[TNP_STATE L1] Calling tnp_weight_events/recognize_items Start recognizing weight service");

  srv.request.target_item_id.data = target_id;
  srv.request.target_container.data = target_container;
  if (weight_recognize_items_client_.call(srv))
  {
    ROS_WARN_STREAM("[TNP_STATE L1] Executing tnp_weight_events/recognize_items Executing the service, targed id" << target_id);
    vec_weight_items_ids_ = srv.response.items_ids;
    vec_weight_items_confs_ = srv.response.confidences;
    measured_weight_ = srv.response.weight;
  }
  else
  {
    ROS_WARN("[TNP_STATE L1] Error tnp_weight_events/recognize_items There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_weight_events/recognize_items");
    return false;
  }
  ROS_WARN("[TNP_STATE L1] Done tnp_weight_events/recognize_items Called successfully");
  return true;
}

bool TaskManagerNode::WeightSetItemsInfo()
{
  tnp_weight_events::SetItemsInfo srv;

  std::vector<std_msgs::String> items_ids;
  std::vector<std_msgs::Float64> items_weights;
  std::vector<std_msgs::String> bin_a_items;
  std::vector<std_msgs::String> bins_bc_items;
  std::vector<std_msgs::String> tote_items;
  std_msgs::String item_id_tmp;
  std_msgs::Float64 item_weight_tmp;

  for (int i = 0; i < database_.listItems.size(); ++i)
  {
    item_id_tmp.data = database_.listItems[i].item_id;
    items_ids.push_back(item_id_tmp);
    item_weight_tmp.data = database_.listItems[i].weight;
    items_weights.push_back(item_weight_tmp);
  }

  for (int i = 0; i < location_list_.bins.size(); ++i)
  {
    if (location_list_.bins[i].id == "A")
    {
      for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
      {
        item_id_tmp.data = location_list_.bins[i].content[j];
        bin_a_items.push_back(item_id_tmp);
      }
    }
    else if (location_list_.bins[i].id == "B" || location_list_.bins[i].id == "C")
    {
      for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
      {
        item_id_tmp.data = location_list_.bins[i].content[j];
        bins_bc_items.push_back(item_id_tmp);
      }
    }
  }

  for (int i = 0; i < location_list_.tote.content.size(); ++i)
  {
    item_id_tmp.data = location_list_.tote.content[i];
    tote_items.push_back(item_id_tmp);
  }

  ROS_INFO_STREAM("Sending " << items_ids.size() << " items");

  srv.request.items_ids = items_ids;
  srv.request.items_weights = items_weights;

  if (weight_set_items_info_client_.call(srv))
  {
    ROS_INFO_STREAM("Success, return value " << srv.response.total_items.data);
  }
  else
  {
    ROS_INFO("FAILED");
    return false;
  }

  return true;
}

bool TaskManagerNode::WeightSetItemsLocation()
{
  tnp_weight_events::SetItemsLocation srv;

  std::vector<std_msgs::String> bin_a_items;
  std::vector<std_msgs::String> bins_bc_items;
  std::vector<std_msgs::String> tote_items;
  std_msgs::String item_id_tmp;
  std_msgs::Float64 item_weight_tmp;

  for (int i = 0; i < location_list_.bins.size(); ++i)
  {
    if (location_list_.bins[i].id == "A")
    {
      for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
      {
        item_id_tmp.data = location_list_.bins[i].content[j];
        bin_a_items.push_back(item_id_tmp);
      }
    }
    else if (location_list_.bins[i].id == "B" || location_list_.bins[i].id == "C")
    {
      for (int j = 0; j < location_list_.bins[i].content.size(); ++j)
      {
        item_id_tmp.data = location_list_.bins[i].content[j];
        bins_bc_items.push_back(item_id_tmp);
      }
    }
  }

  for (int i = 0; i < location_list_.tote.content.size(); ++i)
  {
    item_id_tmp.data = location_list_.tote.content[i];
    tote_items.push_back(item_id_tmp);
  }

  ROS_INFO_STREAM("Sending " << bin_a_items.size() + bins_bc_items.size() + tote_items.size() << " items");

  srv.request.bin_a_items = bin_a_items;
  srv.request.bins_bc_items = bins_bc_items;
  srv.request.tote_items = tote_items;

  if (weight_set_items_location_client_.call(srv))
  {
    ROS_INFO_STREAM("Success, return value " << srv.response.total_items.data);
  }
  else
  {
    ROS_INFO("FAILED");
    return false;
  }

  return true;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Recognition Space
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::RecSpaceRecognizeItems(const std::string& target_item_id,
                                             std::vector<std::string> possible_item_ids)
{
  ROS_WARN_STREAM("[TNP_STATE L1] Start tnp_recognition_space/recognize_items start items recognition, target id"<< target_item_id);

  tnp_recognition_space::recognize_items srv;
  std_msgs::String target_bin_id_msg;
  target_bin_id_msg.data = target_item_id;
  vec_cloud_matching_items_ids_.clear();
  vec_cloud_matching_confs_.clear();
  vec_akaze_svm_items_ids_.clear();
  vec_akaze_svm_confs_.clear();
  vec_color_histogram_items_ids_.clear();
  vec_color_histogram_confs_.clear();

  srv.request.target_item_id = target_bin_id_msg;
  srv.request.possible_item_ids = convert_string_to_msgstring_vec( possible_item_ids );
  if( rec_space_recognize_items_client_.call( srv ) )
  {
    ROS_DEBUG( "[TNP_STATE L2] Done tnp_recognition_space/recognize_items Called successfully" );

    for(size_t iRes(0); iRes < srv.response.classifier_results.size(); ++iRes)
    {
      if(srv.response.classifier_results[iRes].classifierName.data.compare("Cloud matching") == 0)
      {
        // Cloud matching
        for(size_t iCloudRes(0); iCloudRes < srv.response.classifier_results[iRes].item_ids.size(); ++iCloudRes)
        {
          vec_cloud_matching_items_ids_.push_back(srv.response.classifier_results[iRes].item_ids[iCloudRes].data);
          vec_cloud_matching_confs_.push_back(    srv.response.classifier_results[iRes].confidences[iCloudRes].data);
        }
      }
      else if(srv.response.classifier_results[iRes].classifierName.data.compare("AKAZE SVM") == 0)
      {
        // AKAZE SVM
        for(size_t iAkazeRes(0); iAkazeRes < srv.response.classifier_results[iRes].item_ids.size(); ++iAkazeRes)
        {
          vec_akaze_svm_items_ids_.push_back(srv.response.classifier_results[iRes].item_ids[iAkazeRes].data);
          vec_akaze_svm_confs_.push_back(    srv.response.classifier_results[iRes].confidences[iAkazeRes].data);
        }
      }
      else if(srv.response.classifier_results[iRes].classifierName.data.compare("Color histogram") == 0)
      {
          // Color histogram
        for(size_t iColorRes(0); iColorRes < srv.response.classifier_results[iRes].item_ids.size(); ++iColorRes)
        {
          vec_color_histogram_items_ids_.push_back(srv.response.classifier_results[iRes].item_ids[iColorRes].data);
          vec_color_histogram_confs_.push_back(    srv.response.classifier_results[iRes].confidences[iColorRes].data);
        }
      }
      else
      {
        ROS_ERROR("TaskManagerNode::RecSpaceRecognizeItems: Unknown classifier name");
      }
    }
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_recognition_space/recognize_items There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_recognition_space/recognize_itemss");
    return false;
  }

  return true;
}

bool TaskManagerNode::RecSpaceRecordEmptyRS(const int& num_images)
{
  ROS_WARN_STREAM("[TNP_STATE L1] Start tnp_recognition_space/record_empty_rs num of images: " << num_images );

  tnp_recognition_space::record_empty_rs srv;
  std_msgs::Int16 msg;
  msg.data = num_images;
  srv.request.num_images = msg;
  if( rec_space_record_empty_rs_client_.call( srv ) )
  {
    ROS_DEBUG( "[TNP_STATE L2] Done tnp_recognition_space/record_empty_rs Called successfully" );
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_recognition_space/record_empty_rs There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_recognition_space/record_empty_rs");
    return false;
  }

  return true;
}

bool TaskManagerNode::RecSpaceGetBoundingBox()
{
  ROS_WARN_STREAM("[TNP_STATE L1] Start tnp_recognition_space/get_bounding_box");

  tnp_recognition_space::get_bounding_box srv;
  if( rec_space_get_bounding_box_client_.call( srv ) )
  {
    ROS_DEBUG( "[TNP_STATE L2] Done tnp_recognition_space/get_bounding_box Called successfully" );
    bounding_box_information_ = srv.response;
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_recognition_space/get_bounding_box There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_recognition_space/get_bounding_box");
    return false;
  }

  return true;
}

bool TaskManagerNode::RecSpaceSetItemsInfo()
{
  tnp_recognition_space::set_items_info srv;

  tnp_recognition_space::set_items_infoRequest items_info;

  for (int i = 0; i < database_.listItems.size(); ++i)
  {
    std_msgs::String tmp_item_id;
    tmp_item_id.data = database_.listItems[i].item_id;
    items_info.items_ids.push_back( tmp_item_id );

    items_info.known.push_back( database_.listItems[i].known );
  }

  srv.request = items_info;

  if (rec_space_set_items_info_client_.call(srv))
  {
    ROS_INFO_STREAM("Success, return value " << srv.response.num_items_received.data);
  }
  else
  {
    ROS_INFO("FAILED");
    return false;
  }

  return true;
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// End Effector
/*-------------------------------------------------------------------------------------*/
void TaskManagerNode::ItemIsSuctionedCallback(const std_msgs::Int16::ConstPtr &msg)
{
  if (msg->data == 1)
  {
    item_is_suctioned_ = true;
  }
  else
  {
    item_is_suctioned_ = false;
  }
}

void TaskManagerNode::ItemIsGraspedCallback(const std_msgs::Int16::ConstPtr &msg)
{
  if (msg->data == 1)
  {
    item_is_grasped_ = true;
  }
  else
  {
    item_is_grasped_ = false;
  }
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// State messages
/*-------------------------------------------------------------------------------------*/

  void TaskManagerNode::GenerateStatusMessage(std::string current_state, std::string next_state0, std::string next_state1, std::string next_state2, std::string next_state3)
{
  std::string state_machine_current_state;
  std::vector<std::string> state_machine_next_states;
  try
  {
     state_machine_current_state = current_state;
     state_machine_next_states.clear();
     state_machine_next_states.push_back(next_state0);
     state_machine_next_states.push_back(next_state1);
     state_machine_next_states.push_back(next_state2);
     state_machine_next_states.push_back(next_state3);
     publishStateInfo( state_machine_current_state, state_machine_next_states);
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("there was an error sending the status message");
  }

}


bool TaskManagerNode::GetSuctionCandidatesContainerFromOctomap(const std::string& target_container, const int& section_num)
{
  ROS_WARN("[TNP_STATE L1] Start tnp_grasp_planner/get_suction_candidates_container_from_octomap start getting suction candidates");

  tnp_grasp_planner::getSuctionCandidates_container_fromOctomap srv;
  srv.request.container_id.data = target_container;
  srv.request.section_num = section_num;

  if( get_suction_candidates_container_from_octomap_client_.call( srv ) )
  {
    ROS_WARN( "[TNP_STATE L2] Done tnp_grasp_planner/get_suction_candidates_container_from_octomap Called successfully" );

    ROS_DEBUG_STREAM( "Suction candidate poses from depth" );
    for( int i = 0; i < srv.response.suction_poses.size(); ++i )
    {
      ROS_DEBUG_STREAM( "Pose " << i + 1 << ": " << srv.response.suction_poses[i]);
    }

    if(srv.response.suction_poses.size() > 0)
    {
      vec_suction_candidate_poses_in_world_.clear();
      vec_suction_candidate_poses_in_camera_frame_.clear();
      for(size_t i=0; i<srv.response.suction_poses.size(); ++i)
      {
        // save the suckable surfaces in camera frame
        const geometry_msgs::PoseStamped pose_in_camera = transform_pose_now(srv.response.suction_poses[i], "tnp_ee_camera_frame" , tf_listener);
        vec_suction_candidate_poses_in_camera_frame_.push_back(pose_in_camera);
        // transform the suction candidates to world coordinates
        const geometry_msgs::PoseStamped pose_in_world = transform_pose_now(srv.response.suction_poses[i], "iiwa_link_0" , tf_listener);
        vec_suction_candidate_poses_in_world_.push_back(pose_in_world);
      }
      // sort them by the height in the real world (higher first)
      std::vector<std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> > combined_vectors;
      for(size_t i=0; i<vec_suction_candidate_poses_in_world_.size() && i<vec_suction_candidate_poses_in_camera_frame_.size(); ++i)
      {
        combined_vectors.push_back(std::make_pair(vec_suction_candidate_poses_in_world_[i], vec_suction_candidate_poses_in_camera_frame_[i]));
      }
      std::sort (combined_vectors.begin(), combined_vectors.end(), orderCandidatesCombinedVectors );
    }
    else
    {
      // we keep the last candidates (better than nothing)
    }

    ROS_DEBUG_STREAM( "Suction candidate poses in world coordinates (sorted from highest z-coord to lowest)" );
    for( int i = 0; i < vec_suction_candidate_poses_in_world_.size(); ++i )
    {
      ROS_DEBUG_STREAM( "Pose " << i + 1 << ": " << vec_suction_candidate_poses_in_world_[i] );
    }
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_grasp_planner/get_suction_candidates_container_from_octomap There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_grasp_planner/get_suction_candidates_container_from_octomap");
    return false;
  }

  return true;
}


bool TaskManagerNode::UpdateOccupancyMap(const std::string& target_container)
{
  ROS_WARN("[TNP_STATE L1] Start tnp_grasp_planner/update_occupancy_map start map updating");

  tnp_grasp_planner::updateOccupancyMap srv;
  srv.request.container_id.data = target_container;

  ROS_INFO_STREAM( "Calling update occupancy map for container: " << srv.request.container_id.data );

  if( update_occupancy_map_client_.call( srv ) )
  {
    ROS_WARN( "[TNP_STATE L2] Done tnp_grasp_planner/update_occupancy_map Called successfully" );
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_grasp_planner/update_occupancy_map There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_grasp_planner/update_occupancy_map");
    return false;
  }

  return true;
}

bool TaskManagerNode::GetPoseToPlaceItem(const std::string& target_container, const geometry_msgs::PoseStamped& bounding_box_center_pose,
                                         const float& item_width, const float& item_length, const float& item_height)
{
  ROS_WARN("[TNP_STATE L1] Start tnp_grasp_planner/getPoseToPlaceItem start container recognition");

  tnp_grasp_planner::getPoseToPlaceItem srv;
  srv.request.bounding_box_center_pose = bounding_box_center_pose;
  srv.request.item_width = item_width;
  srv.request.item_length = item_length;
  srv.request.item_height = item_height;
  srv.request.container_id.data = target_container;

  if ((target_container.compare("bin_A_1") == 0))
  {
    srv.request.container_id.data = "bin_A";
    srv.request.section_num = 1;
  }
  else if (target_container.compare("bin_A_2") == 0)
  {
    srv.request.container_id.data = "bin_A";
    srv.request.section_num = 2;
  }

  if( get_pose_to_place_item_client_.call( srv ) )
  {
    ROS_WARN( "[TNP_STATE L2] Done tnp_grasp_planner/getPoseToPlaceItem Called successfully" );
    free_spot_found_ = srv.response.free_spot_found;
    geometry_msgs::PoseStamped pose_to_place_item_in_container_coords = srv.response.pose_to_place_item_in_container_coords;
    ROS_INFO_STREAM("Pose to place item (in container coords): " << pose_to_place_item_in_container_coords);

    pose_to_place_item_in_world_coords_ = transform_pose_now(pose_to_place_item_in_container_coords, "iiwa_link_0" , tf_listener);
    ROS_INFO_STREAM("Pose to place item (in world coords): " << pose_to_place_item_in_world_coords_);
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error tnp_grasp_planner/getPoseToPlaceItem There was an error calling the service");
    ROS_ERROR("Failed to call service tnp_grasp_planner/update_occupancy_map");
    free_spot_found_ = false;
    return false;
  }
  return true;
}


/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// LED control
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::SetLedIntensity(const int& led_num, const int& intensity)
{
  if(led_num >= 1 && led_num <= 8 && intensity >= 0 && intensity <=255)
  {
    tnp_led_control::set_led_intensity srv;
    srv.request.target_led_num.data = led_num;
    srv.request.target_intensity.data = intensity;

    if (set_led_intensity_client_.call(srv))
    {
      ROS_INFO("SetLedIntensityService is being called");
    }
    else
    {
      ROS_ERROR("Failed to call service ten_led_control/set_led_intensity");
      ROS_WARN( "[TNP_STATE L2] Error tnp_led_control/set_led_intensity There was an error when calling the service" );
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("SetLedIntensity invalid values: led_num(1-8) is " << led_num << ", intensity (0-255) is " << intensity);
    return false;
  }
  return true;
}

void TaskManagerNode::RestoreLedInitialState()
{
  for( int i = 0; i < 8; ++i )
  {
    SetLedIntensity( i+1, led_intensities_[i] );
  }
}

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Visualization (tnp_monitor)
/*-------------------------------------------------------------------------------------*/
void TaskManagerNode::publishPoseMarker(const geometry_msgs::Pose& pose, const std::string& name)
{
  tnp_msgs::ShowPose srv;
  srv.request.pose = pose;
  srv.request.name = name;
  show_pose_client_.call(srv);
}

/*-------------------------------------------------------------------------------------*/
/// Service definitions
/// testMotions
/*-------------------------------------------------------------------------------------*/
bool TaskManagerNode::TestMotionsCallback(tnp_task_manager::testMotions::Request &req,
                                          tnp_task_manager::testMotions::Response &res)
{
  ROS_INFO("TestMotionsCallback was called");
  ROS_INFO("Executing motion Nr. %d", req.motion_id);
  srand (time(NULL)); /* initialize random seed: */

  // simple test motion
  if (req.motion_id == 1)
  {
    ROS_INFO("Motion 1: move kuka to home and to the tote to test commands can be sent");
    GoToHome();
    ros::Duration(1.0).sleep();
    GoToTote();
    ROS_INFO("Motion 1 was successful");
  }
  // suck item test
  else if (req.motion_id == 2)
  {
    ROS_INFO("Motion 2 has not been defined");
    return false;
  }
  // grasping test on tote, get random items from the DL list
  else if (req.motion_id == 3)
  {
    GoToTote();
    GoToLookIntoTote(0);
    ros::Duration(2.0).sleep();
    DLRecognizeItems();
    if(vec_dl_grasp_center_poses_.size() > 0)
    {
      const int rand_elem = rand() % vec_dl_item_ids_.size();
      GraspItem(vec_dl_item_ids_[rand_elem], vec_dl_grasp_center_poses_[rand_elem],
                vec_dl_grasp_line_pt1_poses_[rand_elem], vec_dl_grasp_line_pt2_poses_[rand_elem], 100);
    }
    else
    {
      ROS_ERROR("No grasping points candidates");
    }
  }
  // grasping test on tote, get all items from the DL list
  else if (req.motion_id == 4)
  {
    GoToTote();
    GoToLookIntoTote(0);
    ros::Duration(2.0).sleep();
    DLRecognizeItems();
    if(vec_dl_grasp_center_poses_.size() > 0)
    {
      for(size_t i=0; i<vec_dl_grasp_center_poses_.size(); ++i)
      {
        GraspItem(vec_dl_item_ids_[i], vec_dl_grasp_center_poses_[i],
                  vec_dl_grasp_line_pt1_poses_[i], vec_dl_grasp_line_pt2_poses_[i], 100);
        GoToTote();
        GoToLookIntoTote(0);
      }
    }
    else
    {
      ROS_ERROR("No grasping points candidates");
    }
  }
  // grasping test on tote, get all items from the DL list
  else if (req.motion_id == 5)
  {
    GoToTote();
    GoToLookIntoTote(0);
    ros::Duration(2.0).sleep();
    DLRecognizeItems();
    if(vec_dl_grasp_center_poses_.size() > 0)
    {
      for(size_t i=0; i<vec_dl_grasp_center_poses_.size(); ++i)
      {
        GraspItem(vec_dl_item_ids_[i], vec_dl_grasp_center_poses_[i],
                  vec_dl_grasp_line_pt1_poses_[i], vec_dl_grasp_line_pt2_poses_[i], 100);
        GoToTote();
        GoToLookIntoTote(0);
      }
    }
    else
    {
      ROS_ERROR("No grasping points candidates");
    }
  }
  // tag the depth info with DL info
  else if(req.motion_id == 6)
  {
    UpdateOccupancyMap("tote");
    GetSuctionCandidatesContainerFromOctomap("tote", 0/*section*/);
    ROS_INFO_STREAM("num of suckable surfaces received (1): " << vec_suction_candidate_poses_in_world_.size());

    DLRecognizeItems();

    ROS_INFO_STREAM("num of suckable surfaces received (2): " << vec_suction_candidate_poses_in_world_.size());
    ROS_INFO_STREAM("num of items recognized: " << vec_dl_item_ids_.size());

    if(vec_dl_item_ids_.size() > 0 && vec_suction_candidate_poses_in_world_.size() > 0)
    {
      ROS_INFO_STREAM("Tag the suction candidates");
      GetTagsForSuctionCandidatesUsingDLInfo();
    }
    else
    {
      ROS_ERROR("No grasping points candidates or no DL recognized items received");
    }

    ROS_INFO_STREAM("Highest point is " << vec_suction_candidate_poses_in_world_[0].pose.position);
  }
  else if(req.motion_id == 7)
  {
    while( location_list_.tote.content.size() > 0 )
    {
      std::string item_tmp = location_list_.tote.content[0];
      UpdateItemLocationInMemory( item_tmp, "tote", "bin_A" );
    }

    const std::string filepath("/root/share/output_stow/item_location_file_output.json");
    std::ofstream ofs_stow_output(filepath.c_str());
    if(ofs_stow_output)
    {
      ofs_stow_output << location_list_.toString();
      ofs_stow_output.close();
    }
    else
    {
      ROS_ERROR_STREAM("File error: " << filepath);
    }
  }
  else if(req.motion_id == 8)
  {
    geometry_msgs::PoseStamped bounding_box_center_pose;
    bounding_box_center_pose.pose.orientation.w = 1.0;
    GetPoseToPlaceItem("bin_A", bounding_box_center_pose, 0.15, 0.15, 0.15);
  }
  else if(req.motion_id == 9)
  {
    float svm_tmp = 0.000463884;

    ROS_INFO_STREAM( "SVM_VECTOR_ITEMS: " );
    for( int i = 0; i < vec_svm_items_ids_.size(); ++i )
    {
      ROS_INFO_STREAM( vec_svm_items_ids_[i] );
      ROS_INFO_STREAM( vec_svm_confs_[i] );
    }
  }
  else if( req.motion_id == 10 )
  {
    ROS_INFO_STREAM( "TESTING HORIZONTAL SWEEP" );
    GoToContainer( "tote" );
    SweepToteHorizontal(0.14);

  }
  else if( req.motion_id == 11 )
  {
    ROS_INFO_STREAM( "TESTING GRASSPING ITEM" );
    GoToContainer( "tote" );
    GoToLookIntoContainer( "tote", 1 );
    ros::Duration(4.0).sleep();
    DLRecognizeItems();

    geometry_msgs::PoseStamped pose_in_world = transform_pose_now(vec_dl_item_center_poses_in_cam_[0], "/iiwa_link_0", tf_listener);
    geometry_msgs::PoseStamped grasp_point_1 = transform_pose_now(vec_dl_grasp_line_pt1_poses_[0], "/iiwa_link_0", tf_listener);
    geometry_msgs::PoseStamped grasp_point_2 = transform_pose_now(vec_dl_grasp_line_pt2_poses_[0], "/iiwa_link_0", tf_listener);

    GoToContainer( "tote" );

    RetrieveItem( 1, vec_dl_item_ids_[0], pose_in_world,
                  grasp_point_1,
                  grasp_point_2,
                   100, false);

    GoToContainer( "tote" );
  }
  else // unknown motion_id
  {
    ROS_INFO("Motion number %d hasn't been defined", req.motion_id);
  }

  return true;
}


void PrintPossibleItemsVec(const std::vector<std::string>& possible_items_ids,
                           const std::vector<float>& possible_items_confidences,
                           const std::string& voter)
{
  try{
    ROS_INFO_STREAM("**************** Adding "<< voter << " vote added*****************");
    std::vector<std::pair<std::string, float> > combined_vectors;
    for(size_t i=0; i<possible_items_ids.size() && i<possible_items_confidences.size(); ++i)
    {
      combined_vectors.push_back(std::make_pair(possible_items_ids[i], possible_items_confidences[i]));
    }
    std::sort (combined_vectors.begin(), combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

    for(size_t i(0); i < combined_vectors.size(); ++i)
    {
      stringstream formatted_stream;
      formatted_stream << fixed << setprecision(2) << round(combined_vectors[i].second*100.0)/100.0;
      ROS_INFO_STREAM("conf: " << formatted_stream.str()<< " id: " << combined_vectors[i].first);
    }
    ROS_INFO_STREAM("****************************************************************");
  } catch(...){ ROS_WARN_STREAM("Error PrintPossibleItemsVec"); }
}

bool TaskManagerNode::decide_item_identity_from_all_votes(std::string& belief_item_id, float& belief_confidence, std::string dl_target_item, float dl_item_conf, bool ignore_DL)
{
  // list of possible items after discrimination using deep learning, weight and svm
  std::vector<std::string> possible_items_ids;
  std::vector<float> possible_items_confidences;

  ROS_INFO_STREAM("dl_vote_share_ "<<dl_vote_share_);
  ROS_INFO_STREAM("weight_vote_share_ "<<weight_vote_share_);
  ROS_INFO_STREAM("svm_vote_share_ "<<svm_vote_share_);
  ROS_INFO_STREAM("cloud_matching_vote_share_ "<<cloud_matching_vote_share_);
  ROS_INFO_STREAM("akaze_svm_vote_share_ "<<akaze_svm_vote_share_);
  ROS_INFO_STREAM("color_histogram_vote_share_ "<<color_histogram_vote_share_);

  //// Add all votes together geometrically sum of (vote * voter_weight)
  if( !ignore_DL ) // When no known item could have been taken from this container
  {
    ROS_INFO("Taking DL result into account, as it may be a known item");
    const float dl_vote = dl_item_conf * dl_vote_share_;
    possible_items_ids.push_back(dl_target_item);
    possible_items_confidences.push_back(dl_vote);
  }
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "DL");

  addVoterToCumulatedVector_GeometricAddition( vec_weight_items_ids_, vec_weight_items_confs_, weight_vote_share_, possible_items_ids, possible_items_confidences );
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "WEIGHT");
  addVoterToCumulatedVector_GeometricAddition( vec_svm_items_ids_, vec_svm_confs_, svm_vote_share_, possible_items_ids, possible_items_confidences );
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "SVM");
  addVoterToCumulatedVector_GeometricAddition( vec_akaze_svm_items_ids_, vec_akaze_svm_confs_, akaze_svm_vote_share_, possible_items_ids, possible_items_confidences );
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "CLOUD_MATCHING");
  addVoterToCumulatedVector_GeometricAddition( vec_cloud_matching_items_ids_, vec_cloud_matching_confs_, cloud_matching_vote_share_, possible_items_ids, possible_items_confidences );
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "AKAZE SVM");
  addVoterToCumulatedVector_GeometricAddition( vec_color_histogram_items_ids_, vec_color_histogram_confs_, color_histogram_vote_share_, possible_items_ids, possible_items_confidences );
  /*debug*/PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "COLOR HISTOGRAM");
  
  //// Remove items that were not in the box
  // If the possible items vector contains items the system will determine if that item
  // was in the order and try to put it in its corresponding box
  if (possible_items_ids.size() > 0)
  {
    // Determine item with biggest confidence among the possible items, i.e., our belief
    for (int i = 0; i < possible_items_ids.size(); ++i)
    {
      if (possible_items_confidences[i] > belief_confidence)
      {
        belief_item_id = possible_items_ids[i];
        belief_confidence = possible_items_confidences[i];
      }
    }
    GenerateStatusMessage("VoteForItem", "DetermineBox", "PutItemIntoBin", " "," ");

    ROS_INFO("----------------------------------");
    ROS_WARN_STREAM("Final determined item: " << belief_item_id);
    ROS_WARN_STREAM("Confidence: " << belief_confidence);
    ROS_INFO("---------------------------------------");

    stringstream formatted_stream;
    formatted_stream << fixed << setprecision(2) << round(belief_confidence*100.0)/100.0;
    //PublishTaskMessages( "----------------------------------", true );
    PublishTaskMessages( " ******* Final determined item: ******* ", true );
    PublishTaskMessages( "   "+belief_item_id+" | "+formatted_stream.str(), true );

    if (debug_on_) getch();

    ROS_WARN_STREAM("[TNP_STATE L0] Executing tnp_task_manager/vote_items Final item determined " << belief_item_id);
    return true;
  }
  else
  {
    return false;
  }
}

bool TaskManagerNode::decide_item_identity_from_all_votes_stow(std::vector<std::string>& possible_items_ids, std::vector<float>& possible_items_confidences, 
  std::string dl_target_item, float dl_item_conf )
{
  ROS_INFO_STREAM("dl_vote_share_ "<<dl_vote_share_);
  ROS_INFO_STREAM("weight_vote_share_ "<<weight_vote_share_);
  ROS_INFO_STREAM("svm_vote_share_ "<<svm_vote_share_);
  ROS_INFO_STREAM("cloud_matching_vote_share_ "<<cloud_matching_vote_share_);
  ROS_INFO_STREAM("akaze_svm_vote_share_ "<<akaze_svm_vote_share_);
  ROS_INFO_STREAM("color_histogram_vote_share_ "<<color_histogram_vote_share_);


  //// Add all votes together geometrically sum of (vote * voter_weight)
  if( dl_item_conf > 0.7 ) /// If conf is higher than .7 we add dl to our decision
  {
    ROS_INFO("Taking DL result into account, as it may be a known item");
    const float dl_vote = dl_item_conf * dl_vote_share_;
    possible_items_ids.push_back(dl_target_item);
    possible_items_confidences.push_back(dl_vote);
  }
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "DL");

  addVoterToCumulatedVector_GeometricAddition( vec_weight_items_ids_, vec_weight_items_confs_, weight_vote_share_, possible_items_ids, possible_items_confidences );
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "WEIGHT");
  addVoterToCumulatedVector_GeometricAddition( vec_svm_items_ids_, vec_svm_confs_, svm_vote_share_, possible_items_ids, possible_items_confidences );
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "SVM");
  addVoterToCumulatedVector_GeometricAddition( vec_akaze_svm_items_ids_, vec_akaze_svm_confs_, akaze_svm_vote_share_, possible_items_ids, possible_items_confidences );
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "CLOUD_MATCHING");
  addVoterToCumulatedVector_GeometricAddition( vec_cloud_matching_items_ids_, vec_cloud_matching_confs_, cloud_matching_vote_share_, possible_items_ids, possible_items_confidences );
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "AKAZE SVM");
  addVoterToCumulatedVector_GeometricAddition( vec_color_histogram_items_ids_, vec_color_histogram_confs_, color_histogram_vote_share_, possible_items_ids, possible_items_confidences );
  PrintPossibleItemsVec(possible_items_ids, possible_items_confidences, "COLOR HISTOGRAM");

  return true;
}

int TaskManagerNode::GetBoxIDOfItem(const std::string &item_id)
{
  int order_id, box_id;
  ItemInOrder(item_id, order_id);    // Look up order ID
  GetBoxInfo(order_id, box_id);
  return box_id;
}

// The item is being held when we call this function.
bool TaskManagerNode::DeliverHeldItemToBox(std::string belief_item_id, bool use_gripper_EE)
{
  bool item_delivered = false;
  int box_id = GetBoxIDOfItem(belief_item_id);
  GenerateStatusMessage("PutItemIntoBox", "GoToHome", "Finish", " "," ");
  GoToContainer("box_"+std::to_string(box_id));

  geometry_msgs::PoseStamped bounding_box_center_pose;
  ROS_WARN_STREAM("======================================");
  ROS_WARN_STREAM("Bounding box from rec space \n" << bounding_box_information_);
  ROS_WARN_STREAM("======================================");
  if(debug_on_) getch();
  bounding_box_center_pose.pose.orientation.w = 1.0;

  GetPoseToPlaceItem("box_"+std::to_string(box_id), bounding_box_center_pose, 
    bounding_box_information_.bb_width.data, 
    bounding_box_information_.bb_height.data, 
    bounding_box_information_.bb_length.data); 
  ROS_WARN_STREAM("======================================");
  ROS_WARN_STREAM("Pose to place the item \n" << pose_to_place_item_in_world_coords_);
  ROS_WARN_STREAM("======================================");
  if(debug_on_) getch();

  if(free_spot_found_) {/*no need to update map*/ ROS_WARN("Free spot found");}
  else {
    ROS_WARN_STREAM("Task manager: free spot not found");
  }

  /*debug*/if(debug_on_) getch();

  if( PutItemIntoContainer( "box_"+std::to_string(box_id), use_gripper_EE, pose_to_place_item_in_world_coords_) )
  {
    item_delivered = true;

    if(!free_spot_found_)
    {
      ROS_WARN_STREAM("Task manager: free spot not found");
      GoToLookIntoContainer("box_"+std::to_string(box_id), 1/*height (1,2,3)*/);
      // update the occupancy map
      OpenEEShutterCloseTheRest(); // close the other shutters and open the EE
      ros::Duration(4.0).sleep(); ///NOTE the robot to be stopped to properly get depth info
      if(debug_on_) getch();
      UpdateOccupancyMap("box_"+std::to_string(box_id));
    } else{/* no need to update*/}

    GoToContainer("box_"+std::to_string(box_id));
  }
  ///// We will never fail to let go of an item above a box, so there is no recovery
  else
  {
    item_delivered = false;
    ROS_ERROR("System was unable to deliver the item, SOMETHING HAS GONE TERRIBLY WRONG");
    ROS_WARN("[TNP_STATE L0] Start /tnp_task_manager System was unable to deliver the item, returning to origin bin");
  }
  return item_delivered;
}

bool TaskManagerNode::SelectCandidateFromDL(int& dl_item_idx, std::string bin_id, int looking_for_known_item, std::vector<geometry_msgs::PoseStamped> vec_poses_blacklist, bool filter_for_sections, bool section_is_A1)
{
  bool valid_candidate_found = false;
  for(dl_item_idx = 0; dl_item_idx < vec_dl_item_ids_.size(); ++dl_item_idx )
  {
    // Condition to determine if the current item from the DL service exists in an order
    int dummy = -1;
    bool item_in_order = ItemInOrder( vec_dl_item_ids_[dl_item_idx], dummy );

    // Check that the headers are fine
    if (( vec_dl_item_center_poses_in_cam_[dl_item_idx].header.frame_id.compare( "" ) == 0 )
     || ( vec_dl_grasp_line_pt1_poses_[dl_item_idx].header.frame_id.compare( "" ) == 0 )
     || ( vec_dl_grasp_line_pt2_poses_[dl_item_idx].header.frame_id.compare( "" ) == 0 ))
    {
      ROS_INFO("Skipped an item because a header was empty");
      continue;
    }

    if( ( looking_for_known_item == 0 && item_in_order ) || ( looking_for_known_item == 1 ) )
    {
      // Do not return a known item with high confidence if we look for an unknown one
      if( ( looking_for_known_item == 1) && (vec_dl_confidences_[dl_item_idx] > .9) )
      {
        ROS_INFO("Skipped an item because it is known, but we are looking for an unknown item.");
        continue;
      }

      // Condition to determine if the current item exists in the current bin
      bool item_in_bin = ItemInBin( bin_id, vec_dl_item_ids_[dl_item_idx] );
      if( ( looking_for_known_item == 0 && item_in_bin ) || ( looking_for_known_item == 1 ) )
      {
        valid_candidate_found = true;

        geometry_msgs::PoseStamped candidate_pose = transform_pose_now(vec_dl_item_center_poses_in_cam_[dl_item_idx], "/iiwa_link_0", tf_listener);
        // Check if item's pose is far enough from previously failed attempts
        if( vec_poses_blacklist.size() > 0 )
        {
          bool pose_matched_blacklist_element = false;
          for(int i_blacklisted_pose=0; i_blacklisted_pose < vec_poses_blacklist.size(); ++i_blacklisted_pose)
          {
            ROS_ERROR_STREAM("Candidate pose" << candidate_pose.pose.position);
            ROS_ERROR_STREAM("Blacklisted pose " << vec_poses_blacklist[i_blacklisted_pose].pose.position);
            const float dx = candidate_pose.pose.position.x - vec_poses_blacklist[i_blacklisted_pose].pose.position.x;
            const float dy = candidate_pose.pose.position.y - vec_poses_blacklist[i_blacklisted_pose].pose.position.y;
            const float square_distance_xy = dx*dx + dy*dy;
            const float distance_xy = sqrt(square_distance_xy);
            if(distance_xy < blacklist_distance_stow_) // candidate is far from blacklisted ///TODO this should be a ROS parameter
            {
              pose_matched_blacklist_element = true;
              break;
            }
          }

          if( pose_matched_blacklist_element )
          {
            valid_candidate_found = false;
          }
        }

        // Check if item position is in the correct half of the bin
        if (filter_for_sections)
        {
          if ( ((section_is_A1)  && (candidate_pose.pose.position.y >= 0.0))
            || ((!section_is_A1) && (candidate_pose.pose.position.y <  0.0)) )
          {
            ROS_INFO_STREAM("A candidate item was filtered because it not in section " << (section_is_A1 ? "A1" : "A2"));
            valid_candidate_found = false;
          }
        }

        if ( valid_candidate_found )
        {
          break;
        }
      }
    }
  }
  return valid_candidate_found;
}

bool TaskManagerNode::RecognizeItemsInContainer(std::string container)
{
  bool has_DLnode_recognized_items = false;
  GenerateStatusMessage("GoToLookIntoContainer", "DLRecognizeItems", " ", " "," ");
  
  GoToLookIntoContainer(container, 0 /*height_position_num*/);
  ros::Duration(3.0).sleep(); ///TODO is this sleep necessary or can it be reduced?

  // DLRecognizeItems service is called to recognize items in the point of view of the
  // current section of the current bin
  GenerateStatusMessage("DLRecognizeItems", "ItemInOrder", "GoToLookIntoContainer", " "," ");
  if (DLRecognizeItems()) //service succeeded
  {
    if (vec_dl_item_ids_.size() > 0) // recognized something and will skip remaining points of view
    {
      ROS_INFO_STREAM(vec_dl_item_ids_.size() << " items were recognized in " << container);
      has_DLnode_recognized_items = true;
      ROS_WARN_STREAM("[TNP_STATE L0] Executing tnp_deep_vision/recognize_items Items recognized in " << container);
    }
    else // No items were recognized
    {
      ROS_INFO_STREAM("No items were recognized in " << container);
      has_DLnode_recognized_items = false;
      ROS_WARN_STREAM("[TNP_STATE L0] Executing tnp_deep_vision/recognize_items No items were recognized in " << container);
    }
  }
  else // DL failed
  {
    ROS_ERROR( "Failed to call service DLRecognizeItems" );
    has_DLnode_recognized_items = false;
  }
  return has_DLnode_recognized_items;
}

void TaskManagerNode::DumpDecisionDataToTnpMonitor(
    std::vector<std::pair<std::string, float>> weight_combined_vectors, 
    std::vector<std::pair<std::string, float>> svm_combined_vectors, 
    std::vector<std::string> vec_cloud_matching_items_ids, 
    std::vector<float> vec_cloud_matching_confs,
    std::vector<std::pair<std::string, float>> akaze_svm_combined_vectors, 
    std::vector<std::pair<std::string, float>> color_histogram_combined_vectors,
    bool display_the_weight)
{
  if(svm_combined_vectors.size()>0)
  {
    stringstream formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream6;
    formatted_stream1 << fixed << setprecision(2) << round(svm_combined_vectors[0].second*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(svm_combined_vectors[1].second*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(svm_combined_vectors[2].second*100.0)/100.0;
    formatted_stream6 << fixed << setprecision(2) << round(svm_vote_share_*100.0)/100.0;
    PublishTaskMessages( "SVM Weight/Volume (" + formatted_stream6.str() + "): ", true );
    PublishTaskMessages( "   "+svm_combined_vectors[0].first+" | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+svm_combined_vectors[1].first+" | "+formatted_stream2.str(), true );
    PublishTaskMessages( "   "+svm_combined_vectors[2].first+" | "+formatted_stream3.str(), true );
    if(svm_combined_vectors.size()>4)
    {
      stringstream formatted_stream4, formatted_stream5;
      formatted_stream4 << fixed << setprecision(2) << round(svm_combined_vectors[3].second*100.0)/100.0;
      formatted_stream5 << fixed << setprecision(2) << round(svm_combined_vectors[4].second*100.0)/100.0;
      PublishTaskMessages( "   "+svm_combined_vectors[3].first+" | "+formatted_stream4.str(), true );
      PublishTaskMessages( "   "+svm_combined_vectors[4].first+" | "+formatted_stream5.str(), true );
    }
  }
  if(vec_cloud_matching_items_ids.size()>0)
  {
    stringstream formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream6;
    formatted_stream1 << fixed << setprecision(2) << round(vec_cloud_matching_confs[0]*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(vec_cloud_matching_confs[1]*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(vec_cloud_matching_confs[2]*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(cloud_matching_vote_share_*100.0)/100.0;
    PublishTaskMessages( "Cloud Matching (" + formatted_stream6.str() + "): ", true );
    PublishTaskMessages( "   "+vec_cloud_matching_items_ids[0]+ " | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+vec_cloud_matching_items_ids[1]+ " | "+formatted_stream2.str(), true );
    if(vec_cloud_matching_confs.size()>4)
    {
      stringstream formatted_stream4, formatted_stream5;
      formatted_stream4 << fixed << setprecision(2) << round(vec_cloud_matching_confs[3]*100.0)/100.0;
      formatted_stream5 << fixed << setprecision(2) << round(vec_cloud_matching_confs[4]*100.0)/100.0;
      PublishTaskMessages( "   "+vec_cloud_matching_items_ids[3]+" | "+formatted_stream4.str(), true );
      PublishTaskMessages( "   "+vec_cloud_matching_items_ids[4]+" | "+formatted_stream5.str(), true );
    }
  }
  if(akaze_svm_combined_vectors.size()>0)
  {
    stringstream formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream6;
    formatted_stream1 << fixed << setprecision(2) << round(akaze_svm_combined_vectors[0].second*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(akaze_svm_combined_vectors[1].second*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(akaze_svm_combined_vectors[2].second*100.0)/100.0;
    formatted_stream6 << fixed << setprecision(2) << round(akaze_svm_vote_share_*100.0)/100.0;
    PublishTaskMessages( "SVM Akaze (" + formatted_stream6.str() + "): " , true );
    PublishTaskMessages( "   "+akaze_svm_combined_vectors[0].first+" | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+akaze_svm_combined_vectors[1].first+" | "+formatted_stream2.str(), true );
    PublishTaskMessages( "   "+akaze_svm_combined_vectors[2].first+" | "+formatted_stream3.str(), true );
  }
  if(color_histogram_combined_vectors.size()>0)
  {
    stringstream formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream6;
    formatted_stream1 << fixed << setprecision(2) << round(color_histogram_combined_vectors[0].second*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(color_histogram_combined_vectors[1].second*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(color_histogram_combined_vectors[2].second*100.0)/100.0;
    formatted_stream6 << fixed << setprecision(2) << round(color_histogram_vote_share_*100.0)/100.0;
    PublishTaskMessages( "Color Histogram (" + formatted_stream6.str() + "): " , true );
    PublishTaskMessages( "   "+color_histogram_combined_vectors[0].first+" | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+color_histogram_combined_vectors[1].first+" | "+formatted_stream2.str(), true );
    PublishTaskMessages( "   "+color_histogram_combined_vectors[2].first+" | "+formatted_stream3.str(), true );
    if(color_histogram_combined_vectors.size()>4)
    {
      stringstream formatted_stream4, formatted_stream5;
      formatted_stream4 << fixed << setprecision(2) << round(color_histogram_combined_vectors[3].second*100.0)/100.0;
      formatted_stream5 << fixed << setprecision(2) << round(color_histogram_combined_vectors[4].second*100.0)/100.0;
      PublishTaskMessages( "   "+color_histogram_combined_vectors[3].first+" | "+formatted_stream4.str(), true );
      PublishTaskMessages( "   "+color_histogram_combined_vectors[4].first+" | "+formatted_stream5.str(), true );
    }
  }
}

// Returns true if it found an item
bool TaskManagerNode::SearchingPhaseAllBins(int& looking_for_known_item, 
  std::string& dl_target_item, float& dl_item_conf, 
  geometry_msgs::PoseStamped& dl_item_pose_in_world, 
  geometry_msgs::PoseStamped& dl_grasp_line_pt1_pose_in_world, 
  geometry_msgs::PoseStamped& dl_grasp_line_pt2_pose_in_world, 
  std::string& bin_id, std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist)
{
  ROS_ERROR("=============== SEARCHING PHASE ==============="); 
  ROS_INFO_STREAM("Starting to look in all bins for " << (looking_for_known_item == 0 ? "known" : "unknown") << "items.");
  // Number of sections per bin 
  std::string bin_id_section = ""; 
  bool got_item_candidate = false;
  bool has_DLnode_recognized_items = false; 

  RestoreLedInitialState(); //set our leds' intensity

  // For loop included to move kuka between the 3 bins, if an item was found it will stop and
  // restart again next cycle
  for (looking_for_known_item = 0; looking_for_known_item < 2; ++looking_for_known_item)
  {
    // Condition created to skip a bin if there are no items from the orders in that specific bin

    // This for is defined for trying to pick up all the known items first
    // and if there are not anymore known items it will try to pick the
    // unknown or untrained ones
    for (int bin_num = 0; bin_num < 3; ++bin_num)
    {
      //Determine bin id according to the bin_num 0=A 1=B 2=C
      bin_id = bin_num == 0 ? "A" : bin_num == 1 ? "B" : "C";

      // looking_for_known_item is the flag created to determine if we are searching for known or unknown items
      // 0 = Known, 1 = Unknown
      std::string type_of_item_searched = (looking_for_known_item == 0 ? "known" : "unknown");
      if (looking_for_known_item == 0)
      {
        if (!OrderItemsInBin(bin_id, type_of_item_searched))
        {
          ROS_INFO_STREAM("Bin " << bin_id << " skipped because there are no " << type_of_item_searched << "needed items inside");
          continue;
        }
        else
        {
          ROS_INFO_STREAM("Getting " << type_of_item_searched << " items in bin " << bin_id);
        }
      }

      GoToContainer("bin_" + bin_id);
      const int num_sections_bin = bin_id.compare( "A" ) ? 1 : 2;

      // This cycle is created to go through different sections in a bin
      // at the moment there are no sections defined this will stop when an item is found
      for (int i_bin_section = 1; i_bin_section <= num_sections_bin; ++i_bin_section)
      {
        GenerateStatusMessage("GoToContainer", "GoToLookIntoContainer", " ", " "," ");
        bin_id_section = bin_id.compare( "A" ) ? "" : "_" + std::to_string( i_bin_section );
        has_DLnode_recognized_items = RecognizeItemsInContainer("bin_"+bin_id+bin_id_section);

        // Condition created to skip the remaining bin sections if there were items recognized
        if( has_DLnode_recognized_items )
        {
          // If pose hasn't previously failed we used that one
          int dl_item_idx = 0;
          ros::Duration(1.0).sleep(); // TODO: Check if this is required for tf
          if( SelectCandidateFromDL(dl_item_idx, bin_id, looking_for_known_item, vec_poses_blacklist) )
          {
            // Get pose from the DL vectors
            dl_target_item = vec_dl_item_ids_[dl_item_idx];
            dl_item_conf = vec_dl_confidences_[dl_item_idx];
            dl_item_pose_in_world = transform_pose_now(vec_dl_item_center_poses_in_cam_[dl_item_idx], "/iiwa_link_0", tf_listener);
            dl_grasp_line_pt1_pose_in_world = transform_pose_now(vec_dl_grasp_line_pt1_poses_[dl_item_idx], "/iiwa_link_0", tf_listener);
            dl_grasp_line_pt2_pose_in_world = transform_pose_now(vec_dl_grasp_line_pt2_poses_[dl_item_idx], "/iiwa_link_0", tf_listener);
            got_item_candidate = true;
            break;
          }
          else
          {
            ROS_INFO_STREAM("No suitable candidate found from DL");
          }
        }
      } // Bin section cycle end here

      if( !has_DLnode_recognized_items ) // DL_recognize_items is false
      {
        ROS_INFO("No items were found on bin %s", bin_id.c_str());
        continue;
      }

      // Candidate found, skip known or unknown cycle
      if (got_item_candidate)
      {
        break;
      }
      else // There were items in the DL list but none of them belong to an order
      {
        if (looking_for_known_item == 0) //known item
        {
          ROS_INFO("No known item was found on bin %s", bin_id.c_str());
        }
        else //unknown
        {
          ROS_INFO("No unknown item was found on bin %s", bin_id.c_str());
        }
        continue;
      }
    } // Bins cycle ends here

    // Condition to skip the next bin cycle if an item candidate was found
    if (got_item_candidate)
    {
      break;
    }
    else
    {
      ROS_INFO("Required items were not found in bin %s", bin_id.c_str());
    }
  }
  return got_item_candidate;
}

// Returns true if it found an item
bool TaskManagerNode::SearchInSingleBin(int& looking_for_known_item, 
  std::string& dl_target_item, float& dl_item_conf, 
  geometry_msgs::PoseStamped& dl_item_pose_in_world, 
  geometry_msgs::PoseStamped& dl_grasp_line_pt1_pose_in_world, 
  geometry_msgs::PoseStamped& dl_grasp_line_pt2_pose_in_world, 
  std::string& bin_id_including_section, std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist, 
  bool filter_for_sections, bool section_is_A1)
{
  ROS_ERROR("=============== SEARCHING PHASE ==============="); 
  std::string type_of_item_searched = (looking_for_known_item == 0 ? "known" : "unknown");
  ROS_INFO_STREAM("Looking into " << bin_id_including_section << " to find " << type_of_item_searched << " items");
  // Number of sections per bin 
  bool got_item_candidate = false;
  bool has_DLnode_recognized_items = false; 

  std::string bin_id;
  if ( (bin_id_including_section.compare("bin_A_1") == 0) || (bin_id_including_section.compare("bin_A_2") == 0))
  {
    bin_id = "A";
  }
  else if ( (bin_id_including_section.compare("bin_B") == 0))
  {
    bin_id = "B";
  }
  else if ( (bin_id_including_section.compare("bin_C") == 0))
  {
    bin_id = "C";
  }


  RestoreLedInitialState(); //set our leds' intensity

  // looking_for_known_item is the flag created to determine if we are searching for known or unknown items
  // 0 = Known, 1 = Unknown
  if (looking_for_known_item == 0)
  {
    if (!OrderItemsInBin(bin_id, type_of_item_searched))
    {
      ROS_INFO_STREAM("Bin " << bin_id << " skipped because there are no " << type_of_item_searched << " needed items inside");
      return false;
    }
  }

  GoToContainer("bin_" + bin_id);

  has_DLnode_recognized_items = RecognizeItemsInContainer(bin_id_including_section);

  // Condition created to skip the remaining bin sections if there were items recognized
  if( has_DLnode_recognized_items )
  {
    // If pose hasn't previously failed we used that one
    int dl_item_idx = 0;
    ros::Duration(1.0).sleep(); // TODO: Check if this is required for tf
    if( SelectCandidateFromDL(dl_item_idx, bin_id, looking_for_known_item, vec_poses_blacklist, filter_for_sections, section_is_A1) )
    {
      // Get pose from the DL vectors
      dl_target_item = vec_dl_item_ids_[dl_item_idx];
      dl_item_conf = vec_dl_confidences_[dl_item_idx];
      dl_item_pose_in_world = transform_pose_now(vec_dl_item_center_poses_in_cam_[dl_item_idx], "/iiwa_link_0", tf_listener);;
      dl_grasp_line_pt1_pose_in_world = transform_pose_now(vec_dl_grasp_line_pt1_poses_[dl_item_idx], "/iiwa_link_0", tf_listener);
      dl_grasp_line_pt2_pose_in_world = transform_pose_now(vec_dl_grasp_line_pt2_poses_[dl_item_idx], "/iiwa_link_0", tf_listener);
      got_item_candidate = true;
    }
    else
    {
      ROS_INFO_STREAM("No suitable candidate found from DL");
    }
  }

  // Candidate found, skip known or unknown cycle
  if (!got_item_candidate)
  {
    if (looking_for_known_item == 0) //known item
    {
      ROS_INFO("No known item was found on bin %s", bin_id.c_str());
    }
    else //unknown
    {
      ROS_INFO("No unknown item was found on bin %s", bin_id.c_str());
    }
  }

  return got_item_candidate;
}

//weight_target_container is probably unnecessary as an input
bool TaskManagerNode::RetrievalPhase(bool& weight_service_ready, std::string target_item, float item_conf,
  geometry_msgs::PoseStamped item_pose_in_world, 
  geometry_msgs::PoseStamped grasp_line_pt1_pose_in_world, 
  geometry_msgs::PoseStamped grasp_line_pt2_pose_in_world, std::string bin_id, bool fuzzyMode)
{
  ROS_ERROR("=============== RETRIEVAL PHASE ===============");
  // If an item was recognized in the previous step we will try to retrieve it
  bool item_retrieved = 0;

  ROS_INFO_STREAM("Highest confidence item: " << target_item);
  ROS_INFO_STREAM("Confidence value: " << item_conf);
  ROS_WARN_STREAM("[TNP_STATE L0] Executing tnp_deep_vision/recognize_items Highest confidence item " << target_item);

  stringstream formatted_stream;
  formatted_stream << fixed << setprecision(2) << round(item_conf*100.0)/100.0;

  PublishTaskMessages( "Deep Learning EE (" + std::to_string(dl_vote_share_) + "):", false );
  PublishTaskMessages( "   "+target_item+" | "+formatted_stream.str(), true );

  // Determine the weight sensors to be used by weight service depending on bin_id
  // bin_a for bin A and bin_bc for bins B and C
  std::string weight_target_container = bin_id.compare("A") == 0 ? "bin_a" : "bins_bc";
  // Prepare the weight service to retrieve information from current bin
  weight_service_ready = WeightGetReadyToPick(target_item, weight_target_container);

  // 0 for Suck and 1 for Grasp
  int item_retrieve_method = 0, retrieve_force = 0;
  GenerateStatusMessage("GetRetrieveMethod", "RetrieveItem", " ", " "," ");

  if( !GetRetrieveMethod( target_item, item_retrieve_method, retrieve_force ) )
  {
    ROS_ERROR( "Failed to obtain preferred retrieve method. Setting retrieve_force to heavy" );
    item_retrieve_method = 0;
    retrieve_force = 40;
  }

  // Cycle to try to retrieve item first by preferred method and if that fails use the opposite method
  ROS_INFO("Trying to retrieve item");
  publishItem(target_item, item_pose_in_world.pose);

  // Calling RetrieveItem function
  GenerateStatusMessage("RetrieveItem", "WeightRecognizeItem", "RetrieveItem", " "," ");

  ROS_INFO_STREAM("Target item: " << target_item);
  ROS_INFO_STREAM("Target pose: " << item_pose_in_world);
  if(debug_on_) getch();

  // First we suck, then if that failed, we add a grasp attempt for certain items

  /// Kuka tries to pick up the item here
  RetrieveItem(0, target_item, item_pose_in_world,
                   grasp_line_pt1_pose_in_world, grasp_line_pt2_pose_in_world,
                   retrieve_force, fuzzyMode);


  /// Try grasping if suction failed
  if (fuzzyMode && !item_is_suctioned_)
  {
    RetrieveItem(1, target_item, item_pose_in_world,
                   grasp_line_pt1_pose_in_world, grasp_line_pt2_pose_in_world,
                   retrieve_force, fuzzyMode);  
  }
  
  if ((item_retrieve_method == 0 && item_is_suctioned_) || item_retrieve_method == 1) // TODO: add || item_is_grasped_
  {
    ROS_INFO("Item %s retrieved successfully", target_item.c_str());
    item_retrieved = true;
  }
  else
  {
    ROS_ERROR("Item %s couldn't be retrieved", target_item.c_str());
    item_retrieved = false;
  }
  GoToContainer("bin_"+bin_id);

  if ((item_retrieve_method == 0 && item_is_suctioned_) || item_retrieve_method == 1) // TODO: add || item_is_grasped_
  {
    item_retrieved = true;
  }
  else
  {
    ROS_ERROR("Item %s dropped on the way up.", target_item.c_str());
    item_retrieved = false;
  }
  return item_retrieved;
}

bool TaskManagerNode::DemocracyPhase(std::string& belief_item_id, float& belief_confidence, 
  bool& weight_service_ready, bool& item_in_order, std::string dl_target_item, 
  float dl_item_conf, geometry_msgs::PoseStamped dl_item_pose_in_world, 
  std::vector<geometry_msgs::PoseStamped>& vec_poses_blacklist,
  std::string bin_id, int& looking_for_known_item)
{
  ROS_ERROR("=============== DEMOCRACY PHASE ===============");

  std::string weight_target_container = bin_id.compare("A") == 0 ? "bin_a" : "bins_bc";
  // Get weight vote
  GenerateStatusMessage("WeightRecognizeItem", "GotoRecSpace", " ", " "," ");
  if (weight_service_ready)
  {
    if( !WeightRecognizeItems(dl_target_item, weight_target_container ))
    {
      //clear the weight vectors if we don't get a response from the weight node
      vec_weight_items_ids_.clear();
      vec_weight_items_confs_.clear();
    }
  }
  // Publish the weight service response before we start the rec space routine
  std::vector<std::pair<std::string, float> > weight_combined_vectors = pairedVectorSortedByConfidence(vec_weight_items_ids_, vec_weight_items_confs_); 
  if(weight_combined_vectors.size()>0)
  {
    stringstream formatted_stream0, formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream4, formatted_stream5, formatted_stream6;
    formatted_stream0 << fixed << setprecision(0) << measured_weight_;
    formatted_stream1 << fixed << setprecision(2) << round(weight_combined_vectors[0].second*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(weight_combined_vectors[1].second*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(weight_combined_vectors[2].second*100.0)/100.0;
    formatted_stream4 << fixed << setprecision(2) << round(weight_combined_vectors[3].second*100.0)/100.0;
    formatted_stream5 << fixed << setprecision(2) << round(weight_combined_vectors[4].second*100.0)/100.0;
    formatted_stream6 << fixed << setprecision(2) << round(weight_vote_share_*100.0)/100.0;
    PublishTaskMessages( "Weight (" + formatted_stream6.str() + "): " + formatted_stream0.str() + " g", true );
    PublishTaskMessages( "   "+weight_combined_vectors[0].first+" | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[1].first+" | "+formatted_stream2.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[2].first+" | "+formatted_stream3.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[3].first+" | "+formatted_stream4.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[4].first+" | "+formatted_stream5.str(), true );
  }

  // Move to the Recognition space
  GenerateStatusMessage("GotoRecSpace", "RecSpaceRecognizeItems", " ", " "," ");
  GoToRecSpace();
  GenerateStatusMessage("GotoRecSpace", "RecSpaceRecognizeItems", " ", " "," ");
  // Get the bounding box and get SVM candidates
  if(RecSpaceGetBoundingBox())
  {
    GetCandidatesFromSVM(bounding_box_information_.bb_width.data, bounding_box_information_.bb_length.data, bounding_box_information_.bb_height.data, measured_weight_); 
  }

  // Call RecSpaceRecognizeItems to get the cloud matching, Akaze SVM and color histogram
  GenerateStatusMessage("RecSpaceRecognizeItems", "VoteForItem", " ", " "," ");
  if(!RecSpaceRecognizeItems(dl_target_item, GetContainerItems("bin_" + bin_id)))
  {
    ROS_ERROR("RecSpaceRecognizeItems failed");
  }

  //sorting vectors by the highest confidence
  //weight
  std::vector<std::pair<std::string, float> > svm_combined_vectors = pairedVectorSortedByConfidence(vec_svm_items_ids_, vec_svm_confs_); 
  std::vector<std::pair<std::string, float> > akaze_svm_combined_vectors = pairedVectorSortedByConfidence(vec_akaze_svm_items_ids_, vec_akaze_svm_confs_); 
  std::vector<std::pair<std::string, float> > color_histogram_combined_vectors = pairedVectorSortedByConfidence(vec_color_histogram_items_ids_, vec_color_histogram_confs_);  

  //pseudo-normalization
  float pseudo_normalization_factor = 4.0;

  if( svm_combined_vectors.size() > 0 )
  {
    float tmp_normalization_factor = 1.0 / ( svm_combined_vectors[0].second + 1e-6 );
    if( tmp_normalization_factor > pseudo_normalization_factor )
    {
      pseudo_normalization_factor = tmp_normalization_factor;
    }

    for( size_t i=0; i < vec_svm_confs_.size(); ++i )
    {
      vec_svm_confs_[i] *= pseudo_normalization_factor;
      svm_combined_vectors[i].second *= pseudo_normalization_factor;
    }
  }

  // This can also be used to record the data easily. We can parse the text from the named logfile later
  dump_decision_making_data_to_screen(dl_target_item, dl_item_conf, weight_combined_vectors, svm_combined_vectors, vec_cloud_matching_items_ids_, vec_cloud_matching_confs_, akaze_svm_combined_vectors, color_histogram_combined_vectors);
  write_decision_making_data_to_log(ofs_data_gathering_, dl_target_item, dl_item_conf, weight_combined_vectors, svm_combined_vectors, vec_cloud_matching_items_ids_, vec_cloud_matching_confs_, akaze_svm_combined_vectors, color_histogram_combined_vectors);

  // Display confidences on screen
  DumpDecisionDataToTnpMonitor(weight_combined_vectors, svm_combined_vectors, vec_cloud_matching_items_ids_, vec_cloud_matching_confs_, akaze_svm_combined_vectors, color_histogram_combined_vectors);
  /*debug*/if(debug_on_) getch();

  bool ignore_dl = false;
  if (looking_for_known_item == 0) {ignore_dl = false;}
  else {ignore_dl = true;}  // When looking for untrained items, DL should not count  
  // Instead of looking_for_known_item, the parameter should be if any known items are ordered & in this bin
  bool identity_detected = decide_item_identity_from_all_votes(belief_item_id, belief_confidence, dl_target_item, dl_item_conf, ignore_dl);

  // If the possible items vector contains items the system will determine if that item
  // was in the order and try to put it in its corresponding box
  if (identity_detected)
  {
    int order_id = 0;
    // Check if belief item is in order and in bin
    if (ItemInOrder(belief_item_id, order_id))
    {
      if (ItemInBin(bin_id, belief_item_id))
      {
        ROS_INFO("Determined item found in order %d", order_id + 1);
        item_in_order = true;
      }
      else
      {
        ROS_ERROR("The determined item was not in the origin bin");
        vec_poses_blacklist.push_back(dl_item_pose_in_world);
      }
    }
    else
    {
      ROS_INFO("The determined item was not included in any order");
      vec_poses_blacklist.push_back(dl_item_pose_in_world);
    }
    GenerateStatusMessage("DetermineBox", "PutItemIntoBox", " ", " "," ");
  }
  return identity_detected;
}


bool TaskManagerNode::RegisterItemAsDelivered(std::string item_id, std::string origin_bin_id)
{
  ROS_ERROR("=============== REGISTRATION PHASE ===============");
  // Register item in the location file and update the item/order location in memory
  
  int delivery_box_id = GetBoxIDOfItem(item_id);
  int order_id = -1;
  ItemInOrder(item_id, order_id);

  UpdateItemLocationInMemory( item_id, "bin_" + origin_bin_id, "box_" + std::to_string(delivery_box_id) );
  UpdateOrderLocationInMemory( order_id, item_id, "box", std::to_string(delivery_box_id) );

  // Close the drawer if no ordered items remain in bin C
  if (!OrderItemsInBin("C",  "any")) { CloseDrawer(); }

  try{
    FILE *LocationFile;
    LocationFile = fopen( "/root/share/output_pick/item_location_file_pick_output.json", "w" );
    fprintf( LocationFile, "%s", location_list_.toString().c_str() );
    fclose( LocationFile );
  }
  catch(...)
  {
    ROS_ERROR("SAVING FILE FAILED");
  }

  if (OrderListCompleted())
  {
    ROS_INFO("All the orders were processed");
  }

  return true;
}


bool TaskManagerNode::PickTaskCallback(tnp_task_manager::pickTask::Request &req,
  tnp_task_manager::pickTask::Response &res)
{
  // 0) Set up
  // Variables to validate the time passed since the start of the service call
  const ros::Time begin_time = ros::Time::now();
  //Start counter to close drawer
  std::thread t1( &TaskManagerNode::DrawerTimeLimit, this, begin_time );
  //Publish a true, to sync with the time on the screen.
  publishRunningStatus(false);
  ros::Duration(0.25).sleep();
  publishRunningStatus(true);
  ROS_WARN("[TNP_STATE L0] Starting /tnp_task_manager/PickTaskCallback Starting pick task");
 
  // Open the drawer 
  OpenDrawer(); 
  // Initialize the following vectors used to store the data from DL, Weight and Rec Space services (?) 
  DLInitializeVectors(); 
 
  // 1) Take all the sweet pickings (the visible trained items in each bin)
  int num_pick_cycles = 0; 
  std::vector<std::string> all_bin_IDs {"bin_A_2", "bin_A_1", "bin_B", "bin_C"};
  int num_attempts = 0;
  int max_num_attempts_on_known_bins = 5;
  ROS_WARN("Starting the first sweep of known items!");
  /// This for-loop is the one where we try to pick all known visible items before restructuring the bins
  for (int i = all_bin_IDs.size()-1; i > -1; i--)
  {
    std::string bin_id;
    if ( (all_bin_IDs[i].compare("bin_A_1") == 0) || (all_bin_IDs[i].compare("bin_A_2") == 0))
    {
      bin_id = "A";
    }
    else if ( (all_bin_IDs[i].compare("bin_B") == 0))
    {
      bin_id = "B";
    }
    else if ( (all_bin_IDs[i].compare("bin_C") == 0))
    {
      bin_id = "C";
    }
    bool no_known_items_left_visible = false;
    while (!no_known_items_left_visible) 
    { 
      // Variables to store when an item fails to be picked up in order to prevent infinite loop 
      std::vector<geometry_msgs::PoseStamped> vec_poses_blacklist; 
       // Variable that define the type of tool used to retrieve the item 0-Suction 1-Grip and Force 
      int item_retrieve_method = 0; 
      int retrieve_force = 0; 

      ROS_WARN_STREAM("[TNP_STATE L0] Starting to pick the low-hanging fruit for " << all_bin_IDs[i]); 

      // Variables to determine success or failure of different actions 
      bool got_item_candidate = false; 
      bool weight_service_ready = false; 
      bool item_retrieved = false; 
      bool blind_retrieving = false; 

      // Variable to store the order_id to determine in which box to store the recognized item 
      int order_id = -1; 

      ///////////////////////////////////////////////////////////////////////////////////////// 
      /// SEARCHING PHASE 
      ///////////////////////////////////////////////////////////////////////////////////////// 
      
      // Declarations of the outputs that will be used in the following phases
      std::string dl_target_item;  
      float dl_item_conf;  
      geometry_msgs::PoseStamped dl_item_pose_in_world;  
      geometry_msgs::PoseStamped dl_grasp_line_pt1_pose_in_world; 
      geometry_msgs::PoseStamped dl_grasp_line_pt2_pose_in_world; 
      int looking_for_known_item = 0;
      
      got_item_candidate = SearchInSingleBin(looking_for_known_item, 
        dl_target_item, dl_item_conf, 
        dl_item_pose_in_world, 
        dl_grasp_line_pt1_pose_in_world, 
        dl_grasp_line_pt2_pose_in_world, 
        all_bin_IDs[i], vec_poses_blacklist);

      if (debug_on_) getch();

      /////////////////////////////////////////////////////////////////////////////////////////
      /// RETRIEVAL PHASE
      /////////////////////////////////////////////////////////////////////////////////////////

      if (got_item_candidate)
      {
          item_retrieved = RetrievalPhase(weight_service_ready,
          dl_target_item, dl_item_conf, dl_item_pose_in_world, 
          dl_grasp_line_pt1_pose_in_world, dl_grasp_line_pt2_pose_in_world, bin_id);
      }
      else
      {
        no_known_items_left_visible = true;
      }

      if (debug_on_) getch();

      //////////////////////////////////////////////////////////////////////////////
      /// DEMOCRACY PHASE
      //////////////////////////////////////////////////////////////////////////////
      ROS_ERROR("=============== DEMOCRACY PHASE ===============");

      std::string belief_item_id = "";
      float belief_confidence = 0.0;
      bool item_in_order = false;

      // Get weight vote
      GenerateStatusMessage("WeightRecognizeItem", "GotoRecSpace", " ", " "," ");
      if (item_retrieved)
      {
        if (!DemocracyPhase(belief_item_id, belief_confidence, 
          weight_service_ready, item_in_order, dl_target_item, 
          dl_item_conf, dl_item_pose_in_world, vec_poses_blacklist, 
          bin_id, looking_for_known_item)) // no believes (possible_items_ids is empty)
        {
          ROS_INFO("System was unable to determine a valid item, returning to origin bin");
          ROS_WARN("[TNP_STATE L0] Start /tnp_task_manager System was unable to determine a valid item, returning to origin bin");
          GoToContainer("bin_"+bin_id);
          /// TODO Move the item to another bin designated as the garbage bin
          geometry_msgs::PoseStamped pose_above_dl_item_pose_in_world = dl_item_pose_in_world;
          pose_above_dl_item_pose_in_world.pose.position.z += 0.12;
          PutItemIntoContainer( "bin_" + bin_id, item_retrieve_method == 0 ? false : true, pose_above_dl_item_pose_in_world);
          GoToContainer("bin_"+bin_id);
        }
      }
      else // item not retrieved
      {
        ROS_ERROR("The item couldn't be retrieved after %d attempts, adding to temporal failure vector to prevent loop",
                  max_retrieve_attempts_);
        ROS_WARN("[TNP_STATE L0] Error /tnp_task_manager The item couldn't be retrieved after %d attempts, adding to temporal failure vector to prevent loop",
                 max_retrieve_attempts_ );

        vec_poses_blacklist.push_back(dl_item_pose_in_world);
      }

      if (debug_on_) getch();

      //////////////////////////////////////////////////////////////////////////////
      /// DELIVERY PHASE
      //////////////////////////////////////////////////////////////////////////////
      ROS_ERROR("=============== DELIVERY PHASE ===============");
      bool item_delivered = false;

      // Validate item an place it in the box or back into a bin
      if (item_in_order && (item_is_suctioned_)) // Add itemIsGrasped as an alternative
      {
        item_delivered = DeliverHeldItemToBox(belief_item_id, item_retrieve_method);
        GoToHome();
      }
      else if (item_is_suctioned_)
      {
        ROS_INFO("The recognized item does not belong to any order, returning to origin bin");
        ROS_WARN("[TNP_STATE L0] Start /tnp_task_manager The recognized item does not belong to any order, returning to origin bin");
        GoToContainer("bin_"+bin_id);
        /// TODO Move the item to another section and remove blacklisted poses on opposite section
        geometry_msgs::PoseStamped pose_above_dl_item_pose_in_world = dl_item_pose_in_world;
        pose_above_dl_item_pose_in_world.pose.position.z += 0.12;
        PutItemIntoContainer( "bin_" + bin_id, item_retrieve_method == 0 ? false : true, pose_above_dl_item_pose_in_world);
        GoToContainer("bin_"+bin_id);

        vec_poses_blacklist.push_back(dl_item_pose_in_world);
      }
      else
      {
        ROS_INFO("No item was successfully picked. Skipping delivery.");
      }

      //////////////////////////////////////////////////////////////////////////////
      /// REGISTRATION PHASE
      //////////////////////////////////////////////////////////////////////////////
      // Register item in the location file and update the item/order location in memory
      if (item_delivered)
      { 
        RegisterItemAsDelivered(belief_item_id, bin_id);
      }
      ROS_WARN_STREAM("At end of loop, blacklist size was: " << vec_poses_blacklist.size());
      num_attempts++;
      if (num_attempts > max_num_attempts_on_known_bins)
      {
        break;
      }
    }
  }
  ROS_WARN("**********************************************");
  ROS_WARN("***********The first sweep is over!***********");
  ROS_WARN("**********************************************");

  // If all items picked, do the finishing routine
  if ( (!OrderItemsInBin( "C", "all" )) && (DRAWER_IS_CLOSED_ == false) )
  {
    ROS_INFO("Closing drawer as there are no items left in bin C.");
    ros::Duration(1).sleep();
    CloseDrawer();
    ros::Duration(1).sleep();
    DRAWER_IS_CLOSED_ = true;
  }

  // 2) Empty side 1 of bin A into side 2
  std::string refusal_bin = "bin_A_1";
  MoveToBinAndUpdateOccupancyMap(refusal_bin);
  TakeItemsOutOfBinUntil("bin_C", "no_orders_or_empty", refusal_bin);
  CloseDrawer();
  ros::Duration(1).sleep();
  TakeItemsOutOfBinUntil("bin_B", "no_orders_or_empty", refusal_bin);
  TakeItemsOutOfBinUntil("bin_A_2", "no_orders_or_empty", refusal_bin);

  while (!OrderListCompleted())
  {
    TakeItemsOutOfBinUntil("bin_C", "no_orders_or_empty", "bin_A_2");
    TakeItemsOutOfBinUntil("bin_B", "no_orders_or_empty", "bin_A_2");
    TakeItemsOutOfBinUntil("bin_A_1", "no_orders_or_empty", "bin_A_2");
    TakeItemsOutOfBinUntil("bin_A_1", "no_orders_or_empty", "bin_A_2");
    TakeItemsOutOfBinUntil("bin_A_2", "no_orders_or_empty", "bin_A_1");  
    TakeItemsOutOfBinUntil("bin_A_2", "no_orders_or_empty", "bin_A_1");
  }

  // 3) Close the drawer C
  CloseDrawer();

  return true;
}

void TaskManagerNode::MoveToRefusalBin(std::string item_id, std::string origin_bin_id, std::string refusal_bin, int item_retrieve_method)
{ 
  std::string refusal_bin_id_without_section;
  if ( (refusal_bin.compare("bin_A_1") == 0) || (refusal_bin.compare("bin_A_2") == 0))
  {
    refusal_bin_id_without_section = "A";
  }
  else if ( (refusal_bin.compare("bin_B") == 0))
  {
    refusal_bin_id_without_section = "B";
  }
  else if ( (refusal_bin.compare("bin_C") == 0))
  {
    refusal_bin_id_without_section = "C";
  }

  ROS_INFO_STREAM("Moving " << item_id << " from bin " << origin_bin_id << " into refusal bin " << refusal_bin);
  GoToContainer(refusal_bin);
  geometry_msgs::PoseStamped dummy_pose;
  dummy_pose.pose.orientation.w;
  GetPoseToPlaceItem(refusal_bin, dummy_pose, 
    bounding_box_information_.bb_width.data, 
    bounding_box_information_.bb_height.data, 
    bounding_box_information_.bb_length.data); 
  PutItemIntoContainer(refusal_bin, item_retrieve_method == 0 ? false : true, pose_to_place_item_in_world_coords_);
  GoToContainer(refusal_bin);
  if (!free_spot_found_)
  {
    ROS_INFO("Updating the occupancy map because no free spot was found in refusal bin");
    MoveToBinAndUpdateOccupancyMap(refusal_bin);
  }
}

void TaskManagerNode::MoveToBinAndUpdateOccupancyMap(std::string bin_id_with_section)
{
  std::string bin_id_without_section = bin_id_with_section;
  if ((bin_id_with_section.compare("bin_A_1")  == 0) || (bin_id_with_section.compare("bin_A_2") == 0))
  {
    bin_id_without_section = "bin_A";
  }
  
  GoToContainer(bin_id_without_section);
  GoToLookIntoContainer(bin_id_without_section, 0);
  ros::Duration(4.0).sleep();
  UpdateOccupancyMap(bin_id_without_section);
  GoToContainer(bin_id_without_section);
}

// Checks:
// "known_items_visible" - Until no known items visible
// "no_orders_or_empty" - Until either no ordered items remain in it or it is empty
// "empty"
// Undesired items are put into refusal bin
void TaskManagerNode::TakeItemsOutOfBinUntil(std::string bin_id_including_section, std::string breakout_condition, std::string refusal_bin)
{
  bool fuzzyMode = true;
  bool keep_going = true;
  bool no_known_items_left_visible = false;
  std::string bin_id;
  bool filter_for_sections = false;
  bool section_is_A1 = ((bin_id_including_section.compare("bin_A_1") == 0) ? true : false);
  int num_of_failed_attempts_in_a_row = 0;
  if ( (bin_id_including_section.compare("bin_A_1") == 0) || (bin_id_including_section.compare("bin_A_2") == 0))
  {
    bin_id = "A";
    filter_for_sections = true;
  }
  else if ( (bin_id_including_section.compare("bin_B") == 0))
  {
    bin_id = "B";
  }
  else if ( (bin_id_including_section.compare("bin_C") == 0))
  {
    bin_id = "C";
  }

  if (!OrderItemsInBin(bin_id, "all"))
  {
    ROS_INFO_STREAM("There are no order items in the bin. Will not go to bin ." << bin_id);   
    return;
  }

  PublishTaskMessages("Looking for unkown items in bin " + bin_id, true);
  
  // Variables to store when an item fails to be picked up in order to prevent infinite loop 
  std::vector<geometry_msgs::PoseStamped> vec_poses_blacklist; 
  while (keep_going)
  {
     // Variable that define the type of tool used to retrieve the item 0-Suction 1-Grip and Force 
    int item_retrieve_method = 0; 
    int retrieve_force = 0; 

    ROS_WARN_STREAM("[TNP_STATE L0] Starting to take items out of " << bin_id_including_section << " with strategy: " << breakout_condition); 
    PublishTaskMessages("num_of_failed_attempts_in_a_row: " + std::to_string(num_of_failed_attempts_in_a_row), true);
    PublishTaskMessages("length of pose blacklist: " + std::to_string(vec_poses_blacklist.size()), true);

    // Variables to determine success or failure of different actions 
    bool got_item_candidate = false; 
    bool weight_service_ready = false; 
    bool item_retrieved = false; 
    bool blind_retrieving = false; 

    // Variable to store the order_id to determine in which box to store the recognized item 
    int order_id = -1; 

    ///////////////////////////////////////////////////////////////////////////////////////// 
    /// SEARCHING PHASE 
    ///////////////////////////////////////////////////////////////////////////////////////// 
    
    // Declarations of the outputs that will be used in the following phases
    std::string dl_target_item;  
    float dl_item_conf;
    geometry_msgs::PoseStamped dl_item_pose_in_world;  
    geometry_msgs::PoseStamped dl_grasp_line_pt1_pose_in_world; 
    geometry_msgs::PoseStamped dl_grasp_line_pt2_pose_in_world; 
    int looking_for_known_item = 1;

    got_item_candidate = SearchInSingleBin(looking_for_known_item, 
      dl_target_item, dl_item_conf, 
      dl_item_pose_in_world, 
      dl_grasp_line_pt1_pose_in_world, 
      dl_grasp_line_pt2_pose_in_world, 
      bin_id_including_section, vec_poses_blacklist, 
      filter_for_sections, section_is_A1);

    if (debug_on_) getch();

    /////////////////////////////////////////////////////////////////////////////////////////
    /// RETRIEVAL PHASE
    /////////////////////////////////////////////////////////////////////////////////////////

    if (got_item_candidate)
    {
        item_retrieved = RetrievalPhase(weight_service_ready,
        dl_target_item, dl_item_conf, dl_item_pose_in_world, 
        dl_grasp_line_pt1_pose_in_world, dl_grasp_line_pt2_pose_in_world, bin_id,
        fuzzyMode);
    }
    else
    {
      no_known_items_left_visible = true;
      ROS_WARN("This is where the blacklist inside TakeItemsOutOfBinUntil would have been cleared");
    }

    if (debug_on_) getch();

    // Check if the item weight is feasible at all
    if (item_retrieved)
    {
      if (!CheckByWeightIfItemIsFeasible(bin_id))
      {
        ROS_INFO_STREAM("The weight of the picked item is not close to anything we want. Putting into refusal bin.");
        MoveToRefusalBin(dl_target_item, "bin_" + bin_id, refusal_bin, item_retrieve_method);
        continue;
      }
    }

    //////////////////////////////////////////////////////////////////////////////
    /// DEMOCRACY PHASE
    //////////////////////////////////////////////////////////////////////////////
    ROS_ERROR("=============== DEMOCRACY PHASE ===============");

    std::string belief_item_id = "";
    float belief_confidence = 0.0;
    bool item_in_order = false;

    // Get weight vote
    GenerateStatusMessage("WeightRecognizeItem", "GotoRecSpace", " ", " "," ");
    if (item_retrieved)
    {
      if (!DemocracyPhase(belief_item_id, belief_confidence, 
        weight_service_ready, item_in_order, dl_target_item, 
        dl_item_conf, dl_item_pose_in_world, vec_poses_blacklist, 
        bin_id, looking_for_known_item)) // no believes (possible_items_ids is empty)
      {
        ROS_INFO_STREAM("System was unable to determine a valid item, putting into refusal_bin: " << refusal_bin);
        ROS_WARN_STREAM("[TNP_STATE L0] Start /tnp_task_manager System was unable to determine a valid item, putting into refusal_bin: " << refusal_bin);
        MoveToRefusalBin(belief_item_id, "bin_" + bin_id, refusal_bin, item_retrieve_method);
      }
      num_of_failed_attempts_in_a_row = 0;
    }
    else // item not retrieved
    {
      ROS_ERROR("The item couldn't be retrieved after %d attempts, adding to temporal failure vector to prevent loop",
                max_retrieve_attempts_);
      ROS_WARN("[TNP_STATE L0] Error /tnp_task_manager The item couldn't be retrieved after %d attempts, adding to temporal failure vector to prevent loop",
               max_retrieve_attempts_ );
      num_of_failed_attempts_in_a_row++;
    }

    if (debug_on_) getch();

    //////////////////////////////////////////////////////////////////////////////
    /// DELIVERY PHASE
    //////////////////////////////////////////////////////////////////////////////
    ROS_ERROR("=============== DELIVERY PHASE ===============");
    bool item_delivered = false;

    // Validate item an place it in the box or back into a bin
    if (item_in_order && (item_is_suctioned_)) // Add itemIsGrasped as an alternative
    {
      item_delivered = DeliverHeldItemToBox(belief_item_id, item_retrieve_method);
      GoToHome();
    }
    else if (item_is_suctioned_)
    {
      ROS_INFO_STREAM("The recognized item does not belong to any order, putting into refusal_bin: " << refusal_bin);
      ROS_WARN_STREAM("[TNP_STATE L0] Start /tnp_task_manager The recognized item does not belong to any order, putting into refusal_bin: " << refusal_bin);
      MoveToRefusalBin(belief_item_id, "bin_" + bin_id, refusal_bin, item_retrieve_method);
    }
    else
    {
      ROS_INFO("No item was picked. Skipping delivery.");
    }

    //////////////////////////////////////////////////////////////////////////////
    /// REGISTRATION PHASE
    //////////////////////////////////////////////////////////////////////////////
    // Register item in the location file and update the item/order location in memory
    if (item_delivered)
    { 
      RegisterItemAsDelivered(belief_item_id, bin_id);
    }

    if (breakout_condition.compare("no_known_items_visible") == 0)
    {
      if (no_known_items_left_visible)
      {
        ROS_INFO("No new items visible anymore. Breaking out of loop.");
        break;
      }
    }
    else if (breakout_condition.compare("empty") == 0)
    {

    }
    else if (breakout_condition.compare("no_orders_or_empty") == 0)
    {
      if (!OrderItemsInBin(bin_id, "all"))
      {
        ROS_INFO("There are no order items left in the bin. Breaking out of loop.");   
        break;
      }
      else
      {
        ROS_INFO("Ordered items are left in bin. Repeating.");
      }
    }

    if (num_of_failed_attempts_in_a_row >= 5)
    {
      ROS_ERROR("Too many failed attempts! Stopping to attempt this bin.");
      break;
    }
   
    if (!got_item_candidate)
    {
      ROS_INFO("The container appears empty. Breaking out of loop.");
      break;
    }
  }
}

// Assumes that weight sensors are set to ready
bool TaskManagerNode::CheckByWeightIfItemIsFeasible(std::string bin_id)
{
  ROS_INFO_STREAM("Checking if the weight resembles an ordered item from the bin: " << bin_id);
  std::string bin_name = bin_id.compare("A") == 0 ? "bin_a" : "bins_bc";
  // Get the similarity of current weight to all objects in container
  ros::Duration(1.0).sleep();
  WeightRecognizeItems(order_list_.listOrders[0].content[0], bin_name);

  int dummy;
  // Check if any of the remaining items in the order are in the vector & similar
  for (int i = 0; i < vec_weight_items_ids_.size(); i++)
  {
    if (ItemInOrder(vec_weight_items_ids_[i].data, dummy))
    {
      if (ItemInBin(bin_id, vec_weight_items_ids_[i].data))
      {
        if (vec_weight_items_confs_[i].data > .8)
        {
          ROS_INFO_STREAM("The weight is similar to ordered item :" << vec_weight_items_ids_[i].data);
          return true;
        }
      }
    }
  }

  ROS_INFO_STREAM("No similar item has been found. The item is probably not part of the ordered ones. ");
  
  // Publish the weight service response
  std::vector<std::pair<std::string, float> > weight_combined_vectors = pairedVectorSortedByConfidence(vec_weight_items_ids_, vec_weight_items_confs_); 
  if(weight_combined_vectors.size()>0)
  {
    stringstream formatted_stream0, formatted_stream1, formatted_stream2, formatted_stream3, formatted_stream4, formatted_stream5;
    formatted_stream0 << fixed << setprecision(0) << measured_weight_;
    formatted_stream1 << fixed << setprecision(2) << round(weight_combined_vectors[0].second*100.0)/100.0;
    formatted_stream2 << fixed << setprecision(2) << round(weight_combined_vectors[1].second*100.0)/100.0;
    formatted_stream3 << fixed << setprecision(2) << round(weight_combined_vectors[2].second*100.0)/100.0;
    formatted_stream4 << fixed << setprecision(2) << round(weight_combined_vectors[3].second*100.0)/100.0;
    formatted_stream5 << fixed << setprecision(2) << round(weight_combined_vectors[4].second*100.0)/100.0;
    PublishTaskMessages( "Weight: " +formatted_stream0.str() + " g", true );
    PublishTaskMessages( "   "+weight_combined_vectors[0].first+" | "+formatted_stream1.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[1].first+" | "+formatted_stream2.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[2].first+" | "+formatted_stream3.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[3].first+" | "+formatted_stream4.str(), true );
    PublishTaskMessages( "   "+weight_combined_vectors[4].first+" | "+formatted_stream5.str(), true );
  }

  PublishTaskMessages("No similar ordered item. Putting aside. ", true );
  return false;
}


std::vector<std::string> TaskManagerNode::GetTagsForSuctionCandidatesUsingDLInfo()
{
  /// NOTE we assume vec_suction_candidate_poses_in_world_ is ordered by the highest z-value
  ROS_INFO("GetTagsForSuctionCandidatesUsingDLInfo executed");

  ROS_INFO_STREAM("GetTagsForSuctionCandidatesUsingDLInfo: num of suckable surfaces received: " << vec_suction_candidate_poses_in_world_.size());
  ROS_INFO_STREAM("GetTagsForSuctionCandidatesUsingDLInfo: num of items recognized: " << vec_dl_item_ids_.size());

  const size_t MAX_SUCTION_CANDIDATES = 100; ///TODO ros parameter
  const size_t MAX_DL_ITEMS = 32; ///TODO ros parameter
  const float CONFIDENCE_WEIGHT = 0.3; ///TODO ros parameter
  const float BB_CENTER_POSE_Z_WEIGHT = 0.7; ///TODO ros parameter

  std::vector<std::string> vec_tags_for_suction_candidates_using_dl_info;

  // Use image_geometry package to transform from spatial coordinates to pixels.
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info_);

  for(size_t i=0; i<vec_suction_candidate_poses_in_camera_frame_.size() && i<MAX_SUCTION_CANDIDATES; ++i)
  {
    ROS_INFO_STREAM(vec_suction_candidate_poses_in_camera_frame_[i].pose.position);
    const cv::Point3d point3d(vec_suction_candidate_poses_in_camera_frame_[i].pose.position.x,
                              vec_suction_candidate_poses_in_camera_frame_[i].pose.position.y,
                              vec_suction_candidate_poses_in_camera_frame_[i].pose.position.z);
    ROS_INFO_STREAM("point3d "<<i<<" " <<point3d);
    const cv::Point2d point2d = cam_model.project3dToPixel(point3d);
    ROS_INFO_STREAM("point2d "<<i<<" " <<point2d);

    // vec_dl_item_ids_ should have the same num of elements of the bounding boxes
    std::string tag_of_most_likely_item = "";
    float max_weird_likeliness = 0.0;
    for(size_t j(0); j < vec_dl_item_ids_.size() && j < MAX_DL_ITEMS; ++j)
    {
      //ROS_INFO_STREAM(vec_dl_item_center_x_[j])
      if( point2d.x > vec_dl_item_center_x_[j] - vec_dl_item_width_[j]/2.0 // left limit of bb
       && point2d.x < vec_dl_item_center_x_[j] + vec_dl_item_width_[j]/2.0 // right limit of bb
       && point2d.y > vec_dl_item_center_y_[j] - vec_dl_item_height_[j]/2.0 // top of bb
       && point2d.y < vec_dl_item_center_y_[j] + vec_dl_item_height_[j]/2.0 // bottom of bb
      )
      {
        //inside the bounding box
        const float weird_likeliness = vec_dl_confidences_[j] * CONFIDENCE_WEIGHT +
                                       vec_dl_item_center_poses_in_cam_[j].pose.position.z * BB_CENTER_POSE_Z_WEIGHT;
        if(weird_likeliness > max_weird_likeliness)
        {
          max_weird_likeliness = weird_likeliness;
          tag_of_most_likely_item = vec_dl_item_ids_[j];
        }
      }
    }
    vec_tags_for_suction_candidates_using_dl_info.push_back(tag_of_most_likely_item);
  }

  ROS_WARN( "Content of vec_tags" );

  for(int i = 0; i < vec_tags_for_suction_candidates_using_dl_info.size(); ++i )
  {
    ROS_WARN_STREAM( vec_tags_for_suction_candidates_using_dl_info[i] );
  }

  ROS_INFO("GetTagsForSuctionCandidatesUsingDLInfo finished");
  return vec_tags_for_suction_candidates_using_dl_info;
}


// Rapid Stow strategy
bool TaskManagerNode::RapidStowTaskCallback(tnp_task_manager::stowTask::Request &req,
  tnp_task_manager::stowTask::Response &res)
{
  ROS_WARN("[TNP_STATE L0] Starting /tnp_task_manager/rapidStowTaskCallback Starting rapid stow task");
  srand (time(NULL)); /* initialize random seed: */

  // Record the initial time
  ros::Time begin_time = ros::Time::now();
  ///Send a true to the monitor to sync
  publishRunningStatus(false);
  ros::Duration(0.5).sleep();
  publishRunningStatus(true);
  //Start counter to close drawer
  std::thread close_drawer_timer( &TaskManagerNode::DrawerTimeLimit, this, begin_time ); // no need cause we're not opening the drawer but it's better to leave JIC

  // Initialization of variables
  int retrieval_tool = 0; //0:suction, 1:gripper
  bool last_delivery_attempt_succeeded = false; // to avoid going to tote and then to lookintotote when we failed to take something
  int num_of_failed_attempts = 0;
  int num_of_failed_attempts_in_a_row = 0;
  int num_of_available_candidates = 0;
  int num_of_DL_elements = 0;
  const int MAX_NUM_OF_ATTEMPTS_BEFORE_SWEEP = 3;
  double crowdiness = 1.0;
  bool skip_octomap = false;
  float sweep_height = 0.01;
  double crowdiness_threshold = 0.25;
  bool use_fuzzy_mode = false;
  int hard_mode = 0;

  // if there are suckable surfaces they will be tagged with the DL info (vector should be aligned to vec_suction_candidate_poses_in_world_)
  std::vector<geometry_msgs::PoseStamped> vec_suckable_surfaces_blacklist;
  std::vector<geometry_msgs::PoseStamped> vec_dl_poses_blacklist;
  double blacklist_distance = 0.1;
  std::vector<std::string>  vec_tags_for_suction_candidates_using_dl_info;

  while (!TimeLimitReached(begin_time))
  {
    // set the led's to the best for DL
    GenerateStatusMessage("GoToContainer", "GoToLookIntoContainer", " ", " "," ");
    if(last_delivery_attempt_succeeded)
    {
      GoToContainer( "tote" );
      last_delivery_attempt_succeeded = false; //for safety
    }
    else { /*skip it and go directly to look into bin to save time*/}

    /// SEARCHING PHASE
    // Look into tote
    GenerateStatusMessage("GoToLookIntoContainer", "UpdateOccupancyMap", " ", " "," ");
    GoToLookIntoContainer( "tote", 0 );

    // Regenerate the Octomap & DL candidates
    GenerateStatusMessage("UpdateOccupancyMap", "DLRecognizeItems", " ", " "," ");
    if(vec_suction_candidate_poses_in_world_.size() == 0 || num_of_failed_attempts > num_of_available_candidates )
    {
      RestoreLedInitialState();
      vec_suckable_surfaces_blacklist.clear();
      vec_dl_poses_blacklist.clear();
      GenerateStatusMessage("DLRecognizeItems", "RetrieveItem", " ", " "," ");
      // update the map of the tote
      OpenEEShutterCloseTheRest(); // close the other shutters and open the EE

      if( !skip_octomap )
      {
        ros::Duration(4.0).sleep(); ///NOTE the robot to be stopped to properly get depth info
        UpdateOccupancyMap("tote");
        GetSuctionCandidatesContainerFromOctomap("tote", 0/*section*/);
      }
      else
      {
        ros::Duration(2.5).sleep();
      }

      DLRecognizeItems(/*use_stow*/true); // this also gets the grasping candidates
      num_of_failed_attempts = 0;
      if(vec_suction_candidate_poses_in_world_.size() > 0)
      {
        vec_tags_for_suction_candidates_using_dl_info = GetTagsForSuctionCandidatesUsingDLInfo();
      }
      num_of_available_candidates = vec_suction_candidate_poses_in_world_.size();
      num_of_DL_elements = vec_dl_item_ids_.size();

      if( num_of_DL_elements / 20.0 < 0.25  )
      {
        skip_octomap = true;
      }
      else
      {
        skip_octomap = false;
      }
    }
    else { /*continue using the DL info and suckable surfaces in memory to avoid looking again*/}

    /*debug*/if( debug_on_ ) getch(); // press a key to continue

    std::vector<std::string> tote_elements = GetContainerItems( "tote" );
    if( /*num_of_DL_elements == 0 ||*/ tote_elements.size() == 0 )
    {
      ROS_WARN( "***********************************" );
      ROS_WARN( "***********SUCCESS!!!?*************" );
      ROS_WARN( "***********************************" );
    }

    ////////////////////////////////////////////////////////////////////////
    // RETRIEVAL PHASE
    ////////////////////////////////////////////////////////////////////////

    bool candidate_came_from_DL = false;
    bool candidate_came_from_surfaces = false;
    // Get a target point for the suction & pick it
    bool last_retrieval_attempt_succeeded = false;
    RetrievalCandidateStruct retrieval_candidate;
    while(!last_retrieval_attempt_succeeded
      && !TimeLimitReached(begin_time)
      && num_of_failed_attempts <= num_of_available_candidates )
    {

      // Finding a retrieval candidate
      bool got_retrievable_candidate = false;
      candidate_came_from_DL = false;
      candidate_came_from_surfaces = false;
      
      crowdiness = vec_dl_item_ids_.size() / 20.0; // 20.0 is the maximum number of items
      if (crowdiness > 1.0) crowdiness = 1.0;
      ROS_WARN_STREAM("Crowdiness is " << crowdiness);
      stringstream formatted_stream;
      formatted_stream << fixed << setprecision(2) << round(crowdiness*100.0)/100.0;
      PublishTaskMessages( "Crowdiness: " + formatted_stream.str(), false );
      PublishTaskMessages( "Num_of_failed_attempts: " + std::to_string(num_of_failed_attempts), true );
      PublishTaskMessages( "num_of_failed_attempts_in_a_row: " + std::to_string(num_of_failed_attempts_in_a_row), true );
      PublishTaskMessages( "hard_mode: " + std::to_string(hard_mode), true );
      PublishTaskMessages( "length dl candidates: " + std::to_string(vec_dl_item_ids_.size()), true );
      PublishTaskMessages( "length surface blacklist: " + std::to_string(vec_suckable_surfaces_blacklist.size()), true );
      PublishTaskMessages( "length surface candidates: " + std::to_string(vec_suction_candidate_poses_in_world_.size()), true );
      PublishTaskMessages( "length surface blacklist: " + std::to_string(vec_suckable_surfaces_blacklist.size()), true );

      // Go for either DL or Octomap candidates
      if (vec_suction_candidate_poses_in_world_.size() > 0) // If DL is not yet active or failed to find a suckable spot
      {
        geometry_msgs::PoseStamped grasp_line_pt1, grasp_line_pt2;
        ROS_INFO("Selecting a candidate from Octomap.");
        
        if(vec_suckable_surfaces_blacklist.size() == 0) // no blacklisted surfaces (considering only the centers)
        {
          retrieval_candidate.suck_pose = vec_suction_candidate_poses_in_world_.at(0);

          grasp_line_pt1 = vec_suction_candidate_poses_in_world_.at(0);
          grasp_line_pt2 = grasp_line_pt1;
          grasp_line_pt1.pose.position.x += .1;

          retrieval_candidate.suck_pose_DL_tags.push_back(vec_tags_for_suction_candidates_using_dl_info.at(0));
          retrieval_candidate.dl_grasp_line_pt1_poses = grasp_line_pt1;
          retrieval_candidate.dl_grasp_line_pt2_poses = grasp_line_pt2;
          got_retrievable_candidate = true;
          candidate_came_from_surfaces = true;
        }
        else
        {
          ROS_ERROR_STREAM("There are " << vec_suckable_surfaces_blacklist.size() << " blacklisted surfaces");
          int num_of_poses_skipped = 0;
          for(int i_suckable_surface=0; i_suckable_surface < vec_suction_candidate_poses_in_world_.size(); ++i_suckable_surface)
          {
            bool pose_matched_blacklist_element = false;
            for(int i_blacklisted_surface=0; i_blacklisted_surface < vec_suckable_surfaces_blacklist.size(); ++i_blacklisted_surface)
            {
              const float dx = vec_suction_candidate_poses_in_world_[i_suckable_surface].pose.position.x - vec_suckable_surfaces_blacklist[i_blacklisted_surface].pose.position.x;
              const float dy = vec_suction_candidate_poses_in_world_[i_suckable_surface].pose.position.y - vec_suckable_surfaces_blacklist[i_blacklisted_surface].pose.position.y;
              const float square_distance_xy = dx*dx + dy*dy;
              const float distance_xy = sqrt(square_distance_xy);
              if(distance_xy < blacklist_distance) // candidate is far from blacklisted ///TODO this should be a ROS parameter
              {
                pose_matched_blacklist_element = true;
                break;
              }
            }

            if( !pose_matched_blacklist_element )
            {
              try
              {
                retrieval_candidate.suck_pose = vec_suction_candidate_poses_in_world_.at(i_suckable_surface);

                grasp_line_pt1 = vec_suction_candidate_poses_in_world_.at(i_suckable_surface);
                grasp_line_pt2 = grasp_line_pt1;
                grasp_line_pt1.pose.position.x += .1;
                
                retrieval_candidate.suck_pose_DL_tags.push_back(vec_tags_for_suction_candidates_using_dl_info.at(i_suckable_surface));
                retrieval_candidate.dl_grasp_line_pt1_poses = grasp_line_pt1;
                retrieval_candidate.dl_grasp_line_pt2_poses = grasp_line_pt2;
                got_retrievable_candidate = true;
                candidate_came_from_surfaces = true;
              }
              catch (...)
              {
                ROS_WARN_STREAM("Error when retrieving a non-blacklisted element. i = " << i_suckable_surface);
                retrieval_candidate.suck_pose_DL_tags.push_back("unknown");
                got_retrievable_candidate = true;
                candidate_came_from_surfaces = true;
              }
              break;
            }
            else
            {
              got_retrievable_candidate = false;
              ++num_of_poses_skipped;
            }
          }

          if (!got_retrievable_candidate)
          {
            ROS_INFO_STREAM("Found no retrievable candidate from list of length " << vec_suction_candidate_poses_in_world_.size() << " as all poses were blacklisted apparently. Clearing blacklist.");
            vec_suckable_surfaces_blacklist.clear();
            vec_suction_candidate_poses_in_world_.clear();
          }

          num_of_available_candidates = vec_suction_candidate_poses_in_world_.size() - num_of_poses_skipped;
        }
      }
      else if (vec_dl_item_ids_.size() > 0)
      {
        ROS_INFO_STREAM("Selecting a candidate from DL. There are " << vec_dl_poses_blacklist.size() << " blacklisted elements.");
        num_of_available_candidates = num_of_DL_elements;
        //DL as a detector
        int num_of_poses_skipped = 0;
        for(size_t i(0); i < vec_dl_item_center_poses_in_cam_.size(); ++i )
        {
          geometry_msgs::PoseStamped candidate_pose = transform_pose_now(vec_dl_item_center_poses_in_cam_[i], "/iiwa_link_0", tf_listener);
          // Check if item's pose is far enough from previously failed attempts
          if( vec_dl_poses_blacklist.size() == 0 )
          {
            ROS_INFO_STREAM("Found retrievable candidate from list of length" << vec_dl_item_center_poses_in_cam_.size() << " as there are no blacklisted poses.");
            retrieval_candidate.suck_pose = candidate_pose;
            retrieval_candidate.suck_pose_DL_tags.push_back(vec_dl_item_ids_[i]);
            got_retrievable_candidate = true;
            candidate_came_from_DL = true;
            break;
          }
          else
          {
            bool pose_matched_blacklist_element = false;
            for(int i_blacklisted_pose=0; i_blacklisted_pose < vec_dl_poses_blacklist.size(); ++i_blacklisted_pose)
            {
              try
              {
                const float dx = candidate_pose.pose.position.x - vec_dl_poses_blacklist[i_blacklisted_pose].pose.position.x;
                const float dy = candidate_pose.pose.position.y - vec_dl_poses_blacklist[i_blacklisted_pose].pose.position.y;
                const float square_distance_xy = dx*dx + dy*dy;
                const float distance_xy = sqrt(square_distance_xy);
                if(distance_xy < blacklist_distance) // candidate is far from blacklisted ///TODO this should be a ROS parameter
                {
                  pose_matched_blacklist_element = true;
                  break;
                }
              }
              catch(...)
              {
                ROS_ERROR_STREAM("Error at i = " << i_blacklisted_pose);
              }
            }

            if( !pose_matched_blacklist_element )
            {
              retrieval_candidate.suck_pose = candidate_pose;
              retrieval_candidate.suck_pose_DL_tags.push_back(vec_dl_item_ids_[i]);
              retrieval_candidate.dl_grasp_line_pt1_poses = vec_dl_grasp_line_pt1_poses_[i];
              retrieval_candidate.dl_grasp_line_pt2_poses = vec_dl_grasp_line_pt2_poses_[i];
              got_retrievable_candidate = true;
              candidate_came_from_DL = true;
              break;
            }
            else
            {
              got_retrievable_candidate = false;
              ++num_of_poses_skipped;
            }
          }
        }
        if (!got_retrievable_candidate)
        {
          ROS_INFO_STREAM("Found no retrievable candidate from list of length" << vec_dl_item_ids_.size() << " as all poses were blacklisted apparently. Clearing blacklist.");
          vec_dl_poses_blacklist.clear();
          vec_dl_item_ids_.clear();
          vec_suction_candidate_poses_in_world_.clear();
        }
        

        num_of_available_candidates = vec_dl_item_center_poses_in_cam_.size() - num_of_poses_skipped;
      }

      PublishTaskMessages( "candidate_came_from_DL: " + std::to_string(candidate_came_from_DL), true );
      PublishTaskMessages( "candidate_came_from_surfaces: " + std::to_string(candidate_came_from_surfaces), true );

      // Picking up the item
      if(got_retrievable_candidate)
      {
        GenerateStatusMessage("RetrieveItem", "GoToContainer", "RetrieveItem", " "," ");
        // Publish
        if (retrieval_candidate.suck_pose_DL_tags.size() > 0)
        {
          PublishTaskMessages( "Deep learning (EE): ", true );
          PublishTaskMessages( "   - " + retrieval_candidate.suck_pose_DL_tags.at(0), true );
          PublishTaskMessages( "      X: " + std::to_string(retrieval_candidate.suck_pose.pose.position.x), true );
          PublishTaskMessages( "      Y: " + std::to_string(retrieval_candidate.suck_pose.pose.position.y), true );
          PublishTaskMessages( "      Z: " + std::to_string(retrieval_candidate.suck_pose.pose.position.z), true );
        }

        // prepare weight
        ROS_WARN( "DEBUG BEFORE GET READY TO PICK" );
        WeightGetReadyToPick(retrieval_candidate.suck_pose_DL_tags.at(0), "tote");

        /*debug*/geometry_msgs::PoseStamped graspLinePt1, graspLinePt2;
        ROS_WARN( "DEBUG RETRIEVE ITEM" );

        use_fuzzy_mode = false;

        if(hard_mode >= 1)
        {
          use_fuzzy_mode = true;
        }

        if(RetrieveItem( 0, "unknown", retrieval_candidate.suck_pose, graspLinePt1, graspLinePt2, 40.0, use_fuzzy_mode, "tote" )) //too much force
        {
          if(item_is_suctioned_)
          {
            last_retrieval_attempt_succeeded = true;

            if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
            if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
            
            num_of_failed_attempts_in_a_row = 0;
          }
          else
          {
            last_retrieval_attempt_succeeded = false;
            if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
            if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
            ++num_of_failed_attempts;
            ++num_of_failed_attempts_in_a_row;
            GoToContainer("tote");
          }
        }
        else
        {
          ROS_INFO("Failed to retrieve item.");
          last_retrieval_attempt_succeeded = false;
          
          // Also try grasping
          if( (hard_mode >= 1) && !last_retrieval_attempt_succeeded )
          {
            ROS_WARN("Trying to grasp");
            RetrieveItem( 1, "unknown", retrieval_candidate.suck_pose, retrieval_candidate.dl_grasp_line_pt1_poses, retrieval_candidate.dl_grasp_line_pt2_poses, 1500.0, use_fuzzy_mode, "tote" );
            if(item_is_grasped_)
            {
              last_retrieval_attempt_succeeded = true;

              if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
              if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}  
              
              num_of_failed_attempts_in_a_row = 0;
            }
            else
            {
              ROS_WARN("No successful grasp detected");
              last_retrieval_attempt_succeeded = false;
              if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
              if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
              ++num_of_failed_attempts;
              ++num_of_failed_attempts_in_a_row;
              GoToContainer("tote");
            }
          }
          else
          {
            ROS_INFO("Adding to blacklist.");
            if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
            if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
            ++num_of_failed_attempts;
            ++num_of_failed_attempts_in_a_row;
          }
        }
      }

      // Horizontal sweep (last-ditch movement)
      if ( (!last_retrieval_attempt_succeeded) &&
          ( ( num_of_failed_attempts_in_a_row > MAX_NUM_OF_ATTEMPTS_BEFORE_SWEEP && crowdiness < 0.05 ) ||   // If we failed too often in a row
          ((vec_suction_candidate_poses_in_world_.size() == 0) && (vec_dl_item_ids_.size() == 0)) ) )       // If neither DL nor suction found anything
      {
        // Do the horizontal sweep or something
        GoToContainer( "tote" );
        SweepToteHorizontal( sweep_height, false );
        if(item_is_suctioned_)
        {
          last_retrieval_attempt_succeeded = true;

          // Blacklist even after success when it's unlikely that there is an item underneath
          if (crowdiness < 0.5)
          {
            if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
            if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
          }

          num_of_failed_attempts_in_a_row = 0;
        }

        // And once more to the left if the other failed.
        if (!last_retrieval_attempt_succeeded)
        {
          SweepToteHorizontal( sweep_height, true );
          if(item_is_suctioned_)
          {
            last_retrieval_attempt_succeeded = true;

            // Blacklist even after success when it's unlikely that there is an item underneath
            if (crowdiness < 0.5)
            {
              if (candidate_came_from_surfaces) {vec_suckable_surfaces_blacklist.push_back(retrieval_candidate.suck_pose);}
              if (candidate_came_from_DL) {vec_dl_poses_blacklist.push_back(retrieval_candidate.suck_pose);}
            }

            num_of_failed_attempts_in_a_row = 0;
          }
        }
        sweep_height -= 0.03;
        hard_mode = 1;
      }

      if (num_of_failed_attempts_in_a_row >= 4)
      {
        ROS_WARN("Setting hard mode on.");
        hard_mode = 1;  
      }

      // Stop attempting retrieve if no candidates left
      if ( (vec_dl_item_ids_.size() == 0) && (vec_suction_candidate_poses_in_world_.size() == 0 ))  { 
        PublishTaskMessages( "No  " + std::to_string(candidate_came_from_DL), true );
        break;
      }

    } // end of while
    ///////////////////////// END OF SENSING & RETRIEVAL PHASE


    if (last_retrieval_attempt_succeeded)
    {
      ROS_INFO("Moving to tote after successful retrieval.");
      GoToContainer("tote");
    }

    PublishTaskMessages( "Successfully retrieved item. Delivering.", true );

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEMOCRACY PHASE (Rapid Stow)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM("Entering democracy phase.");
    std::vector<std::string> possible_items_ids;
    std::vector<float> possible_items_confidences;
    std::string destination_bin = "bin_A";
    std::string final_item_name = "";
    float final_confidence = 0.0;
    if(last_retrieval_attempt_succeeded)
    {
      WeightRecognizeItems("", "tote");
      GoToContainer("bin_A");   // MANUAL INTERVENTION FOR THE WEIGHT SENSORS
      
      //recognize with weight

      stringstream formatted_stream;
      formatted_stream << fixed << setprecision(2) << round(measured_weight_*100.0)/100.0;
      PublishTaskMessages("Weight: " + formatted_stream.str() + " g", true );

      //weight
      for(size_t iWeightRes = 0; iWeightRes < vec_weight_items_ids_.size(); ++iWeightRes)
      {
        retrieval_candidate.weight_item_ids.push_back(vec_weight_items_ids_[iWeightRes].data);
        retrieval_candidate.weight_confidences.push_back(vec_weight_items_confs_[iWeightRes].data);
      }

      std::vector<std::pair<std::string, float> > weight_combined_vectors;
      for(size_t i=0; i<retrieval_candidate.weight_item_ids.size() && i<retrieval_candidate.weight_confidences.size(); ++i)
      {
        weight_combined_vectors.push_back(std::make_pair(retrieval_candidate.weight_item_ids[i], retrieval_candidate.weight_confidences[i]));
      }
      std::sort (weight_combined_vectors.begin(), weight_combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

      ROS_WARN( "DEBUG BEFORE PRINTING WGT" );
      PublishTaskMessages( "Weight highest candidates: ", true );
      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(weight_combined_vectors.at(0).second*100.0)/100.0;
      PublishTaskMessages( "   - " + weight_combined_vectors.at(0).first + " | " + formatted_stream.str(), true );
      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(weight_combined_vectors.at(1).second*100.0)/100.0;
      PublishTaskMessages( "   - " + weight_combined_vectors.at(1).first + " | " + formatted_stream.str(), true );
      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(weight_combined_vectors.at(2).second*100.0)/100.0;
      PublishTaskMessages( "   - " + weight_combined_vectors.at(2).first + " | " + formatted_stream.str(), true );
      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(weight_combined_vectors.at(3).second*100.0)/100.0;
      PublishTaskMessages( "   - " + weight_combined_vectors.at(3).first + " | " + formatted_stream.str(), true );
      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(weight_combined_vectors.at(4).second*100.0)/100.0;
      PublishTaskMessages( "   - " + weight_combined_vectors.at(4).first + " | " + formatted_stream.str(), true );
      
      if( retrieval_candidate.suck_pose_DL_tags.size() > 0 )
      {
        decide_item_identity_from_all_votes_stow( possible_items_ids, possible_items_confidences,
          retrieval_candidate.suck_pose_DL_tags.at(0), retrieval_candidate.dl_confidence );
      }
      else
      {
        decide_item_identity_from_all_votes_stow( possible_items_ids, possible_items_confidences,
          "", retrieval_candidate.dl_confidence ); 
      }
      
      //order the vector of possible items
      std::vector<std::pair<std::string, float> > combined_vectors;
      for(size_t i=0; i<possible_items_ids.size() && i<possible_items_confidences.size(); ++i)
      {
        combined_vectors.push_back(std::make_pair(possible_items_ids[i], possible_items_confidences[i]));
      }
      std::sort (combined_vectors.begin(), combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

      std::vector<std::pair<std::string, float> > svm_combined_vectors;
      for(size_t i=0; i<vec_svm_items_ids_.size() && i<vec_svm_confs_.size(); ++i)
      {
        svm_combined_vectors.push_back(std::make_pair(vec_svm_items_ids_[i], vec_svm_confs_[i]));
      }
      std::sort (svm_combined_vectors.begin(), svm_combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

      std::vector<std::pair<std::string, float> > akaze_svm_combined_vectors;
      for(size_t i=0; i<vec_akaze_svm_items_ids_.size() && i<vec_akaze_svm_confs_.size(); ++i)
      {
        akaze_svm_combined_vectors.push_back(std::make_pair(vec_akaze_svm_items_ids_[i], vec_akaze_svm_confs_[i]));
      }
      std::sort (akaze_svm_combined_vectors.begin(), akaze_svm_combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

      std::vector<std::pair<std::string, float> > color_histogram_combined_vectors;
      for(size_t i=0; i<vec_color_histogram_items_ids_.size() && i<vec_color_histogram_confs_.size(); ++i)
      {
        color_histogram_combined_vectors.push_back(std::make_pair(vec_color_histogram_items_ids_[i], vec_color_histogram_confs_[i]));
      }
      std::sort (color_histogram_combined_vectors.begin(), color_histogram_combined_vectors.end(), orderCandidatesCombinedVectorsIdConfs );

      dump_decision_making_data_to_screen( retrieval_candidate.dl_item_id,
                                           retrieval_candidate.dl_confidence,
                                           weight_combined_vectors,
                                           svm_combined_vectors,
                                           vec_cloud_matching_items_ids_,
                                           vec_cloud_matching_confs_,
                                           akaze_svm_combined_vectors,
                                           color_histogram_combined_vectors);
      
      write_decision_making_data_to_log( ofs_data_gathering_, 
                                         retrieval_candidate.dl_item_id,
                                         retrieval_candidate.dl_confidence,
                                         weight_combined_vectors,
                                         svm_combined_vectors,
                                         vec_cloud_matching_items_ids_,
                                         vec_cloud_matching_confs_,
                                         akaze_svm_combined_vectors,
                                         color_histogram_combined_vectors);
      
      // assign a name to the item for the registry and
      // if confidence is high decide the bin
      for (size_t i = 0; i < combined_vectors.size(); ++i)
      {
        if (combined_vectors[i].first.length() > 0 && ItemInTote(combined_vectors[i].first) )
        {
          final_item_name = combined_vectors[i].first;
          final_confidence = combined_vectors[i].second;
          
          if (combined_vectors[i].second > 0.8) ///TODO this should be a ros parameter
          {
            DetermineBinBySize(combined_vectors[i].first, destination_bin);
            destination_bin = "bin_"+destination_bin;
            if(destination_bin.compare("bin_C")==0)
            {
              destination_bin = "bin_B"; // avoiding bin_C to nulify risk of having it open at the end of the round
            }
          }

          break;
        }
      }

      ROS_WARN_STREAM("destination_bin " << destination_bin);
      
      ROS_INFO("----------------------------------");
      ROS_WARN_STREAM("Final determined item: " << final_item_name);
      ROS_WARN_STREAM("Confidence: " << final_confidence);
      ROS_INFO("---------------------------------------");

      formatted_stream.str(std::string());
      formatted_stream << fixed << setprecision(2) << round(final_confidence*100.0)/100.0;
      PublishTaskMessages( "Final determined item: ", true );
      PublishTaskMessages( "   "+final_item_name+" | "+formatted_stream.str(), true );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DELIVERY PHASE (putting the item into the storage system)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM("Entering delivery phase.");
    if(last_retrieval_attempt_succeeded)
    {
      ROS_INFO_STREAM("Entering delivery phase, as last retrieval attempt succeeded.");
      ///erase the first element to avoid visiting the same spot
      if ( (candidate_came_from_surfaces) && (vec_suction_candidate_poses_in_world_.size() > 0) )
      {
        vec_suction_candidate_poses_in_world_.erase(vec_suction_candidate_poses_in_world_.begin());
      }
      else
      {
        ROS_INFO("Suction candidate vector was < 0. Not erasing element.");
      }

      if ( (candidate_came_from_DL) && (vec_tags_for_suction_candidates_using_dl_info.size() > 0))
      {
        vec_tags_for_suction_candidates_using_dl_info.erase(vec_tags_for_suction_candidates_using_dl_info.begin());
      }
      else
      {
        ROS_INFO("DL candidate vector was < 0. Not erasing element.");
      }

      GenerateStatusMessage("GoToContainer", "GetPoseToPlaceItem", " ", " "," ");
      GoToContainer( destination_bin );
      GenerateStatusMessage("GetPoseToPlaceItem", "PutItemIntoContainer", " ", " "," ");
      // find a free spot where to place the item
      /*debug*/geometry_msgs::PoseStamped bounding_box_center_pose; //TODO this value is ignored
      bounding_box_center_pose.pose.orientation.w = 1.0;
      ROS_WARN_STREAM("destination_bin " << destination_bin);
      GetPoseToPlaceItem(destination_bin, bounding_box_center_pose, 0.15, 0.15, 0.105); ///TODO this is just a rough approximation

      bool need_to_update_occupancy_map = false;
      if(free_spot_found_) {/*no need to update map*/ ROS_WARN("Free spot found");}
      else
      {
        ROS_WARN_STREAM("Task manager: free spot not found");
        need_to_update_occupancy_map = true;
      }
      GenerateStatusMessage("PutItemIntoContainer", " ", " ", " "," "); //TODO Define the next state

      PutItemIntoContainer( destination_bin, retrieval_tool == 0 ? false : true, pose_to_place_item_in_world_coords_);
      last_delivery_attempt_succeeded = true;

      if( need_to_update_occupancy_map )
      {
        GoToLookIntoContainer(destination_bin, 1/*height (1,2,3)*/);
        // update the map of the tote
        OpenEEShutterCloseTheRest(); // close the other shutters and open the EE
        ros::Duration(4.0).sleep(); ///NOTE the robot to be stopped to properly get depth info
        UpdateOccupancyMap(destination_bin);
      }
    }
    
    try
    {
      /// REGISTRATION PHASE
      /// TODO register the item after being tagged or all of them
      if(last_delivery_attempt_succeeded)
      {
        ROS_INFO_STREAM("Delivery succeeded. Registering item.");

        //// Update location of item
        UpdateItemLocationInMemory(final_item_name, "tote", destination_bin);
        const std::string filepath("/root/share/output_stow/item_location_file_output.json");
        std::ofstream ofs_stow_output(filepath.c_str());
        if(ofs_stow_output)
        {
          ofs_stow_output << location_list_.toString();
          ofs_stow_output.close();
        }
        else
        {
          ROS_ERROR_STREAM("File error: " << filepath);
        }
      }
    }
    catch(...)
    {
      ROS_ERROR("Error when writing item to location file.");
    }
    ROS_INFO_STREAM("Last line of the loop.");
  }

  ROS_WARN("[TNP_STATE L0] Done /tnp_task_manager/RapidStowTaskCallback Service called successfully");
  
  publishRunningStatus(false);

  return true;
}

bool TaskManagerNode::SetupBeforeRoundCallback(tnp_task_manager::setupBeforeRound::Request &req,
                                               tnp_task_manager::setupBeforeRound::Response &res)
{
  bool result = false;
  std::string task_name = req.task_name;

  CloseDrawer();

  // Loading parameters from ros::param
  LoadParameters();

  // Loading data from files (box_sizes, item_location_file, image and info files)
  if (!LoadFiles(task_name))
  {
    ROS_ERROR("There was a problem loading information from json files");
    result = false;
  }

  // Sending items information to weight node
  if (!WeightSetItemsInfo())
  {
    ROS_ERROR("There was a problem calling WeightSetItemsInfo service");
    result = false;
  }

  // Sending items location to weight node
  if (!WeightSetItemsLocation())
  {
    ROS_ERROR("There was a problem calling WeightSetItemsLocation service");
    result = false;
  }

  if( task_name.compare( "pick" ) == 0 )
  {
    // Loading data for orders information (order_file, box_n_size parameter)
    if (!LoadBoxesAndOrders())
    {
      ROS_ERROR("There was a problem loading information from ros parameters");
      result = false;
    }
  }

  RecSpaceSetItemsInfo();

  if (task_name.compare("pick") == 0)
  {
    // Trigger the background shooting 5 takes required by the recognition space to work
    ROS_WARN("Is the robot in the Recognition space? If not, move it there to take the shots for the background removal");
    DeploySuction();
    GoToRecSpace();
    RecSpaceRecordEmptyRS(5);
  }
  else
  {
    ROS_WARN("Skipping RecSpace recording because it's stowing time");
  }

  ROS_INFO("Finished setting up round.");
  return result;
}

bool TaskManagerNode::InitialSetupCallback(tnp_task_manager::initialSetup::Request &req,
                                               tnp_task_manager::initialSetup::Response &res)
{
  bool result = false;
  FILE *RetrieveFile;

  RetrieveFile = fopen( "/root/share/PutAmazonDataHere/our_prior_knowledge.json", "r" );
  if( RetrieveFile == NULL )
  {
    ROS_INFO( "Creating our_prior_knowledge.json" );
    retrieve_list_.load( "/root/share/PutAmazonDataHere" );
    RetrieveFile = fopen( "/root/share/PutAmazonDataHere/our_prior_knowledge.json", "w" );
    fprintf( RetrieveFile, "%s", retrieve_list_.toString().c_str() );
    fclose( RetrieveFile );
    result = true;
  }
  else
  {
    fclose( RetrieveFile );
    ROS_WARN( "our_prior_knowledge.json was already created, if you want to create the new one" );
    ROS_WARN( "delete the existing one in the shared folder an run this service again" );
    result = true;
  }

  return result;
}

bool TaskManagerNode::Transform2PosesCallback(tnp_task_manager::transform2poses::Request &req,
                                               tnp_task_manager::transform2poses::Response &res)
{
  ROS_INFO_STREAM( "Received Pose: " << req.original_pose );
  ROS_INFO_STREAM( "Target Frame " << req.target_frame );

  res.transformed_pose = transform_pose_now( req.original_pose, req.target_frame , tf_listener );

  return true;
}


/*-------------------------------------------------------------------------------------*/
/// main
/*-------------------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  FILE *jsonFile;

  ros::init(argc, argv, "tnp_task_manager");

  //Create an object of class TaskManagerNode that will take care of everything
  TaskManagerNode SAPObject;
  ROS_WARN("[TNP_STATE L0] Starting /tnp_task_manager Starting the task manager node");
  ROS_INFO("Task manager node started");

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
