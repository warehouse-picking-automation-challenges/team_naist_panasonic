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

#include "tnp_grasp_planner.h"

////----------------------------------------------------------------------------
//// Conditional Euclidean Clustering Condition Functions
////----------------------------------------------------------------------------
int squared_distance;
float thresh_intensity;
float thresh_normal_angle_cos;
float thresh_curvature;
float thresh_intensity_l_dist;

bool enforceIntensitySimilarity(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b,
                                float squared_distance)
{
  if (fabs(point_a.intensity - point_b.intensity) < thresh_intensity)
    return (true);
  else
    return (false);
}
bool enforceCurvatureSimilarity(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b,
                                float squared_distance)
{
  Eigen::Vector3f point_a_normal = Eigen::Map<const Eigen::Vector3f>(point_a.normal, 3);
  Eigen::Vector3f point_b_normal = Eigen::Map<const Eigen::Vector3f>(point_b.normal, 3);
  if (fabs(point_a_normal.dot(point_b_normal)) > thresh_normal_angle_cos)
  {
    if (fabs(point_a.curvature - point_b.curvature) < thresh_curvature)
      return (true);
  }
  return (false);
}
bool enforceCurvatureOrIntensitySimilarity(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b,
                                           float squared_distance)
{
  Eigen::Vector3f point_a_normal = Eigen::Map<const Eigen::Vector3f>(point_a.normal, 3);
  Eigen::Vector3f point_b_normal = Eigen::Map<const Eigen::Vector3f>(point_b.normal, 3);
  if (fabs(point_a_normal.dot(point_b_normal)) > thresh_normal_angle_cos)
    return (true);
  if (fabs(point_a.intensity - point_b.intensity) < thresh_intensity)
    return (true);
  return (false);
}
bool customRegionGrowing(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b,
                         float squared_distance)
{
  Eigen::Vector3f point_a_normal = Eigen::Map<const Eigen::Vector3f>(point_a.normal, 3);
  Eigen::Vector3f point_b_normal = Eigen::Map<const Eigen::Vector3f>(point_b.normal, 3);
  if (squared_distance < squared_distance)
  {
    if (fabs(point_a_normal.dot(point_b_normal)) > thresh_normal_angle_cos)
      return (true);
    if (fabs(point_a.intensity - point_b.intensity) < thresh_intensity)
      return (true);
  }
  else
  {
    if (fabs(point_a.intensity - point_b.intensity) < thresh_intensity_l_dist)
      return (true);
  }
  return (false);
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


////---------------------------------------------------------------------------
//// Colors for visualize clusters
////---------------------------------------------------------------------------
float colors[6][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

////---------------------------------------------------------------------------
//// Constructor of tnp_grasp_planner
////---------------------------------------------------------------------------
GraspPlannerNode::GraspPlannerNode() :
    cloud_(new pcl::PointCloud<PointType>()), cloud_cluster_colored_(new pcl::PointCloud<PointType>())
    ///TODO the resolution of the containers trees is hardcoded to 0.01 [m]
        , bin_A_tree_(0.01), bin_B_tree_(0.01), bin_C_tree_(0.01), box_1_tree_(0.01), box_2_tree_(0.01), box_3_tree_(
        0.01), tote_tree_(0.01), amnesty_tree_(0.01)
{
  //default init

  // Initialization -- everything there what should be there? (1) internal components (2) external components

  // Setup -- stage 2

  // Get operative
  // Establish ros services
  setupROSConfiguration();
}

////---------------------------------------------------------------------------
//// sets up services to offer topics and subscribes to main topics
//// @return true, if every subscription worked.
////---------------------------------------------------------------------------
bool GraspPlannerNode::setupROSConfiguration()
{
  // Set parameters from launch file
  setParameters();

  //Topic you want to publish
  bin_A_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/bin_A_occupancy_map", 5);
  bin_B_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/bin_B_occupancy_map", 5);
  bin_C_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/bin_C_occupancy_map", 5);
  tote_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/tote_occupancy_map", 5);
  amnesty_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/amnesty_occupancy_map", 5);
  box_1_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/box_1_occupancy_map", 5);
  box_2_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/box_2_occupancy_map", 5);
  box_3_occupancy_map_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/box_3_occupancy_map", 5);
  suckable_surfaces_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/suckable_surfaces_map", 5);
  calibration_octomap_pub_ = n_.advertise<octomap_msgs::Octomap>("tnp_grasp_planner/calibration_occupancy_map", 5);

  //Topic you want to subscribe
  cloud_from_sr300_sub_ = n_.subscribe(cloud_topic_sr300_, 1, &GraspPlannerNode::CloudFromSR300Callback, this);
  cloud_from_phoxi_sub_ = n_.subscribe(cloud_topic_phoxi_, 1, &GraspPlannerNode::CloudFromPhoxiCallback, this); // Photoneo

  // services you want to advertise
  getSuctionCandidates_bb_srv_ = n_.advertiseService("tnp_grasp_planner/get_suction_candidates_bb",
                                                     &GraspPlannerNode::GetSuctionCandidatesBBCallback, this);
  getSuctionCandidates_container_srv_ = n_.advertiseService("tnp_grasp_planner/get_suction_candidates_container",
                                                            &GraspPlannerNode::GetSuctionCandidatesContainerCallback,
                                                            this);
  getGripCandidates_bb_srv_ = n_.advertiseService("tnp_grasp_planner/get_grip_candidates_bb",
                                                  &GraspPlannerNode::GetGripCandidatesBBCallback, this);
  getGripCandidates_container_srv_ = n_.advertiseService("tnp_grasp_planner/get_grip_candidates_container",
                                                         &GraspPlannerNode::GetGripCandidatesContainerCallback, this);
  getContainerOccupancy_srv_ = n_.advertiseService("tnp_grasp_planner/get_container_occupancy",
                                                   &GraspPlannerNode::GetContainerOccupancyCallback, this);
  savePCDFile_srv_ = n_.advertiseService("tnp_grasp_planner/save_pcd_file", &GraspPlannerNode::SavePCDFileCallback,
                                         this);
  getPoseToPlaceItem_srv_ = n_.advertiseService("tnp_grasp_planner/get_pose_to_place_item",
                                                &GraspPlannerNode::GetPoseToPlaceItemCallback, this);
  updateOccupancyMap_srv_ = n_.advertiseService("tnp_grasp_planner/update_occupancy_map",
                                                &GraspPlannerNode::UpdateOccupancyMapCallback, this);
  getSuctionCandidates_container_fromOctomap_srv_ = n_.advertiseService("tnp_grasp_planner/get_suction_candidates_container_from_octomap",
                                                &GraspPlannerNode::GetSuctionCandidatesContainerFromOctomapCallback, this);

  // services you want to subscribe to
  // Photoneo
  phoxi_camera_trigger_image_client_ = n_.serviceClient<phoxi_camera::TriggerImage>("/phoxi_camera/trigger_image");
  phoxi_camera_get_frame_client_ = n_.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");
  can_robot_place_item_here_client_ = n_.serviceClient<tnp_kuka_motion::canRobotPlaceItemHere>("/tnp_kuka_motion/canRobotPlaceItemHere");
  

  return true;
}

bool GraspPlannerNode::setParameters()
{
  n_.param<std::string>("tnp_grasp_planner/cloud_topic_sr300", cloud_topic_sr300_, "/camera/depth_registered/points");
  n_.param<std::string>("tnp_grasp_planner/cloud_topic_phoxi", cloud_topic_phoxi_, "/phoxi_camera/pointcloud");
  n_.param<std::string>("tnp_grasp_planner/pcd_directory", pcd_save_dir_, "/root/");
  n_.param<float>("tnp_grasp_planner/ne_param/search_radius", search_radius_, 0.1);
  n_.param<float>("tnp_grasp_planner/de_param/octree_resolution", octree_resolution_, 0.1);
  n_.param<float>("tnp_grasp_planner/pf_param/thresh_min_z", thresh_passthrough_minz_, 0.0);
  n_.param<float>("tnp_grasp_planner/pf_param/thresh_max_z", thresh_passthrough_maxz_, 2.0);
  n_.param<float>("tnp_grasp_planner/ec_param/cluster_tolerance", cluster_tolerance_, 0.1);
  n_.param<int>("tnp_grasp_planner/ec_param/min_cluster_size", min_cluster_size_, 100);
  n_.param<int>("tnp_grasp_planner/ec_param/max_cluster_size", max_cluster_size_, std::numeric_limits<int>::max());
  n_.param<float>("tnp_grasp_planner/cec_param/squared_distance", squared_distance_, 1.0);
  n_.param<float>("tnp_grasp_planner/cec_param/thresh_intensity", thresh_intensity_, 1.0);
  n_.param<float>("tnp_grasp_planner/cec_param/thresh_normal_angle_deg", thresh_normal_angle_deg_, 90.0);
  n_.param<float>("tnp_grasp_planner/cec_param/thresh_curvature", thresh_curvature_, 0.0);
  n_.param<float>("tnp_grasp_planner/cec_param/thresh_intensity_l_dist", thresh_intensity_l_dist_, 1.0);
  n_.param<float>("tnp_grasp_planner/ne_param/search_radius_suck", search_radius_suck_, 0.1);
  n_.param<float>("tnp_grasp_planner/occu_param/grid_size", occu_grid_size_, 0.01);
  n_.param<bool>("tnp_grasp_planner/tnp_debug", tnp_debug_, false);

  ///////////////////////////////////////////////////////////////////////////////
  // Occupancy maps parameters
  ///////////////////////////////////////////////////////////////////////////////

  // reference frame
  n_.param<std::string>("tnp_grasp_planner/occu_param/depth_camera_frame", depth_camera_frame_, "tnp_ee_depth_camera_frame");
  // threshold to determine if a voxel is occupied or free (based on its occupancy probability)
  n_.param<float>("tnp_grasp_planner/occu_param/occupied_voxel_threshold", occupied_voxel_threshold_, 0.6);
  // this is a virtual/real division in the bin A measured from the origin of bin_A (in local coordinates) in y
  n_.param<float>("tnp_grasp_planner/bin_A_virtual_division_in_y", bin_A_virtual_division_in_y_, 0.5);
  // start searching for a free spot from this z (container coordinates)
  n_.param<float>("tnp_grasp_planner/start_searching_free_spot_from_this_z", start_searching_free_spot_from_this_z_, 0.01);

  // containers parameters
  // bin_A
  n_.getParam("tnp_environment/bin_A_w", bin_A_w_); // in local x
  n_.getParam("tnp_environment/bin_A_l", bin_A_l_); // in local y
  n_.getParam("tnp_environment/bin_A_h", bin_A_h_); // in z
  n_.getParam("tnp_environment/bin_A_x", bin_A_x_); // origin in global x
  n_.getParam("tnp_environment/bin_A_y", bin_A_y_); // origin in global y
  n_.getParam("tnp_environment/bin_A_z", bin_A_z_); // origin in z

  // bin_B
  n_.getParam("tnp_environment/bin_B_w", bin_B_w_); // in local x
  n_.getParam("tnp_environment/bin_B_l", bin_B_l_); // in local y
  n_.getParam("tnp_environment/bin_B_h", bin_B_h_); // in z
  n_.getParam("tnp_environment/bin_B_x", bin_B_x_); // origin in global x
  n_.getParam("tnp_environment/bin_B_y", bin_B_y_); // origin in global y
  n_.getParam("tnp_environment/bin_B_z", bin_B_z_); // origin in z

  // bin_C
  n_.getParam("tnp_environment/bin_C_w", bin_C_w_); // in local x
  n_.getParam("tnp_environment/bin_C_l", bin_C_l_); // in local y
  n_.getParam("tnp_environment/bin_C_h", bin_C_h_); // in z
  n_.getParam("tnp_environment/bin_C_x", bin_C_x_); // origin in global x
  n_.getParam("tnp_environment/bin_C_y", bin_C_y_); // origin in global y
  n_.getParam("tnp_environment/bin_C_z", bin_C_z_); // origin in z

  // tote
  n_.getParam("tnp_environment/tote_w", tote_w_); // in local x
  n_.getParam("tnp_environment/tote_l", tote_l_); // in local y
  n_.getParam("tnp_environment/tote_h", tote_h_); // in z
  n_.getParam("tnp_environment/tote_x", tote_x_); // origin in global x
  n_.getParam("tnp_environment/tote_y", tote_y_); // origin in global y
  n_.getParam("tnp_environment/tote_z", tote_z_); // origin in z

  // amnesty
  n_.getParam("tnp_environment/amnesty_w", amnesty_w_); // in local x
  n_.getParam("tnp_environment/amnesty_l", amnesty_l_); // in local y
  n_.getParam("tnp_environment/amnesty_h", amnesty_h_); // in z
  n_.getParam("tnp_environment/amnesty_x", amnesty_x_); // origin in global x
  n_.getParam("tnp_environment/amnesty_y", amnesty_y_); // origin in global y
  n_.getParam("tnp_environment/amnesty_z", amnesty_z_); // origin in z

  /// FIXME there are no defaults
  // box_1
  n_.getParam("tnp_environment/box_1_w", box_1_w_); // in local x
  n_.getParam("tnp_environment/box_1_l", box_1_l_); // in local y
  n_.getParam("tnp_environment/box_1_h", box_1_h_); // in z
  n_.getParam("tnp_environment/box_1_x", box_1_x_); // origin in global x
  n_.getParam("tnp_environment/box_1_y", box_1_y_); // origin in global y
  n_.getParam("tnp_environment/box_1_z", box_1_z_); // origin in z

  // box_2
  n_.getParam("tnp_environment/box_2_w", box_2_w_); // in local x
  n_.getParam("tnp_environment/box_2_l", box_2_l_); // in local y
  n_.getParam("tnp_environment/box_2_h", box_2_h_); // in z
  n_.getParam("tnp_environment/box_2_x", box_2_x_); // origin in global x
  n_.getParam("tnp_environment/box_2_y", box_2_y_); // origin in global y
  n_.getParam("tnp_environment/box_2_z", box_2_z_); // origin in z

  // box_3
  n_.getParam("tnp_environment/box_3_w", box_3_w_); // in local x
  n_.getParam("tnp_environment/box_3_l", box_3_l_); // in local y
  n_.getParam("tnp_environment/box_3_h", box_3_h_); // in z
  n_.getParam("tnp_environment/box_3_x", box_3_x_); // origin in global x
  n_.getParam("tnp_environment/box_3_y", box_3_y_); // origin in global y
  n_.getParam("tnp_environment/box_3_z", box_3_z_); // origin in z

  return true;
}

////---------------------------------------------------------------------------
//// Helpers
////---------------------------------------------------------------------------

octomap::Pointcloud GraspPlannerNode::GetOctomapCloudFromSR300()
{
  std::lock_guard<std::mutex> lock(octomap_cloud_from_sr300_mutex_);
  return octomap_cloud_from_sr300_;
}

void GraspPlannerNode::SetOctomapCloudFromSR300(const octomap::Pointcloud& new_cloud)
{
  std::lock_guard<std::mutex> lock(octomap_cloud_from_sr300_mutex_);
  octomap_cloud_from_sr300_ = new_cloud;
}

octomap::Pointcloud GraspPlannerNode::GetOctomapCloudFromPhoxi()
{
  std::lock_guard<std::mutex> lock(octomap_cloud_from_phoxi_mutex_);
  return octomap_cloud_from_phoxi_;
}

void GraspPlannerNode::SetOctomapCloudFromPhoxi(const octomap::Pointcloud& new_cloud)
{
  std::lock_guard<std::mutex> lock(octomap_cloud_from_phoxi_mutex_);
  octomap_cloud_from_phoxi_ = new_cloud;
}

bool GraspPlannerNode::TriggerPhoxiCamera()
{
  phoxi_camera::TriggerImage srv1;
  if (phoxi_camera_trigger_image_client_.call(srv1))
  {
    ROS_INFO("/phoxi_camera/trigger_image is being called");
    ROS_WARN("[TNP_STATE L1] Executing /phoxi_camera/trigger_image Executing the service");
  }
  else
  {
    ROS_WARN("[TNP_STATE L2] Error /phoxi_camera/trigger_image There was an error calling the service");
    ROS_ERROR("Failed to call service /phoxi_camera/trigger_image");
    return false;
  }

  ros::Duration(1.0).sleep(); //TODO its too long to wait!!

  phoxi_camera::GetFrame srv2;
  srv2.request.in = -1;
  ROS_WARN("The Photoneo may hang"); ///TODO photoneo may hang
  if (phoxi_camera_get_frame_client_.call(srv2))
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
  return true;
}

bool GraspPlannerNode::canRobotPlaceItemHere(geometry_msgs::Pose target_pose, std::string container_name, bool use_gripper_EE)
{
  double dist = sqrt(target_pose.position.x * target_pose.position.x + target_pose.position.y * target_pose.position.y);
  if (dist > .70)
  {
    return false;
  }
  return true;
}


////---------------------------------------------------------------------------
//// Service definitions
////---------------------------------------------------------------------------

void generateBoxesFromOctomap(std::vector<fcl::CollisionObject*>& boxes, fcl::OcTree& tree)
{

  std::vector<std::array<fcl::FCL_REAL, 6> > boxes_ = tree.toBoxes();
  for (std::size_t i = 0; i < boxes_.size(); ++i)
  {
    fcl::FCL_REAL x = boxes_[i][0];
    fcl::FCL_REAL y = boxes_[i][1];
    fcl::FCL_REAL z = boxes_[i][2];
    fcl::FCL_REAL size = boxes_[i][3];
    fcl::FCL_REAL cost = boxes_[i][4];
    fcl::FCL_REAL threshold = boxes_[i][5];
    fcl::Box* box = new fcl::Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject* obj = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                                         fcl::Transform3f(fcl::Vec3f(x, y, z)));
    boxes.push_back(obj);
  }
}

/// This function is a slightly modified version of the one in t from the test_fcl_utility.h from FCL library
/// @brief Collision data stores the collision request and the result given by collision algorithm.
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  fcl::CollisionRequest request;

  /// @brief Collision result
  fcl::CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;
};

//==============================================================================
/// This function is a slightly modified version of the one in t from the test_fcl_utility.h from FCL library
bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
  auto* cdata = static_cast<CollisionData*>(cdata_);
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if (cdata->done)
    return true;

  collide(o1, o2, request, result);

  if (!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

bool GraspPlannerNode::FillOccupancyMap(octomap::OcTree* saved_occupancy_map_tree, const float& container_h)
{
  // fill the map according to what is occluded and should be considered as occupied
  ROS_WARN("Filling map...");
  for (octomap::OcTree::tree_iterator it = saved_occupancy_map_tree->begin_tree(), end =
      saved_occupancy_map_tree->end_tree(); it != end; ++it)
  {
    const octomap::OcTreeKey key = it.getKey();
    const octomap::point3d curr_coord = saved_occupancy_map_tree->keyToCoord(key);
    ROS_DEBUG_STREAM("curr_coord "<<curr_coord);

    if (it->getOccupancy() > occupied_voxel_threshold_) // considered occupied if greater than threshold
    {
      octomap::point3d below_curr_coord(curr_coord.x(), curr_coord.y(), curr_coord.z() - 0.01);
      while (below_curr_coord.z() > 0.005)
      {
        if (curr_coord.z() >= 0.0 && curr_coord.z() <= container_h)
        {
          saved_occupancy_map_tree->updateNode(below_curr_coord, true);
          below_curr_coord.z() -= 0.01;
        }
        else
        {
          break;
        }
      }
    }
  }
  ROS_INFO("Filling map... done");

  return true;
}

bool GraspPlannerNode::UpdateOccupancyMapCallback(tnp_grasp_planner::updateOccupancyMap::Request &req,
                                                  tnp_grasp_planner::updateOccupancyMap::Response &res)
{
  ROS_INFO("UpdateOccupancyMapCallback was called");

  // set the corresponding occupancy maps and container size
  octomap::OcTree* saved_occupancy_map_tree;
  ros::Publisher* container_occupancy_map_pub;
  float container_w = 0.0, container_l = 0.0, container_h = 0.0;
  if (req.container_id.data.compare("bin_A") == 0)
  {
    saved_occupancy_map_tree = &bin_A_tree_;
    container_occupancy_map_pub = &bin_A_occupancy_map_pub_;
    container_w = bin_A_w_;
    container_l = bin_A_l_;
    container_h = bin_A_h_;
  }
  else if (req.container_id.data.compare("bin_B") == 0)
  {
    saved_occupancy_map_tree = &bin_B_tree_;
    container_occupancy_map_pub = &bin_B_occupancy_map_pub_;
    container_w = bin_B_w_;
    container_l = bin_B_l_;
    container_h = bin_B_h_;
  }
  else if (req.container_id.data.compare("bin_C") == 0)
  {
    saved_occupancy_map_tree = &bin_C_tree_;
    container_occupancy_map_pub = &bin_C_occupancy_map_pub_;
    container_w = bin_C_w_;
    container_l = bin_C_l_;
    container_h = bin_C_h_;
  }
  else if (req.container_id.data.compare("box_1") == 0)
  {
    saved_occupancy_map_tree = &box_1_tree_;
    container_occupancy_map_pub = &box_1_occupancy_map_pub_;
    container_w = box_1_w_;
    container_l = box_1_l_;
    container_h = box_1_h_;
  }
  else if (req.container_id.data.compare("box_2") == 0)
  {
    saved_occupancy_map_tree = &box_2_tree_;
    container_occupancy_map_pub = &box_2_occupancy_map_pub_;
    container_w = box_2_w_;
    container_l = box_2_l_;
    container_h = box_2_h_;
  }
  else if (req.container_id.data.compare("box_3") == 0)
  {
    saved_occupancy_map_tree = &box_3_tree_;
    container_occupancy_map_pub = &box_3_occupancy_map_pub_;
    container_w = box_3_w_;
    container_l = box_3_l_;
    container_h = box_3_h_;
  }
  else if (req.container_id.data.compare("tote") == 0)
  {
    saved_occupancy_map_tree = &tote_tree_;
    container_occupancy_map_pub = &tote_occupancy_map_pub_;
    container_w = tote_w_;
    container_l = tote_l_;
    container_h = tote_h_;
  }
  else if (req.container_id.data.compare("amnesty") == 0)
  {
    saved_occupancy_map_tree = &amnesty_tree_;
    container_occupancy_map_pub = &amnesty_occupancy_map_pub_;
    container_w = amnesty_w_;
    container_l = amnesty_l_;
    container_h = amnesty_h_;
  }
  else
  {
    ROS_ERROR_STREAM("UpdateOccupancyMapCallback: Unknown container id: " << req.container_id.data);
    return false;
  }

  ROS_DEBUG_STREAM("container_w " << container_w);
  ROS_DEBUG_STREAM("container_h " << container_h);
  ROS_DEBUG_STREAM("container_l " << container_l);
  
  // get the current point cloud
  octomap::Pointcloud observed_pointcloud = GetOctomapCloudFromSR300(); // this is depth_camera_frame (see the parameters)

  // transform it to container coordinates
  tf::StampedTransform transform;
  try
  {
    const ros::Time t = ros::Time::now();
    tf_listener_.waitForTransform(req.container_id.data, depth_camera_frame_, t, ros::Duration(3.0));
    tf_listener_.lookupTransform(req.container_id.data, depth_camera_frame_, t, transform);
  }
  catch (const tf::ExtrapolationException &e)
  {
    ROS_ERROR_STREAM("UpdateOccupancyMapCallback: transform couldn't be obtained");
    ROS_ERROR_STREAM("Error: " << e.what());
    return false;
  }

  const tf::Vector3 tf_trans = transform.getOrigin();
  const tf::Quaternion tf_rot = transform.getRotation();
  const octomath::Vector3 octo_trans(tf_trans.x(), tf_trans.y(), tf_trans.z());
  const octomath::Quaternion octo_rot(tf_rot.getW(), tf_rot.getX(), tf_rot.getY(), tf_rot.getZ());
  const octomath::Pose6D transform_camera_to_container(octo_trans, octo_rot);
  observed_pointcloud.transform(transform_camera_to_container);

  // clear saved map
  saved_occupancy_map_tree->clear();

  // generate the tree from the point cloud
  const octomap::point3d sensor_origin(0.0, 0.0, 0.0); //not sure what effect this has
  saved_occupancy_map_tree->insertPointCloud(observed_pointcloud, sensor_origin);
  if(tnp_debug_)
  {
    /*debug*/octomap_msgs::Octomap bmap_msg;
    /*debug*/bmap_msg.header.stamp = ros::Time(0); // seems to solve the issue of not appearing in rviz
    /*debug*/bmap_msg.header.frame_id = req.container_id.data;
    /*debug*/octomap_msgs::binaryMapToMsg(*saved_occupancy_map_tree, bmap_msg);
    /*debug*/calibration_octomap_pub_.publish(bmap_msg);
  }

  // clean the map by getting rid of the nodes outside the container
  ROS_INFO("Pruning tree...");
  ROS_INFO_STREAM("original tree size is " << saved_occupancy_map_tree->size());
  for (octomap::OcTree::tree_iterator it = saved_occupancy_map_tree->begin_tree(), end =
      saved_occupancy_map_tree->end_tree(); it != end; ++it)
  {
    octomap::point3d curr_node = it.getCoordinate();
    if (curr_node.x() < 0.0 || curr_node.x() > container_w || curr_node.y() < 0.0 || curr_node.y() > container_l
        || curr_node.z() < 0.0 || curr_node.z() > (container_h + 0.05)) //added a margin for protruding items
    {
      saved_occupancy_map_tree->deleteNode(curr_node);
    }
  }
  ROS_INFO_STREAM("new tree size is " << saved_occupancy_map_tree->size());
  ROS_INFO("Pruning done");

  saved_occupancy_map_tree->updateInnerOccupancy(); // TODO necessary?

  // generate the map message to publish it and see it in rviz
  octomap_msgs::Octomap bmap_msg;
  bmap_msg.header.stamp = ros::Time(0); // seems to solve the issue of not appearing in rviz
  bmap_msg.header.frame_id = req.container_id.data;
  octomap_msgs::binaryMapToMsg(*saved_occupancy_map_tree, bmap_msg);
  container_occupancy_map_pub->publish(bmap_msg);

  return true;
}

bool GraspPlannerNode::GetPoseToPlaceItemCallback(tnp_grasp_planner::getPoseToPlaceItem::Request &req,
                                                  tnp_grasp_planner::getPoseToPlaceItem::Response &res)
{
  ROS_INFO("GetPoseToPlaceItemCallback was called");

  // set the corresponding occupancy maps and container size
  // TODO refactor this part cause it's repeated in the UpdateOccupancyMapCallback
  octomap::OcTree* saved_occupancy_map_tree;
  ros::Publisher* container_occupancy_map_pub;
  float container_w = 0.0, container_l = 0.0, container_h = 0.0;

  // section_num applies only to bin A (sections are separated by bin_A_virtual_division_in_y)
  // 0: no section specified (consider the whole bin)
  // 1: section 1 (the closest to the origin of the bin)
  // 2: section 2 (the farthest from the origin of the bin)
  int section_num = req.section_num;

  if (req.container_id.data.compare("bin_A") == 0)
  {
    saved_occupancy_map_tree = &bin_A_tree_;
    container_occupancy_map_pub = &bin_A_occupancy_map_pub_;
    container_w = bin_A_w_;
    container_l = bin_A_l_;
    container_h = bin_A_h_;
  }
  else if (req.container_id.data.compare("bin_B") == 0)
  {
    saved_occupancy_map_tree = &bin_B_tree_;
    container_occupancy_map_pub = &bin_B_occupancy_map_pub_;
    container_w = bin_B_w_;
    container_l = bin_B_l_;
    container_h = bin_B_h_;
  }
  else if (req.container_id.data.compare("bin_C") == 0)
  {
    saved_occupancy_map_tree = &bin_C_tree_;
    container_occupancy_map_pub = &bin_C_occupancy_map_pub_;
    container_w = bin_C_w_;
    container_l = bin_C_l_;
    container_h = bin_C_h_;
  }
  else if (req.container_id.data.compare("box_1") == 0)
  {
    saved_occupancy_map_tree = &box_1_tree_;
    container_occupancy_map_pub = &box_1_occupancy_map_pub_;
    container_w = box_1_w_;
    container_l = box_1_l_;
    container_h = box_1_h_;
  }
  else if (req.container_id.data.compare("box_2") == 0)
  {
    saved_occupancy_map_tree = &box_2_tree_;
    container_occupancy_map_pub = &box_2_occupancy_map_pub_;
    container_w = box_2_w_;
    container_l = box_2_l_;
    container_h = box_2_h_;
  }
  else if (req.container_id.data.compare("box_3") == 0)
  {
    saved_occupancy_map_tree = &box_3_tree_;
    container_occupancy_map_pub = &box_3_occupancy_map_pub_;
    container_w = box_3_w_;
    container_l = box_3_l_;
    container_h = box_3_h_;
  }
  else if (req.container_id.data.compare("tote") == 0)
  {
    saved_occupancy_map_tree = &tote_tree_;
    container_occupancy_map_pub = &tote_occupancy_map_pub_;
    container_w = tote_w_;
    container_l = tote_l_;
    container_h = tote_h_;
  }
  else if (req.container_id.data.compare("amnesty") == 0)
  {
    saved_occupancy_map_tree = &amnesty_tree_;
    container_occupancy_map_pub = &amnesty_occupancy_map_pub_;
    container_w = amnesty_w_;
    container_l = amnesty_l_;
    container_h = amnesty_h_;
  }
  else
  {
    ROS_ERROR_STREAM("GetPoseToPlaceItemCallback: Unknown container id: " << req.container_id.data);
    return false;
  }
  /*debug*/ROS_INFO_STREAM("Tree size " << saved_occupancy_map_tree->size());

  ROS_INFO_STREAM("container_w " << container_w);
  ROS_INFO_STREAM("container_h " << container_h);
  ROS_INFO_STREAM("container_l " << container_l);

  float item_width = req.item_width < 0.14 ? 0.14 : req.item_width; // m
  float item_length = req.item_length < 0.14 ? 0.14 : req.item_length; // m
  float item_height = req.item_height > 0.14 ? 0.14 : req.item_height; // m

  // Adjust the item dimensions to be a cube of side length = max item dimension
  double max_dimension = .15;
  if ((item_width >= item_length) && (item_width >= item_height))
  {
    max_dimension = item_width;
  }
  else if ((item_length >= item_width) && (item_length >= item_height))
  {
    max_dimension = item_length;
  }
  else if ((item_height >= item_width) && (item_height >= item_length))
  {
    max_dimension = item_height;
  }
  item_width = max_dimension;
  item_length = max_dimension;
  item_height = max_dimension;
  ROS_INFO_STREAM("UpdateOccupancyMap: item_width " << item_width);
  ROS_INFO_STREAM("UpdateOccupancyMap: item_length " << item_length);
  ROS_INFO_STREAM("UpdateOccupancyMap: item_height " << item_height);
  ROS_INFO_STREAM("UpdateOccupancyMap: container " << req.container_id.data << ", section num " << section_num);

  if ((item_width == 0) || (item_length == 0) || (item_height == 0))
  {
    ROS_WARN("Received item dimensions zero. Adjusting to default item size.");
    item_width = .15;
    item_length = .15;
    item_height = .05;
  }

  // Initialize to the center of the container in case our algorithm fails
  ROS_WARN_STREAM("Debug pose planning: " << (rand() % 20) / 100.0 << ", sign: " << ( (rand() % 1) ? 1 : -1));
  float free_spot_x = container_w / 2.0 + container_w * (rand() % 20) / 100.0  *  ( (rand() % 1) ? 1 : -1);
  float free_spot_y = container_l / 2.0 + container_l * (rand() % 20) / 100.0  *  ( (rand() % 1) ? 1 : -1);
  float free_spot_z = item_height / 2.0;
  float yaw = 0.0;
  bool free_spot_found = false;
  bool generate_default_pose_for_sections = false;
  double safety_margin = .025;

  // Look for a free spot if the tree is not empty, otherwise return the center of the container and above it (see free_spot_z)
  if(saved_occupancy_map_tree->size() > 0)
  {
    // convert the octomap::octree to fcl::octree fcl_octree object
    fcl::OcTree *fcl_tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(saved_occupancy_map_tree));

    // define a transform to move the item box
    fcl::Transform3f tf0;
    // Check quaternion validity
    if ((req.bounding_box_center_pose.pose.orientation.x < 1e-6) && 
        (req.bounding_box_center_pose.pose.orientation.y < 1e-6) && 
        (req.bounding_box_center_pose.pose.orientation.z < 1e-6) && 
        (req.bounding_box_center_pose.pose.orientation.w < 1e-6))
    {
      ROS_WARN("Received an invalid quaternion in grasp planner. Setting to identity.");
      tf0.setQuatRotation(
        fcl::Quaternion3f(0.0,
                          0.0,
                          0.0,
                          1.0));
    }
    else
    {
      tf0.setQuatRotation(
        fcl::Quaternion3f(req.bounding_box_center_pose.pose.orientation.w,
                          req.bounding_box_center_pose.pose.orientation.x,
                          req.bounding_box_center_pose.pose.orientation.y,
                          req.bounding_box_center_pose.pose.orientation.z));  
    }
    

    // convert the occupancy map to a vector of boxes to find collisions between each box and the item
    std::vector<fcl::CollisionObject *> boxes;
    generateBoxesFromOctomap(boxes, *fcl_tree);

    // Initialize a collision manager for the group of boxes (the occupancy map).
    // FCL provides various different implementations of CollisionManager.
    // Generally, the DynamicAABBTreeCollisionManager would provide the best performance.
    fcl::BroadPhaseCollisionManager *manager1 = new fcl::DynamicAABBTreeCollisionManager();
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      manager1->registerObject(boxes[i]);
    }
    // Setup the manager, which is related to initializing the broadphase acceleration structure according to objects input
    manager1->setup();
    
    for (int orientation = 0; orientation < 2; ++orientation) 
    {
      float swappable_width = item_width;
      float swappable_length = item_length;
      if(orientation==1) {
        swappable_width = item_length;
        swappable_length = item_width;
        yaw = -M_PI/2.0;
      }
      // create a box to represent the item
      std::shared_ptr<fcl::Box> item_box(new fcl::Box(swappable_width, swappable_length, item_height));
      
      ROS_INFO("Searching for a free spot to place the item...");

      bool break_for_z = false;
      bool break_for_y = false;


      float start_search_from_this_y = safety_margin;
      float finish_search_at_this_y = container_l - safety_margin;

      if (section_num == 1) {
        // finish_search_at_this_y = bin_A_virtual_division_in_y_;
        finish_search_at_this_y = .35;
      }
      else if (section_num == 2) {
        start_search_from_this_y = .55;
      }
      else {
        // search in the whole container
      }

      /// NOTE the fcl::box origin is in the center of the box
      double step_size = 0.01;
      for (float z = start_searching_free_spot_from_this_z_ + item_height / 2.0; z < (container_h - item_height / 2.0) + 0.05; z += step_size) {
        for (float y = start_search_from_this_y + item_length / 2.0; y < (finish_search_at_this_y - item_length / 2.0); y += step_size) {
          for (float x = 0.01 + item_width / 2.0; x < (container_w - item_width / 2.0); x += step_size) {
            // change the position of the item to test if it collides
            tf0.setTranslation(fcl::Vec3f(x, y, z));
            fcl::CollisionObject co0(item_box, tf0);
            
            // Collision query between the item and the entire occupancy map
            CollisionData collision_data;
            manager1->collide(&co0, &collision_data, defaultCollisionFunction);
            

            if (!collision_data.result.isCollision())
            {
              // Create pose to check with kuka_motion if it is feasible
              geometry_msgs::PoseStamped target_pose, target_pose_in_world;
              target_pose.header.frame_id = req.container_id.data;
              target_pose.pose.position.x = x;
              target_pose.pose.position.y = y;
              target_pose.pose.position.z = z;
              target_pose.pose.orientation.w = 1.0;
              target_pose_in_world = transform_pose_now(target_pose, "iiwa_link_0", tf_listener_);
              ROS_INFO_STREAM("Debug 2 ");
              if (canRobotPlaceItemHere(target_pose_in_world.pose, req.container_id.data, 0))
              {
                ROS_INFO_STREAM("free spot for item found: " << x << ", " << y << ", " << z); ///TODO add comment of the frame that this is in
                free_spot_x = x;                                                              //These two lines assume that the object is centered under the end effector frame
                free_spot_y = y;
                free_spot_z = z;
                free_spot_found = true;
                break_for_y = true;
                break_for_z = true;
                break;
              }
            }
          }
          if (break_for_y)
            break;
        }
        if (break_for_z)
          break;
      }
    }
    ROS_INFO("Done");

    if (!free_spot_found)
    {
      ROS_ERROR("No free spot found, sending a default pose (corner of the container and 14cm above the floor)");
      // Make it an appropriate default pose for the sections
      generate_default_pose_for_sections = true;
    }

  }
  else // saved tree is empty
  {
    if( (item_length+0.02) < container_l) {
      free_spot_x = item_width/2.0 + 0.02; //These two lines assume that the object is centered under the end effector frame
      free_spot_y = item_length/2.0 + 0.01;
      free_spot_z = item_height/2.0;
    }
    else
    {
      free_spot_x = item_length/2.0 + 0.01; //These two lines assume that the object is centered under the end effector frame
      free_spot_y = item_width/2.0 + 0.02;
      free_spot_z = item_height/2.0;
    }
    ROS_WARN("Saved tree is empty, returning a default pose in the container");

    //return a default pose as the tree is empty
    free_spot_found = true;
    generate_default_pose_for_sections = true;
  }

  if (generate_default_pose_for_sections)
  {
    ROS_WARN_STREAM("Generating default pose for section: " << section_num);
    // Make it an appropriate default pose for the sections
    if( section_num == 1 )
    {
      free_spot_x = container_w / 2.0;
      free_spot_y = container_l * .2;
      free_spot_z = item_height / 2.0;
    }
    else if( section_num == 2 )
    {
      free_spot_x = container_w / 2.0;
      free_spot_y = container_l * .8;
      free_spot_z = item_height / 2.0;
    }
  }

  ////// Add the item at the free spot to the map
  // discretize the item in cm voxels
  const int item_width_voxels = ceil(item_width / 0.01);
  const int item_length_voxels = ceil(item_length / 0.01);
  const int item_height_voxels = ceil(item_height / 0.01);
  // for loop with the voxels
  for (int x_voxel = 0; x_voxel < item_width_voxels; ++x_voxel)
  {
    for (int y_voxel = 0; y_voxel < item_length_voxels; ++y_voxel)
    {
      for (int z_voxel = 0; z_voxel < item_height_voxels; ++z_voxel)
      {
        octomap::point3d aux_point((float)free_spot_x - (item_width/2.0) + (x_voxel * 0.01) + 0.005f,
                                   (float)free_spot_y - (item_length/2.0) + (y_voxel * 0.01) + 0.005f,
                                   (float)free_spot_z - (item_height/2.0) + (z_voxel * 0.01) + 0.005f);
        saved_occupancy_map_tree->updateNode(aux_point, true); // integrate 'occupied' measurement
      }
    }
  }

  
  ///// Generate the map message to publish it and see it in rviz
  octomap_msgs::Octomap bmap_msg;
  bmap_msg.header.stamp = ros::Time(0); // seems to solve the issue of not appearing in rviz
  bmap_msg.header.frame_id = req.container_id.data;
  octomap_msgs::binaryMapToMsg(*saved_occupancy_map_tree, bmap_msg);
  container_occupancy_map_pub->publish(bmap_msg);

  // output pose in container coordinates
  res.pose_to_place_item_in_container_coords.header.stamp = ros::Time::now();
  res.pose_to_place_item_in_container_coords.header.frame_id = req.container_id.data;
  res.pose_to_place_item_in_container_coords.pose.position.x = free_spot_x + 0.025; //extra space for safety
  res.pose_to_place_item_in_container_coords.pose.position.y = free_spot_y + 0.025; //extra space for safety
  res.pose_to_place_item_in_container_coords.pose.position.z = free_spot_z + item_height/2.0 +0.01; // extra space for safety
  if (res.pose_to_place_item_in_container_coords.pose.position.z < .12) 
  {
    ROS_WARN_STREAM("Adjusting default pose higher, it's too scary this low: " << res.pose_to_place_item_in_container_coords.pose.position.z);
    res.pose_to_place_item_in_container_coords.pose.position.z = .12;
  }
  //TODO add item rotation
  res.pose_to_place_item_in_container_coords.pose.orientation.w = 1.0;
  res.pose_to_place_item_in_container_coords.pose.orientation.x = 0.0;
  res.pose_to_place_item_in_container_coords.pose.orientation.y = 0.0;
  res.pose_to_place_item_in_container_coords.pose.orientation.z = 0.0;
  res.free_spot_found = free_spot_found;

  ROS_INFO_STREAM("GetPoseToPlaceItem: Sending pose to place item \n" << res.pose_to_place_item_in_container_coords.pose.position);

  return true;
}

inline bool ComparePoseByZvalue(geometry_msgs::PoseStamped i, geometry_msgs::PoseStamped j)
{
  return (i.pose.position.z > j.pose.position.z);
}

// insert elements in order to have them sorted
void insert_sorted( std::vector<geometry_msgs::PoseStamped> &sorted_vec, const geometry_msgs::PoseStamped& new_elem ) {
    std::vector<geometry_msgs::PoseStamped>::iterator it = std::lower_bound( sorted_vec.begin(), sorted_vec.end(), new_elem, ComparePoseByZvalue ); // find proper position in descending order
    sorted_vec.insert( it, new_elem ); // insert before iterator it
}


bool GraspPlannerNode::GetSuctionCandidatesContainerFromOctomapCallback(
    tnp_grasp_planner::getSuctionCandidates_container_fromOctomap::Request &req,
    tnp_grasp_planner::getSuctionCandidates_container_fromOctomap::Response &res)
{
  ROS_INFO("GetSuctionCandidatesContainerFromOctomapCallback was called");

  // set the corresponding occupancy maps and container size
  // TODO refactor this part cause it's repeated in the UpdateOccupancyMapCallback
  octomap::OcTree* saved_occupancy_map_tree;
  float container_w = 0.0, container_l = 0.0, container_h = 0.0;

  // section_num applies only to bin A (sections are separated by bin_A_virtual_division_in_y)
  // 0: no section specified (consider the whole bin)
  // 1: section 1 (the closest to the origin of the bin)
  // 2: section 2 (the farthest from the origin of the bin)
  int section_num = 0;

  if (req.container_id.data.compare("bin_A") == 0)
  {
    saved_occupancy_map_tree = &bin_A_tree_;
    container_w = bin_A_w_;
    container_l = bin_A_l_;
    container_h = bin_A_h_;
    section_num = req.section_num; //this is the only bin with more than one section
    if(section_num != 1 && section_num != 2) section_num = 0; //keep the default if wrong section number is provided (i.e., consider the whole bin)
  }
  else if (req.container_id.data.compare("bin_B") == 0)
  {
    saved_occupancy_map_tree = &bin_B_tree_;
    container_w = bin_B_w_;
    container_l = bin_B_l_;
    container_h = bin_B_h_;
  }
  else if (req.container_id.data.compare("bin_C") == 0)
  {
    saved_occupancy_map_tree = &bin_C_tree_;
    container_w = bin_C_w_;
    container_l = bin_C_l_;
    container_h = bin_C_h_;
  }
  else if (req.container_id.data.compare("box_1") == 0)
  {
    saved_occupancy_map_tree = &box_1_tree_;
    container_w = box_1_w_;
    container_l = box_1_l_;
    container_h = box_1_h_;
  }
  else if (req.container_id.data.compare("box_2") == 0)
  {
    saved_occupancy_map_tree = &box_2_tree_;
    container_w = box_2_w_;
    container_l = box_2_l_;
    container_h = box_2_h_;
  }
  else if (req.container_id.data.compare("box_3") == 0)
  {
    saved_occupancy_map_tree = &box_3_tree_;
    container_w = box_3_w_;
    container_l = box_3_l_;
    container_h = box_3_h_;
  }
  else if (req.container_id.data.compare("tote") == 0)
  {
    saved_occupancy_map_tree = &tote_tree_;
    container_w = tote_w_;
    container_l = tote_l_;
    container_h = tote_h_;
  }
  else if (req.container_id.data.compare("amnesty") == 0)
  {
    saved_occupancy_map_tree = &amnesty_tree_;
    container_w = amnesty_w_;
    container_l = amnesty_l_;
    container_h = amnesty_h_;
  }
  else
  {
    ROS_ERROR_STREAM("GetSuctionCandidatesContainerFromOctomapCallback: Unknown container id: " << req.container_id.data);
    return false;
  }
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: container_w " << container_w);
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: container_l " << container_l);
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: container_h " << container_h);
  /*debug*/ROS_INFO_STREAM("Tree size " << saved_occupancy_map_tree->size());

  // 3D-template matcher

  // define the template size
  const float template_width = 0.03; // m
  const float template_length = 0.03; // m
  const float template_height = 0.01; // m
  const float template_num_of_voxels = std::ceil(template_width*template_length*template_height/(0.01*0.01*0.01));
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: template_width " << template_width);
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: template_length " << template_length);
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: template_height " << template_height);
  ROS_INFO_STREAM("GetSuctionCandidatesFromOctomap: template_num_of_voxels " << template_num_of_voxels);

  // convert the octomap::octree to fcl::octree fcl_octree object
  fcl::OcTree* fcl_tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(saved_occupancy_map_tree));

  // convert the occupancy map to a vector of boxes to find collisions between each box and the item
  std::vector<fcl::CollisionObject*> boxes;
  generateBoxesFromOctomap(boxes, *fcl_tree);

  // Initialize a collision manager for the group of boxes (the occupancy map).
  // FCL provides various different implementations of CollisionManager.
  // Generally, the DynamicAABBTreeCollisionManager would provide the best performance.
  fcl::BroadPhaseCollisionManager* manager1 = new fcl::DynamicAABBTreeCollisionManager();
  for (std::size_t i = 0; i < boxes.size(); ++i)
  {
    manager1->registerObject(boxes[i]);
  }
  // Setup the manager, which is related with initializing the broadphase acceleration structure according to objects input
  manager1->setup();

  ROS_INFO("Searching for a suckable surface (3D-template matching)...");

  bool suckable_surface_found = false;
  float start_search_from_this_y = 0.01;
  float finish_search_at_this_y = container_l;

  /// NOTE this only applies for bin_A
  if(section_num == 1)
  {
    finish_search_at_this_y = bin_A_virtual_division_in_y_;
  }
  else if(section_num == 2)
  {
    // start_search_from_this_y = bin_A_virtual_division_in_y_;
  }
  else
  {
    // search in the whole container
  }

  //suckable surface initialization
  float suckable_point_x = container_w/2.0;
  float suckable_point_y = container_l/2.0;
  float suckable_point_z = container_h;
  
  std::vector<geometry_msgs::PoseStamped> vec_suckable_surfaces;
  int num_suckable_surface_candidates = 0;

  bool break_for_y = false;
  bool break_for_x = false;

  /// NOTE the fcl::box origin is in the center of the box

  const float X_START_BORDER_SAFETY_MARGIN = 0.01;
  const float X_END_BORDER_SAFETY_MARGIN = 0.03;
  const float Y_START_SAFETY_MARGIN = 0.01;
  const float Y_END_SAFETY_MARGIN = 0.03;
  //48x32 - borders - size of template
  const int MAX_NUM_SUCKABLE_SURFACE_CANDIDATES = 1500;

  // brute force search
  for (float y = start_search_from_this_y  + template_length / 2.0 + X_START_BORDER_SAFETY_MARGIN; y < finish_search_at_this_y - template_length / 2.0 - X_END_BORDER_SAFETY_MARGIN; y += 0.01)
  {
    for (float x = 0.01 + template_width / 2.0 + Y_START_SAFETY_MARGIN; x < container_w - template_width / 2.0 - Y_END_SAFETY_MARGIN; x += 0.01) // to avoid checking the whole container
    {
      for (float z = container_h; z > 0.01 + template_height / 2.0; z -= 0.01)
      {
        int count_collisions_of_template_with_map = 0; //this has to be 9 to consider that is a suckable surface

        // generate the template boxes in a different position every time
        std::vector<fcl::CollisionObject*> template_boxes;
        bool break_i = false;
        for(int i=0; i<3; ++i)
        {
          for(int j=0; j<3; ++j)
          {
            fcl::Box* box = new fcl::Box(0.01, 0.01, 0.01);
            const float box_x = x + i*0.01 + 0.005;
            const float box_y = y + j*0.01 + 0.005;
            const float box_z = z + 0.005;
            fcl::CollisionObject* aux = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                                                 fcl::Transform3f(fcl::Vec3f(box_x, box_y, box_z)));
            CollisionData collision_data;
            manager1->collide(aux, &collision_data, defaultCollisionFunction);
            ROS_DEBUG_STREAM("collision_data.result.numContacts() " <<collision_data.result.numContacts());

            // if there is a collision count it
            if (collision_data.result.isCollision())
            {
              ++count_collisions_of_template_with_map;
            }
            else // if no collision, break
            {
              break_i = true;
              break;
            }
          }
          if(break_i) break;
        }

        ROS_DEBUG_STREAM("count_collisions_of_template_with_map " <<count_collisions_of_template_with_map);
        if(count_collisions_of_template_with_map == 9)
        {
          suckable_surface_found = true;
          const float suckable_point_x = x + 0.015;
          const float suckable_point_y = y + 0.015;
          const float suckable_point_z = z + 0.005;
          geometry_msgs::PoseStamped suckable_surface_candidate_pose;
          suckable_surface_candidate_pose.header.stamp = ros::Time::now();
          suckable_surface_candidate_pose.header.frame_id = req.container_id.data;
          suckable_surface_candidate_pose.pose.position.x = suckable_point_x;
          suckable_surface_candidate_pose.pose.position.y = suckable_point_y;
          suckable_surface_candidate_pose.pose.position.z = suckable_point_z;
          suckable_surface_candidate_pose.pose.orientation.w = 1.0;
          suckable_surface_candidate_pose.pose.orientation.x = 0.0;
          suckable_surface_candidate_pose.pose.orientation.y = 0.0;
          suckable_surface_candidate_pose.pose.orientation.z = 0.0;
          ROS_DEBUG_STREAM("suckable surface found: " << suckable_point_x << ", " <<
                          suckable_point_y << ", " << suckable_point_z);
          insert_sorted(vec_suckable_surfaces, suckable_surface_candidate_pose); // added them in order
          if(num_suckable_surface_candidates >= MAX_NUM_SUCKABLE_SURFACE_CANDIDATES) // found suckable surface!
          {
            // these 3 breaks finish the search
            break_for_y = true;
            break_for_x = true;
            break;
          }
          else
          {
            ++num_suckable_surface_candidates;
            x += 0.03; //increase x with the size of the template to optimize
            break; // this break ends the search in z
          }
        }
      }
      if (break_for_x)
        break;
    }
    if (break_for_y)
      break;
  }
  ROS_INFO("Done");

  if (!suckable_surface_found)
  {
    ROS_ERROR("No suckable surface found");
  }

  // generate template tree (for visualization)
  octomap::OcTree template_tree(0.01);
  // discretize the template in cm voxels
  const int template_width_voxels = ceil(template_width / 0.01);
  const int template_length_voxels = ceil(template_length / 0.01);
  const int template_height_voxels = ceil(template_height / 0.01);
  // visualize all the surface candidates
  for(size_t i=0; i<vec_suckable_surfaces.size(); ++i)
  {
    for (int x_voxel = 0; x_voxel < template_width_voxels; ++x_voxel)
    {
      for (int y_voxel = 0; y_voxel < template_length_voxels; ++y_voxel)
      {
        for (int z_voxel = 0; z_voxel < template_height_voxels; ++z_voxel)
        {
          octomap::point3d aux_point((float)vec_suckable_surfaces[i].pose.position.x + (x_voxel * 0.01) - 0.015f,
                                     (float)vec_suckable_surfaces[i].pose.position.y + (y_voxel * 0.01) - 0.015f,
                                     (float)vec_suckable_surfaces[i].pose.position.z + (z_voxel * 0.01) - 0.005f);
          template_tree.updateNode(aux_point, true); // integrate 'occupied' measurement
        }
      }
    }
  }

  octomap_msgs::Octomap suckable_surface_msg;
  suckable_surface_msg.header.stamp = ros::Time(0);
  suckable_surface_msg.header.frame_id = req.container_id.data;
  octomap_msgs::binaryMapToMsg(template_tree, suckable_surface_msg);
  suckable_surfaces_pub_.publish(suckable_surface_msg);

  // print 10 poses of debug
  for(size_t i=0; i<vec_suckable_surfaces.size() && i<10; ++i)
  {
    ROS_INFO_STREAM("Suckable surface " << i << " from tnp_grasp: " << vec_suckable_surfaces[i]);
  }

  // output pose in container coordinates
  for(size_t i=0; i<vec_suckable_surfaces.size(); ++i)
  {
    ROS_DEBUG_STREAM("Suckable surface " << i << " from tnp_grasp: " << vec_suckable_surfaces[i]);
    res.suction_poses.push_back(vec_suckable_surfaces[i]);
  }

  return true;
}

bool GraspPlannerNode::GetSuctionCandidatesBBCallback(tnp_grasp_planner::getSuctionCandidates_bb::Request &req,
                                                      tnp_grasp_planner::getSuctionCandidates_bb::Response &res)
{
  ROS_DEBUG("GetSuctionCandidatesBBCallback was called");
  ROS_ERROR("GetSuctionCandidatesBBCallback NOT IMPLEMENTED");

  return true;
}

bool GraspPlannerNode::GetSuctionCandidatesContainerCallback(
    tnp_grasp_planner::getSuctionCandidates_container::Request &req,
    tnp_grasp_planner::getSuctionCandidates_container::Response &res)
{
  ROS_DEBUG("GetSuctionCandidatesContainerCallback was called");

  setParameters();

  // Reset candidates vector
  suction_candidates_.clear();

  // Load a point cloud of an empty container
  ROS_INFO("Loading file with empty container pointcloud...");
  std::string container_pcd_path = pcd_save_dir_ + req.container + ".pcd";
  pcl::PointCloud<PointType>::Ptr container_cloud(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile(container_pcd_path, *container_cloud);
  ROS_INFO("Loading file with empty container pointcloud... done");

  // Extract different points i.e. a point cloud of items inside container
  if (cloud_->points.size() == 0)
  {
    ROS_ERROR("Input point cloud is empty!");
    return false;
  }
  ROS_INFO_STREAM("Input cloud has " << cloud_->points.size() << " data points");

  ROS_INFO("Get the difference between the saved and the current pointclouds...");
  pcl::PointCloud<PointType>::Ptr diff(new pcl::PointCloud<PointType>);
  diff = diff_extraction(container_cloud, cloud_);
  ROS_INFO("Get the difference between the saved and the current pointclouds... Done");
  if (diff->points.size() == 0)
  {
    ROS_INFO("There is no difference between the empty container and a input cloud");
    return false;
  }

  // PassThrough filter
  ROS_INFO("Passthrough filter...");
  pcl::PointCloud<PointType>::Ptr diff_cloud(new pcl::PointCloud<PointType>);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(diff);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(thresh_passthrough_minz_, thresh_passthrough_maxz_);
  pass.filter(*diff_cloud);
  ROS_INFO("Passthrough filter... Done");
  if (diff_cloud->points.size() == 0)
  {
    ROS_INFO("There is no data point after PassThrough filter");
    return false;
  }
  ROS_INFO_STREAM("Diff cloud has " << diff_cloud->points.size() << " data points");

  // Clustering
  ROS_INFO("Clustering...");
  std::vector<pcl::PointCloud<PointType>::Ptr> clusters;
  /*debug*/ros::Time time_before_conditionalEuclideanClustering = ros::Time::now();
  conditionalEuclideanClustering(diff_cloud, clusters);
  /*debug*/ROS_INFO_STREAM("conditionalEuclideanClustering took " << ((ros::Time::now() - time_before_conditionalEuclideanClustering).toSec()) << "s");
  ROS_INFO("Clustering... Done");
  if (clusters.size() > 0)
  {
    ROS_INFO_STREAM("Number of suckable clusters: " << clusters.size());
  }
  else
  {
    ROS_INFO("Suckable cluster not detected");
    return false;
  }

  ROS_INFO("Estimating suction candidates...");
  estimateSuctionCandidates(clusters);
  res.suction_poses = suction_candidates_;
  ROS_INFO("Estimating suction candidates... Done");

  return true;
}

bool GraspPlannerNode::GetGripCandidatesBBCallback(tnp_grasp_planner::getGripCandidates_bb::Request &req,
                                                   tnp_grasp_planner::getGripCandidates_bb::Response &res)
{
  ROS_DEBUG("GetGripCandidatesBBCallback was called");
  ROS_ERROR("GetGripCandidatesBBCallback NOT IMPLEMENTED");

  return true;
}

bool GraspPlannerNode::GetGripCandidatesContainerCallback(tnp_grasp_planner::getGripCandidates_container::Request &req,
                                                          tnp_grasp_planner::getGripCandidates_container::Response &res)
{
  ROS_DEBUG("GetSuctionCandidatesBBCallback was called");
  ROS_ERROR("GetSuctionCandidatesBB NOT IMPLEMENTED");

  return true;
}

bool GraspPlannerNode::GetContainerOccupancyCallback(tnp_grasp_planner::getContainerOccupancy::Request &req,
                                                     tnp_grasp_planner::getContainerOccupancy::Response &res)
{
  ROS_DEBUG("GetContainerOccupancyCallback was called");
  ROS_ERROR("GetContainerOccupancyCallback NOT IMPLEMENTED");

  setParameters();

  float x_size, y_size, z_size;
  std::stringstream ss_pname;
  ss_pname << "tnp_environment/" << req.container << "_w";
  n_.param<float>(ss_pname.str(), x_size, 0.04); // 0.34
  ss_pname.str("");
  ss_pname << "tnp_environment/" << req.container << "_l";
  n_.param<float>(ss_pname.str(), y_size, 0.03); // 0.297
  ss_pname.str("");
  ss_pname << "tnp_environment/" << req.container << "_h";
  n_.param<float>(ss_pname.str(), z_size, 0.02); // 0.121
  int x_grid_num = (int)(x_size / occu_grid_size_);
  int y_grid_num = (int)(y_size / occu_grid_size_);
  int z_grid_num = (int)(z_size / occu_grid_size_);
  std::cout << "grid dim: " << x_grid_num << " x " << y_grid_num << " x " << z_grid_num << std::endl;

  std_msgs::ByteMultiArray occu_grid;
  occu_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
  occu_grid.layout.dim[0].label = "x";
  occu_grid.layout.dim[0].size = x_grid_num;
  occu_grid.layout.dim[0].stride = z_grid_num * y_grid_num * x_grid_num;
  occu_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
  occu_grid.layout.dim[1].label = "y";
  occu_grid.layout.dim[1].size = y_grid_num;
  occu_grid.layout.dim[1].stride = z_grid_num * y_grid_num;
  occu_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
  occu_grid.layout.dim[2].label = "z";
  occu_grid.layout.dim[2].size = z_grid_num;
  occu_grid.layout.dim[2].stride = z_grid_num;

  // Load a point cloud of an empty container
  std::string container_pcd_path = pcd_save_dir_ + req.container + ".pcd";
  pcl::PointCloud<PointType>::Ptr container_cloud(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile(container_pcd_path, *container_cloud);

  // Extract different points i.e. a point cloud of items inside container
  if (cloud_->points.size() == 0)
  {
    ROS_ERROR("Input point cloud is empty!");
    return false;
  }
  std::stringstream ss;
  ss << pcd_save_dir_ << "input.pcd";
  pcl::io::savePCDFileBinary(ss.str(), *cloud_);
  ROS_INFO_STREAM("Input cloud has " << cloud_->points.size() << " data points");
  ROS_INFO_STREAM("Saved input cloud to: " << ss.str());

  pcl::PointCloud<PointType>::Ptr diff(new pcl::PointCloud<PointType>);
  diff = diff_extraction(container_cloud, cloud_);
  if (diff->points.size() == 0)
  {
    ROS_INFO("No difference detected. The container is empty.");
    setOccupancyGridEmpty(occu_grid);
    res.occupancy_grid = occu_grid;
    return true;
  }

  return true;
}

bool GraspPlannerNode::SavePCDFileCallback(tnp_grasp_planner::savePCDFile::Request &req,
                                           tnp_grasp_planner::savePCDFile::Response &res)
{
  ROS_DEBUG("SavePCDFileCallback was called");
  std::stringstream ss;
  ss << pcd_save_dir_ << req.container << ".pcd";
  int success = pcl::io::savePCDFileBinary(ss.str(), *cloud_);
  if (success == 0)
  {
    ROS_INFO_STREAM("Cloud has " << cloud_->points.size() << " points");
    ROS_INFO_STREAM("Saved PCD file to: " << ss.str());
    res.file_path = ss.str();
    return true;
  }
  else
  {
    return false;
  }
}

////---------------------------------------------------------------------------
//// Topics callbacks declarations
////---------------------------------------------------------------------------

void pointCloud2ToOctomapWithScale(const sensor_msgs::PointCloud2& cloud, octomap::Pointcloud& octomapCloud, 
  const float& scale_x, const float& scale_y, const float& scale_z){
    octomapCloud.reserve(cloud.data.size() / cloud.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
        octomapCloud.push_back((*iter_x)*scale_x, (*iter_y)*scale_y, (*iter_z)*scale_z);
    }
  }

// SR300
void GraspPlannerNode::CloudFromSR300Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, *cloud_);
  ROS_DEBUG_STREAM("Received cloud from SR300 with " << msg->height * msg->width << " points");
  octomap::Pointcloud new_cloud;
  pointCloud2ToOctomapWithScale(*msg, new_cloud, 1.0, 1.0, 1.0); // no rescale
  SetOctomapCloudFromSR300(new_cloud);
}

// Photoneo
void GraspPlannerNode::CloudFromPhoxiCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // TODO
}

////---------------------------------------------------------------------------
//// private functions
////---------------------------------------------------------------------------
pcl::PointCloud<PointType>::Ptr GraspPlannerNode::diff_extraction(pcl::PointCloud<PointType>::Ptr cloud_base,
                                                                  pcl::PointCloud<PointType>::Ptr cloud_test)
{
  // Create octree
  pcl::octree::OctreePointCloudChangeDetector<PointType> octree(octree_resolution_);
  octree.setInputCloud(cloud_base);
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  octree.setInputCloud(cloud_test);
  octree.addPointsFromInputCloud();

  std::vector<int> newPointIdxVector;
  octree.getPointIndicesFromNewVoxels(newPointIdxVector);

  // Prepare output point cloud
  pcl::PointCloud<PointType>::Ptr cloud_diff(new pcl::PointCloud<PointType>);
  cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
  cloud_diff->height = 1;
  cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

  int diff_points_num = 0;
  for (size_t i = 0; i < newPointIdxVector.size(); i++)
  {
    cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
    cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
    cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
    cloud_diff->points[i].r = cloud_test->points[newPointIdxVector[i]].r;
    cloud_diff->points[i].g = cloud_test->points[newPointIdxVector[i]].g;
    cloud_diff->points[i].b = cloud_test->points[newPointIdxVector[i]].b;
    diff_points_num++;
  }

  // Set size of diff cloud
  cloud_diff->width = diff_points_num;
  cloud_diff->height = 1;
  cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

  return cloud_diff;
}

void GraspPlannerNode::euclideanClustering(pcl::PointCloud<PointType>::Ptr cloud_in,
                                           std::vector<pcl::PointCloud<PointType>::Ptr> &clusters)
{
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);

  int j = 0;
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back(cloud_in->points[*pit]);
      PointType p;
      p.x = cloud_in->points[*pit].x;
      p.y = cloud_in->points[*pit].y;
      p.z = cloud_in->points[*pit].z;
      p.r = colors[j % 6][0];
      p.g = colors[j % 6][1];
      p.b = colors[j % 6][2];
      cloud_out->points.push_back(p);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
        << std::endl;
    clusters.push_back(cloud_cluster);
    j++;
  }
  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  cloud_cluster_colored_ = cloud_out;
}

void GraspPlannerNode::conditionalEuclideanClustering(pcl::PointCloud<PointType>::Ptr cloud_in,
                                                      std::vector<pcl::PointCloud<PointType>::Ptr> &clusters)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloudXYZRGBtoXYZI(*cloud_in, *cloud_xyzi);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
  /*debug*/ros::Time time_before_estimateNormals = ros::Time::now();
  estimateNormals(cloud_xyzi, cloud_with_normals); // time > 1s
  /*debug*/ROS_INFO_STREAM("estimateNormals took " << ((ros::Time::now() - time_before_estimateNormals).toSec()));

  // Set up a Conditional Euclidean Clustering class
  pcl::IndicesClustersPtr cluster_indices(new pcl::IndicesClusters);
  pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
  pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
  pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
  cec.setInputCloud(cloud_with_normals);
  //// set condition function parameters
  squared_distance = squared_distance_;
  thresh_intensity = thresh_intensity_;
  thresh_normal_angle_cos = cos(thresh_normal_angle_deg_ / 180.0 * M_PI);
  thresh_curvature = thresh_curvature_;
  thresh_intensity_l_dist = thresh_intensity_l_dist_;
  //// set condition function
  cec.setConditionFunction(&enforceCurvatureSimilarity);
  cec.setClusterTolerance(cluster_tolerance_);
  cec.setMinClusterSize(min_cluster_size_); //time < 1ms
  cec.setMaxClusterSize(max_cluster_size_); //time < 1ms
  /*debug*/ros::Time time_before = ros::Time::now();
  cec.segment(*cluster_indices); //time > 11s!!!!!!!!!!!!!!!!!!!!!
  /*debug*/ROS_INFO_STREAM("cec.segment took " << ((ros::Time::now() - time_before).toSec()) << "s");
  cec.getRemovedClusters(small_clusters, large_clusters);
  ROS_INFO_STREAM("CEC: " << cluster_indices->size() << " clusters detected");
  ROS_INFO_STREAM("CEC: " << small_clusters->size() << " small clusters detected");
  ROS_INFO_STREAM("CEC: " << large_clusters->size() << " large clusters detected");

  // Output visualization
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_out_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::PointCloudXYZRGBtoXYZHSV(*cloud_in, *cloud_out_hsv);
  for (size_t i = 0; i < cloud_in->points.size(); ++i) //time 2ms
  {
    cloud_out_hsv->points[i].x = cloud_in->points[i].x;
    cloud_out_hsv->points[i].y = cloud_in->points[i].y;
    cloud_out_hsv->points[i].z = cloud_in->points[i].z;
  }

  float brighter = 0.3;
  for (int i = 0; i < small_clusters->size(); ++i) // time < 1ms
  {
    for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
    {
      if (cloud_out_hsv->points[(*small_clusters)[i].indices[j]].v > 1.0 - brighter)
      {
        cloud_out_hsv->points[(*small_clusters)[i].indices[j]].v = 1.0;
      }
      else
      {
        cloud_out_hsv->points[(*small_clusters)[i].indices[j]].v += brighter;
      }
      pcl::PointXYZHSVtoXYZRGB(cloud_out_hsv->points[(*small_clusters)[i].indices[j]],
                               cloud_out->points[(*small_clusters)[i].indices[j]]);
    }
  }

  float dimmer = 0.3;
  for (int i = 0; i < large_clusters->size(); ++i) // time < 1ms
  {
    for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
    {
      if (cloud_out_hsv->points[(*large_clusters)[i].indices[j]].v < dimmer)
      {
        cloud_out_hsv->points[(*large_clusters)[i].indices[j]].v = 0;
      }
      else
      {
        cloud_out_hsv->points[(*large_clusters)[i].indices[j]].v -= dimmer;
      }
      pcl::PointXYZHSVtoXYZRGB(cloud_out_hsv->points[(*large_clusters)[i].indices[j]],
                               cloud_out->points[(*large_clusters)[i].indices[j]]);
    }
  }

  for (int i = 0; i < cluster_indices->size(); ++i) // time > 10ms
  {
    pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
    for (int j = 0; j < (*cluster_indices)[i].indices.size(); ++j)
    {
      cloud_cluster->points.push_back(cloud_out->points[(*cluster_indices)[i].indices[j]]);
      cloud_out->points[(*cluster_indices)[i].indices[j]].r = colors[i % 6][0];
      cloud_out->points[(*cluster_indices)[i].indices[j]].g = colors[i % 6][1];
      cloud_out->points[(*cluster_indices)[i].indices[j]].b = colors[i % 6][2];
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO_STREAM("cluster " << i << " has " << cloud_cluster->points.size() << " data points");
    clusters.push_back(cloud_cluster);
  }

  pcl::copyPointCloud(*cloud_out, *cloud_cluster_colored_);
}

inline bool SortingVectorFunction(const geometry_msgs::PoseStamped& i, const geometry_msgs::PoseStamped& j)
{
  return (i.pose.position.z < j.pose.position.z);
}

void GraspPlannerNode::estimateSuctionCandidates(std::vector<pcl::PointCloud<PointType>::Ptr> clusters)
{
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*clusters[i], xyz_centroid);
    pcl::PointCloud<PointTypeNormal>::Ptr cloud_cluster_normal(new pcl::PointCloud<PointTypeNormal>);
    estimateNormalsSuction(clusters[i], cloud_cluster_normal);
    // ToDo: pick up normal vector near the centroid

    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.stamp = ros::Time::now();
    pose_tmp.header.frame_id = depth_camera_frame_;
    pose_tmp.pose.position.x = xyz_centroid[0];
    pose_tmp.pose.position.y = xyz_centroid[1];
    pose_tmp.pose.position.z = xyz_centroid[2];
    pose_tmp.pose.orientation.x = 0.0;
    pose_tmp.pose.orientation.y = 0.0;
    pose_tmp.pose.orientation.z = 0.0;
    pose_tmp.pose.orientation.w = 1.0;
    suction_candidates_.push_back(pose_tmp);
  }

  //sort vector by z values
  std::sort(suction_candidates_.begin(), suction_candidates_.end(), SortingVectorFunction);

}

void GraspPlannerNode::estimateNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
  // Set up a Normal Estimation class and merge data in cloud_with_normals
  pcl::copyPointCloud(*cloud_in, *cloud_with_normals);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>);
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(search_tree);
  ne.setRadiusSearch(search_radius_);
  ne.compute(*cloud_with_normals);
}

void GraspPlannerNode::estimateNormalsSuction(pcl::PointCloud<PointType>::Ptr cloud_in,
                                              pcl::PointCloud<PointTypeNormal>::Ptr cloud_with_normals)
{
  // Set up a Normal Estimation class and merge data in cloud_with_normals
  pcl::copyPointCloud(*cloud_in, *cloud_with_normals);
  pcl::NormalEstimation<PointType, PointTypeNormal> ne;
  pcl::search::KdTree<PointType>::Ptr search_tree(new pcl::search::KdTree<PointType>);
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(search_tree);
  ne.setRadiusSearch(search_radius_suck_);
  ne.compute(*cloud_with_normals);
}

void GraspPlannerNode::setOccupancyGridEmpty(std_msgs::ByteMultiArray& occu_grid)
{
  for (size_t i = 0; i < occu_grid.layout.dim[0].stride; ++i)
  {
    occu_grid.data.push_back(0);
  }
}

/*-------------------------------------------------------------------------------------*/
/// main
/*-------------------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_grasp_planner");

  //Create an object of class GraspPlannerNode that will take care of everything
  GraspPlannerNode SAPObject;
  ROS_DEBUG("[TNP_STATE L0] Starting /tnp_grasp_planner Starting the grasp planner node");
  ROS_INFO("Grasp planner node started");

  mkdir(SAPObject.pcd_save_dir_.c_str(), 0775);
  ROS_INFO_STREAM(
      "Set parameters from launch file:" << std::endl << "  cloud_topic_sr300_ = " << SAPObject.cloud_topic_sr300_ << std::endl << "  pcd_save_dir_      = " << SAPObject.pcd_save_dir_ << std::endl << "  search_radius_     = " << SAPObject.search_radius_ << std::endl << "  octree_resolution_ = " << SAPObject.octree_resolution_ << std::endl << "  thresh_passthrough_minz_ = " << SAPObject.thresh_passthrough_minz_ << std::endl << "  thresh_passthrough_maxz_ = " << SAPObject.thresh_passthrough_maxz_ << std::endl << "  cluster_tolerance_ = " << SAPObject.cluster_tolerance_ << std::endl << "  min_cluster_size_  = " << SAPObject.min_cluster_size_ << std::endl << "  max_cluster_size_  = " << SAPObject.max_cluster_size_ << std::endl << "  squared_distance_        = " << SAPObject.squared_distance_ << std::endl << "  thresh_intensity_        = " << SAPObject.thresh_intensity_ << std::endl << "  thresh_normal_angle_deg_ = " << SAPObject.thresh_normal_angle_deg_ << std::endl << "  thresh_curvature_        = " << SAPObject.thresh_curvature_ << std::endl << "  thresh_intensity_l_dist_ = " << SAPObject.thresh_intensity_l_dist_ << std::endl << "  search_radius_suck_      = " << SAPObject.search_radius_suck_ << std::endl << "  occu_grid_size_    = " << SAPObject.occu_grid_size_ << std::endl);

  ros::spin();

  return 0;
}
