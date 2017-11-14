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

#ifndef TNP_GRASP_PLANNER_H
#define TNP_GRASP_PLANNER_H

#include <sys/stat.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/ByteMultiArray.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include "tf/transform_listener.h"

// services
#include "tnp_grasp_planner/getSuctionCandidates_bb.h"
#include "tnp_grasp_planner/getSuctionCandidates_container.h"
#include "tnp_grasp_planner/getGripCandidates_bb.h"
#include "tnp_grasp_planner/getGripCandidates_container.h"
#include "tnp_grasp_planner/getContainerOccupancy.h"
#include "tnp_grasp_planner/savePCDFile.h"
#include "tnp_grasp_planner/getPoseToPlaceItem.h"
#include "tnp_grasp_planner/updateOccupancyMap.h"
#include "tnp_grasp_planner/getSuctionCandidates_container_fromOctomap.h"
#include "tnp_kuka_motion/canRobotPlaceItemHere.h"
#include "phoxi_camera/TriggerImage.h"
#include "phoxi_camera/GetFrame.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeNormal;

//octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/Pointcloud.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <mutex>

//fcl
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include <boost/array.hpp>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

/**
 * GraspPlannerNode, calculates grasping points
 */
class GraspPlannerNode
{
public:
  GraspPlannerNode();

  bool setupROSConfiguration();
  bool setParameters();

  //Helpers
  octomap::Pointcloud GetOctomapCloudFromSR300();
  void SetOctomapCloudFromSR300(const octomap::Pointcloud& new_cloud);
  octomap::Pointcloud GetOctomapCloudFromPhoxi();
  void SetOctomapCloudFromPhoxi(const octomap::Pointcloud& new_cloud);
  bool canRobotPlaceItemHere(geometry_msgs::Pose target_pose, std::string container_name, bool use_gripper_EE = false);
  //Photoneo
  bool TriggerPhoxiCamera();

  // Service callback declarations
  bool
  GetSuctionCandidatesBBCallback(tnp_grasp_planner::getSuctionCandidates_bb::Request &req,
                                 tnp_grasp_planner::getSuctionCandidates_bb::Response &res);
  bool
  GetSuctionCandidatesContainerCallback(tnp_grasp_planner::getSuctionCandidates_container::Request &req,
                                        tnp_grasp_planner::getSuctionCandidates_container::Response &res);
  bool
  GetGripCandidatesBBCallback(tnp_grasp_planner::getGripCandidates_bb::Request &req,
                              tnp_grasp_planner::getGripCandidates_bb::Response &res);
  bool
  GetGripCandidatesContainerCallback(tnp_grasp_planner::getGripCandidates_container::Request &req,
                                     tnp_grasp_planner::getGripCandidates_container::Response &res);
  bool
  GetContainerOccupancyCallback(tnp_grasp_planner::getContainerOccupancy::Request &req,
                                tnp_grasp_planner::getContainerOccupancy::Response &res);
  bool
  SavePCDFileCallback(tnp_grasp_planner::savePCDFile::Request &req,
                      tnp_grasp_planner::savePCDFile::Response &res);

  bool
  UpdateOccupancyMapCallback(tnp_grasp_planner::updateOccupancyMap::Request &req,
                             tnp_grasp_planner::updateOccupancyMap::Response &res);
                
  bool FillOccupancyMap(octomap::OcTree* saved_occupancy_map_tree, const float& container_h);

  bool
  GetPoseToPlaceItemCallback(tnp_grasp_planner::getPoseToPlaceItem::Request &req,
                             tnp_grasp_planner::getPoseToPlaceItem::Response &res);

  bool
  GetSuctionCandidatesContainerFromOctomapCallback(
      tnp_grasp_planner::getSuctionCandidates_container_fromOctomap::Request &req,
      tnp_grasp_planner::getSuctionCandidates_container_fromOctomap::Response &res);

  // Topics callbacks declarations
  void
  CloudFromSR300Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void
  CloudFromPhoxiCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // Public member variables (can be set with launch file)
  std::string cloud_topic_sr300_; // SR300 topic name to subscribe
  std::string cloud_topic_phoxi_; // Phoxi (Photoneo) topic name to subscribe
  std::string pcd_save_dir_; // PCD directory
  float search_radius_;      // Parameters for surface normal estimation
  float octree_resolution_;  // Parameters for difference extraction
  float cluster_tolerance_;  // Parameters for Simple/Conditional Euclidean Clustering
  int min_cluster_size_;     //  |- minimum cluster size
  int max_cluster_size_;     //  `- maximum cluster size
  float squared_distance_;        // Parameters for Conditional Euclidean Clustering
  float thresh_intensity_;        //  |- disparity in intensity (0-255)
  float thresh_normal_angle_deg_; //  |- disparity between normal angles in degree (0-360)
  float thresh_curvature_; //  |- disparity in curvature (0.0-1.0)
  float thresh_intensity_l_dist_; //  `- disparity in intensity for two points with long distance (0-255)
  float search_radius_suck_; // Parameters for surface normal estimation (for sucking candidate)
  float thresh_passthrough_minz_;
  float thresh_passthrough_maxz_;
  float occu_grid_size_;

private:
  ros::NodeHandle n_, n_tmp_;
  ros::Subscriber cloud_from_sr300_sub_;
  ros::Subscriber cloud_from_phoxi_sub_;

  // visualization of occupancy maps
  ros::Publisher bin_A_occupancy_map_pub_;
  ros::Publisher bin_B_occupancy_map_pub_;
  ros::Publisher bin_C_occupancy_map_pub_;
  ros::Publisher tote_occupancy_map_pub_;
  ros::Publisher amnesty_occupancy_map_pub_;
  ros::Publisher box_1_occupancy_map_pub_;
  ros::Publisher box_2_occupancy_map_pub_;
  ros::Publisher box_3_occupancy_map_pub_;
  ros::Publisher item_octomap_pub_;
  ros::Publisher suckable_surfaces_pub_;
  ros::Publisher calibration_octomap_pub_;

  // services
  ros::ServiceServer getSuctionCandidates_bb_srv_;
  ros::ServiceServer getSuctionCandidates_container_srv_;
  ros::ServiceServer getGripCandidates_bb_srv_;
  ros::ServiceServer getGripCandidates_container_srv_;
  ros::ServiceServer getContainerOccupancy_srv_;
  ros::ServiceServer savePCDFile_srv_;
  ros::ServiceServer getPoseToPlaceItem_srv_;
  ros::ServiceServer updateOccupancyMap_srv_;
  ros::ServiceServer getSuctionCandidates_container_fromOctomap_srv_;

  // service clients
  // Photoneo
  ros::ServiceClient phoxi_camera_trigger_image_client_;
  ros::ServiceClient phoxi_camera_get_frame_client_;
  // occupancy maps
  ros::ServiceClient update_occupancy_map_service_client_;
  // get pose where to place item
  ros::ServiceClient can_robot_place_item_here_client_; 

  // Private member functions
  pcl::PointCloud<PointType>::Ptr
  diff_extraction(pcl::PointCloud<PointType>::Ptr cloud_base, pcl::PointCloud<PointType>::Ptr cloud_test);
  void
  euclideanClustering(pcl::PointCloud<PointType>::Ptr cloud_in, std::vector<pcl::PointCloud<PointType>::Ptr> &clusters);
  void
  conditionalEuclideanClustering(typename pcl::PointCloud<PointType>::Ptr cloud_in,
                                 std::vector<pcl::PointCloud<PointType>::Ptr> &clusters);
  void
  estimateSuctionCandidates(std::vector<pcl::PointCloud<PointType>::Ptr> clusters);

  void
  estimateNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
  void
  estimateNormalsSuction (pcl::PointCloud<PointType>::Ptr cloud_in,
                         pcl::PointCloud<PointTypeNormal>::Ptr cloud_with_normals);
  void
  setOccupancyGridEmpty(std_msgs::ByteMultiArray& occu_grid);

  // Private member variables
  pcl::PointCloud<PointType>::Ptr cloud_;
  pcl::PointCloud<PointType>::Ptr cloud_cluster_colored_;
  std::vector<geometry_msgs::PoseStamped> suction_candidates_;

  // occupancy maps
  octomap::OcTree bin_A_tree_, bin_B_tree_, bin_C_tree_;
  octomap::OcTree box_1_tree_, box_2_tree_, box_3_tree_;
  octomap::OcTree tote_tree_, amnesty_tree_;
  // cloud from the depth sensor
  octomap::Pointcloud octomap_cloud_from_sr300_;
  octomap::Pointcloud octomap_cloud_from_phoxi_;
  std::mutex octomap_cloud_from_sr300_mutex_;
  std::mutex octomap_cloud_from_phoxi_mutex_;
  // containers info
  float bin_A_w_, bin_A_l_, bin_A_h_, bin_A_x_, bin_A_y_, bin_A_z_;
  float bin_B_w_, bin_B_l_, bin_B_h_, bin_B_x_, bin_B_y_, bin_B_z_;
  float bin_C_w_, bin_C_l_, bin_C_h_, bin_C_x_, bin_C_y_, bin_C_z_;
  float box_1_w_, box_1_l_, box_1_h_, box_1_x_, box_1_y_, box_1_z_;
  float box_2_w_, box_2_l_, box_2_h_, box_2_x_, box_2_y_, box_2_z_;
  float box_3_w_, box_3_l_, box_3_h_, box_3_x_, box_3_y_, box_3_z_;
  float tote_w_, tote_l_, tote_h_, tote_x_, tote_y_, tote_z_;
  float amnesty_w_, amnesty_l_, amnesty_h_, amnesty_x_, amnesty_y_, amnesty_z_;
  // depth camera frame
  std::string depth_camera_frame_;
  // threshold to determine if a voxel is occupied or free (based on its occupancy probability)
  float occupied_voxel_threshold_;
  // this is a virtual/real division in the bin A measured from the origin of bin_A (in local coordinates) in y
  float bin_A_virtual_division_in_y_;
  // start searching for a free spot from this z (container coordinates)
  float start_searching_free_spot_from_this_z_;

  // tf
  tf::TransformListener tf_listener_;

  //debug flag
  bool tnp_debug_;

};

#endif
