// tnp_task_manager_helpers.cpp
// Simple functions to help us condense our code

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
 
#ifndef TNP_TASK_MANAGER_HELPER_H
#define TNP_TASK_MANAGER_HELPER_H


#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "vector"

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
#include <termios.h>    // non-blocking getchar
#include <thread>       // To wait for time_to_destination
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <mutex>
#include <fstream>
#include <cmath>

/*-------------------------------------------------------------------------------------*/
/// Helpers definitions
/// Grasp Planner
/*-------------------------------------------------------------------------------------*/
inline bool orderCandidates(geometry_msgs::PoseStamped &i, geometry_msgs::PoseStamped &j)
{
  return (i.pose.position.z > j.pose.position.z);
}

inline bool orderCandidatesCombinedVectors(std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> &i,
                                           std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> &j)
{
  return (i.first.pose.position.z > j.first.pose.position.z);
}

inline bool
orderCandidatesCombinedVectorsIdConfs(std::pair<std::string, float> &i, std::pair<std::string, float> &j)
{
  return (i.second > j.second);
}


std::vector<std::pair<std::string, float> > pairedVectorSortedByConfidence(std::vector<std::string> item_ids, std::vector<float> item_confs)
{
  std::vector<std::pair<std::string, float>> combined_vector;
  for(size_t i=0; i<item_ids.size() && i<item_confs.size(); ++i)
  {
    combined_vector.push_back(std::make_pair(item_ids[i], item_confs[i]));
  }
  std::sort (combined_vector.begin(), combined_vector.end(), orderCandidatesCombinedVectorsIdConfs );
  return combined_vector;
}

std::vector<std::pair<std::string, float>> pairedVectorSortedByConfidence(std::vector<std_msgs::String> item_ids, std::vector<float> item_confs)
{
  std::vector<std::pair<std::string, float>> combined_vector;
  for(size_t i=0; i<item_ids.size() && i<item_confs.size(); ++i)
  {
    combined_vector.push_back(std::make_pair(item_ids[i].data, item_confs[i]));
  }
  std::sort (combined_vector.begin(), combined_vector.end(), orderCandidatesCombinedVectorsIdConfs );
  return combined_vector;
}

std::vector<std::pair<std::string, float>> pairedVectorSortedByConfidence(std::vector<std_msgs::String> item_ids, std::vector<std_msgs::Float64> item_confs)
{
  std::vector<std::pair<std::string, float>> combined_vector;
  for(size_t i=0; i<item_ids.size() && i<item_confs.size(); ++i)
  {
    combined_vector.push_back(std::make_pair(item_ids[i].data, item_confs[i].data));
  }
  std::sort (combined_vector.begin(), combined_vector.end(), orderCandidatesCombinedVectorsIdConfs );
  return combined_vector;
}

void dump_decision_making_data_to_screen(
    std::string dl_target_item, float dl_item_conf, 
    std::vector<std::pair<std::string, float>> weight_combined_vectors, 
    std::vector<std::pair<std::string, float>> volume_combined_vectors, 
    std::vector<std::string> vec_cloud_matching_items_ids_, 
    std::vector<float> vec_cloud_matching_confs_, 
    std::vector<std::pair<std::string, float>> akaze_svm_combined_vectors, 
    std::vector<std::pair<std::string, float>> color_histogram_combined_vectors)
{
  ROS_INFO( "-----START OF DECISION-MAKING DATA-----");
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "DL results");
  ROS_INFO( "---------------------------------------");
  ROS_INFO_STREAM( dl_target_item << " " << dl_item_conf);
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "Weight results");
  ROS_INFO( "---------------------------------------");
  for (size_t i = 0; i < weight_combined_vectors.size(); ++i)
    ROS_INFO_STREAM( weight_combined_vectors[i].first << " " << weight_combined_vectors[i].second);
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "Volume results");
  ROS_INFO( "---------------------------------------");
  for (size_t i = 0; i < volume_combined_vectors.size(); ++i)
    ROS_INFO_STREAM( volume_combined_vectors[i].first << " " << volume_combined_vectors[i].second);
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "Cloud matching results");
  ROS_INFO( "---------------------------------------");
  for (size_t i = 0; i < vec_cloud_matching_items_ids_.size(); ++i)
    ROS_INFO_STREAM( vec_cloud_matching_items_ids_[i] << " " << vec_cloud_matching_confs_[i]);
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "Akaze results");
  ROS_INFO( "---------------------------------------");
  for (size_t i = 0; i < akaze_svm_combined_vectors.size(); ++i)
    ROS_INFO_STREAM( akaze_svm_combined_vectors[i].first << " " << akaze_svm_combined_vectors[i].second);
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "Color histogram results");
  ROS_INFO( "---------------------------------------");
  ROS_INFO( "------END OF DECISION-MAKING DATA------");
  for (size_t i = 0; i < color_histogram_combined_vectors.size(); ++i)
    ROS_INFO_STREAM( color_histogram_combined_vectors[i].first << " " << color_histogram_combined_vectors[i].second);
}

std::vector<float> convert_msgfloat_to_float_vec(std::vector<std_msgs::Float64> msgvec)
{
  std::vector<float> outvec;
  for(size_t i=0; i<msgvec.size(); ++i)
  {
    outvec.push_back(msgvec[i].data);
  }
  return outvec;
}

std::vector<std::string> convert_msgstring_to_string_vec(std::vector<std_msgs::String> msgvec)
{
  std::vector<std::string> outvec;
  for(size_t i=0; i<msgvec.size(); ++i)
  {
    outvec.push_back(msgvec[i].data);
  }
  return outvec;
}

std::vector<std_msgs::String> convert_string_to_msgstring_vec(std::vector<std::string> msgvec)
{
  std::vector<std_msgs::String> outvec;
  std_msgs::String tmp_string;
  for(size_t i=0; i<msgvec.size(); ++i)
  {
    tmp_string.data = msgvec[i];
    outvec.push_back(tmp_string);
  }
  
  return outvec;
}

void accumulate_votes_geometric_sum(std::vector<std::string> & possible_items_ids, std::vector<float>& possible_items_confidences, std::vector<std::string> vec_weight_items_ids_, std::vector<float> vec_weight_items_confs_, float weight_vote_share_)
{
    for (size_t i = 0; i < vec_weight_items_ids_.size(); ++i)
      {
        const float weight_vote = vec_weight_items_confs_[i] * weight_vote_share_;
        bool located = false;
        for (size_t j = 0; j < possible_items_ids.size(); ++j)
        {
          if (vec_weight_items_ids_[i].compare(possible_items_ids[j]) == 0)
          {
            located = true;
            possible_items_confidences[j] += weight_vote;
            break;
          }
        }
        if (!located)
        {
          possible_items_ids.push_back(vec_weight_items_ids_[i]);
          possible_items_confidences.push_back(weight_vote);
        }
      }
}

void write_decision_making_data_to_log(
    std::ofstream& ofs_data_gathering,
    std::string dl_target_item, 
    float dl_item_conf, 
    std::vector<std::pair<std::string, float>> weight_combined_vectors, 
    std::vector<std::pair<std::string, float>> volume_combined_vectors, 
    std::vector<std::string> vec_cloud_matching_items_ids_, 
    std::vector<float> vec_cloud_matching_confs_, 
    std::vector<std::pair<std::string, float>> akaze_svm_combined_vectors, 
    std::vector<std::pair<std::string, float>> color_histogram_combined_vectors)
{
  try {
      if(ofs_data_gathering)
      {
        ofs_data_gathering<<"--------------------------"<<std::endl;
        ofs_data_gathering<<ros::Time::now()<<std::endl;
        ofs_data_gathering<<"# DL: "<<std::endl;
        ofs_data_gathering<<dl_target_item << " " << dl_item_conf << std::endl;
        ofs_data_gathering<<"# Weight: " <<std::endl;
        for(size_t ilog(0); ilog<weight_combined_vectors.size(); ++ilog)
        {
          ofs_data_gathering<<weight_combined_vectors[ilog].first<<" "<<weight_combined_vectors[ilog].second<<std::endl;
        }
        ofs_data_gathering<<"# Volume: " <<std::endl;
        for(size_t ilog(0); ilog<volume_combined_vectors.size(); ++ilog)
        {
          ofs_data_gathering<<volume_combined_vectors[ilog].first<<" "<<volume_combined_vectors[ilog].second<<std::endl;
        }
        ofs_data_gathering<<"# Cloud Matching: " <<std::endl;
        for(size_t ilog(0); ilog<vec_cloud_matching_items_ids_.size(); ++ilog)
        {
          ofs_data_gathering<<vec_cloud_matching_items_ids_[ilog]<<" "<<vec_cloud_matching_confs_[ilog]<<std::endl;
        }
        ofs_data_gathering<<"# Akaze SVM: " <<std::endl;
        for(size_t ilog(0); ilog<akaze_svm_combined_vectors.size(); ++ilog)
        {
          ofs_data_gathering<<akaze_svm_combined_vectors[ilog].first<<" "<<akaze_svm_combined_vectors[ilog].second<<std::endl;
        }
        ofs_data_gathering<<"# Color Histogram: " <<std::endl;
        for(size_t ilog(0); ilog<color_histogram_combined_vectors.size(); ++ilog)
        {
          ofs_data_gathering<<color_histogram_combined_vectors[ilog].first<<" "<<color_histogram_combined_vectors[ilog].second<<std::endl;
        }
      }
      else {ROS_ERROR("File error (ofs_data_gathering)");}
    }
    catch(...)
    {
      ROS_ERROR("File error (ofs_data_gathering)");
    }
}


bool addVoterToCumulatedVector_GeometricAddition(
    std::vector<std_msgs::String> voter_item_ids,
    std::vector<std_msgs::Float64> voter_item_confidences,
    float voter_vote_share,
    std::vector<std::string>& possible_item_ids,
    std::vector<float>& possible_item_confidences )
{
    for( size_t i = 0; i < voter_item_ids.size(); ++i )
    {
        const float voter_vote = voter_item_confidences[i].data * voter_vote_share;
        bool item_already_in_vector = false;
        for( size_t j = 0; j < possible_item_ids.size(); ++j )
        {
          if( voter_item_ids[i].data.compare( possible_item_ids[j] ) == 0 )
          {
            item_already_in_vector = true;
            possible_item_confidences[j] += voter_vote;
            break;
            }
        }
        if( !item_already_in_vector )
        {
          possible_item_ids.push_back( voter_item_ids[i].data );
          possible_item_confidences.push_back( voter_vote );
        }
    }
    return false;
}


bool addVoterToCumulatedVector_GeometricAddition(
    std::vector<std::string> voter_item_ids,
    std::vector<float> voter_item_confidences,
    float voter_vote_share,
    std::vector<std::string>& possible_item_ids,
    std::vector<float>& possible_item_confidences )
{
    for( size_t i = 0; i < voter_item_ids.size(); ++i )
    {
        const float voter_vote = voter_item_confidences[i] * voter_vote_share;
        bool item_already_in_vector = false;
        for( size_t j = 0; j < possible_item_ids.size(); ++j )
        {
          if( voter_item_ids[i].compare( possible_item_ids[j] ) == 0 )
          {
            item_already_in_vector = true;
            possible_item_confidences[j] += voter_vote;
            break;
            }
        }
        if( !item_already_in_vector )
        {
          possible_item_ids.push_back( voter_item_ids[i] );
          possible_item_confidences.push_back( voter_vote );
        }
    }
    return false;
}

#endif
