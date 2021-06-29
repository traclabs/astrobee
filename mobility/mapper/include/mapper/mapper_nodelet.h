/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef MAPPER_MAPPER_NODELET_H_
#define MAPPER_MAPPER_NODELET_H_

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// ROS libraries
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <octomap_msgs/Octomap.h>

// Keepout zones for the planner
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <pluginlib/class_list_macros.h>

// FSW utils
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_serialization.h>
#include <ff_util/config_server.h>

// Service definition for zone registration
#include <ff_msgs/Hazard.h>
#include <ff_msgs/GetMap.h>

// Service messages
#include <ff_msgs/SetFloat.h>

// General messages
#include <std_msgs/Empty.h>

// C++ libraries
#include <fstream>
#include <vector>
#include <string>
#include <exception>
#include <thread>         // std::thread
#include <mutex>
#include <atomic>

// Astrobee message types
#include "ff_msgs/Segment.h"
#include "ff_msgs/ControlState.h"

// Classes
#include "mapper/octoclass.h"
#include "mapper/polynomials.h"
#include "mapper/sampled_trajectory.h"

// Data structures
#include "mapper/structs.h"

// My defined libraries
#include "mapper/visualization_functions.h"

namespace mapper {

namespace fs = boost::filesystem;

// Convenience declarations

class MapperNodelet : public ff_util::FreeFlyerNodelet {
 public:
  MapperNodelet();
  ~MapperNodelet();

 protected:
  void Initialize(ros::NodeHandle *nh);

  // Callbacks (see callbacks.cpp for implementation) ----------------
  // Callback for handling incoming point cloud messages
  void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  // Callback for handling incoming new trajectory messages
  void SegmentCallback(const ff_msgs::Segment::ConstPtr &msg);

  // Send diagnostics
  void DiagnosticsCallback(const ros::TimerEvent &event);

  // Configure callback for updating config file
  bool ReconfigureCallback(dynamic_reconfigure::Config &config);

  // Called when the EKF resets
  void ResetCallback(std_msgs::EmptyConstPtr const& msg);

  // Assert a fault - katie's fault code handling will eventually go in here
  void Complete(int32_t response);

  void PreemptCallback(void);

  void CancelCallback(void);

  // Services (see services.cpp for implementation) -----------------
  // Update resolution of the map
  bool UpdateResolution(ff_msgs::SetFloat::Request &req,
                        ff_msgs::SetFloat::Response &res);

  // Update map memory time
  bool UpdateMemoryTime(ff_msgs::SetFloat::Request &req,
                        ff_msgs::SetFloat::Response &res);

  // Update map inflation
  bool MapInflation(ff_msgs::SetFloat::Request &req,
                    ff_msgs::SetFloat::Response &res);

  // Reset the map
  bool ResetMap(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res);

  // Callback to get the free space in the map
  bool GetFreeMapCallback(ff_msgs::GetMap::Request& req,
                          ff_msgs::GetMap::Response& res);

  // Callback to get the obstacles in the map
  bool GetObstacleMapCallback(ff_msgs::GetMap::Request& req,
                              ff_msgs::GetMap::Response& res);

  // Threads (see threads.cpp for implementation) -----------------
  // Thread for fading memory of the octomap
  void FadeTask(ros::TimerEvent const& event);

  // Threads for constantly updating the tfTree values
  void HazTfTask(ros::TimerEvent const& event);
  void PerchTfTask(ros::TimerEvent const& event);
  void BodyTfTask(ros::TimerEvent const& event);

  // Thread for collision checking
  void CollisionCheckTask();

  // Thread for getting pcl data and populating the octomap
  void OctomappingTask();

  // Initialize fault management
  void InitFault(std::string const& msg);

 private:
  // Declare global variables (structures defined in structs.h)
  GlobalVariables globals_;
  MutexStruct mutexes_;
  SemaphoreStruct semaphores_;

  // Thread variables
  std::thread h_octo_thread_, h_fade_thread_, h_collision_check_thread_;

  // Subscriber variables
  ros::Subscriber haz_sub_, perch_sub_, segment_sub_, reset_sub_;

  // Octomap services
  ros::ServiceServer resolution_srv_, memory_time_srv_;
  ros::ServiceServer map_inflation_srv_, reset_map_srv_;

  // Thread rates (hz)
  double tf_update_rate_, fading_memory_update_rate_;

  // // Path planning services
  // ros::ServiceServer RRT_srv, octoRRT_srv, PRM_srv, graph_srv, Astar_srv;
  ros::ServiceServer newTraj_srv;

  // Trajectory validation variables -----------------------------
  State state_;                                       // State of the mapper (structure defined in struct.h)
  ff_util::Segment segment_;                          // Segment
  ros::ServiceServer get_free_map_srv_;               // Get free map service
  ros::ServiceServer get_obstacle_map_srv_;           // Set obstacle map service
  ros::Timer timer_d_, timer_f_, timer_h_, timer_p_, timer_b_;  // Diagnostics
  std::atomic<bool> killed_;
  ff_util::ConfigServer cfg_;                         // Config server

  // TF2
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  tf2_ros::Buffer buffer_;

  // Marker publishers
  ros::Publisher hazard_pub_;                      // Collision warnign
  ros::Publisher obstacle_marker_pub_;             // Obstacle map
  ros::Publisher free_space_marker_pub_;           // Obstacle map
  ros::Publisher obstacle_cloud_pub_;              // Obstacle map
  ros::Publisher free_space_cloud_pub_;            // Obstacle map
  ros::Publisher obstacle_octomap_pub_;            // Obstacle map
  ros::Publisher free_space_octomap_pub_;          // Obstacle map
  ros::Publisher inflated_obstacle_marker_pub_;    // Inflated obstacle map
  ros::Publisher inflated_free_space_marker_pub_;  // Inflated free map
  ros::Publisher inflated_obstacle_cloud_pub_;     // Inflated obstacle map
  ros::Publisher inflated_free_space_cloud_pub_;   // Inflated free map
  ros::Publisher path_marker_pub_;
  ros::Publisher cam_frustum_pub_;
  ros::Publisher map_keep_in_out_pub_;
};

}  // namespace mapper

#endif  // MAPPER_MAPPER_NODELET_H_
