// Copyright (c) 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* *************************************************************
 * Simplified Costmap publisher
 * Reads Octomap messages and updates a costmap structure.
 * Publishes this costmap on a regular interval
 * *************************************************************/

#pragma once


#include <chrono>
#include <functional>
#include <memory>
#include <mutex>       // std::mutex
//#include <iterator>

#include "nav_drone_costmap_3d/costmap_3d.hpp"
#include "nav_drone_costmap_3d/cost_values.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_drone_msgs/msg/costmap.hpp"

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav_drone_costmap_3d {

class CostmapPublisher : public rclcpp::Node
{
public:
  CostmapPublisher();

protected:
  // Variables for node paramaters
  std::string map_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  double lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  double robot_radius_;
  double safety_radius_;
      
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<octomap::OcTree> octomap_;
  std::shared_ptr<Costmap3D> costmap_;
      
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex costmap_mutex;
  rclcpp::Time last_costmap_update_, last_octomap_update_;
  double last_costmap_radius_;
  geometry_msgs::msg::Pose last_update_pose_;
  geometry_msgs::msg::Twist last_velocity_;
  
  // MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg); 
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
  // ODOM SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  
  // MAP UPDATE COSTMAP ////////////////////////////////////////////////////////////////////////////////////////////////  
  std::pair<int, int> get_ez_grid_pos(const octomap::point3d & goal);
  bool update_costmap(const geometry_msgs::msg::PoseStamped & current_pose,
                      const double bounding_box_radius);
        
  // COSTMAP PUBLISHER ////////////////////////////////////////////////////////////////////////////////////////////////
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);  
  void publish_costmap();
  rclcpp::Publisher<nav_drone_msgs::msg::Costmap>::SharedPtr costmap_publisher_;
};  // class CostmapPublisher

}  // namespace nav_drone_costmap_3d
