// Copyright (c) 2023 Eric Slaghuis
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
 * Deals with the interface to the navigation stack, and calls
 * the stated plugin.
 * *************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <iterator>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav_drone_msgs/action/compute_path_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include "nav_drone_util/robot_utils.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/visibility_control.h"
#include <nav_drone_core/planner.hpp>

#include <pluginlib/class_loader.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

namespace nav_drone_planner
{
class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathToPose = nav_drone_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
  
  using PlannerMap = std::unordered_map<std::string, nav_drone_core::Planner::Ptr>;

  NAV_DRONE_PUBLIC
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // Variables for node paramaters
  std::string map_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
    
  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav_drone_core::Planner> loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::shared_ptr<octomap::OcTree> octomap_;
  
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  
  void init();
  
  // MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg); 
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
  
   // PLANNER ACTION SERVER //////////////////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
  
  
  // UTILITY FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////
  bool getStartPose(
    std::shared_ptr<const ComputePathToPose::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);
  
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);
  
  bool validate_path(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

};  // class PlannerServer

}  // namespace nav_drone_planner
