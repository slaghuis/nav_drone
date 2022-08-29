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
 * Deals with the interface to the navigation stack, and calls
 * the stated plugin.
 * *************************************************************/

#include <functional>
#include <memory>
#include <chrono>
#include <thread>

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

namespace nav_drone
{
class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathToPose = nav_drone_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  NAV_DRONE_PUBLIC
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("planner_server", options),
    loader_("nav_drone_core", "nav_drone_core::Planner")
  {
    using namespace std::placeholders;
    
    // Create a transform listener
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Declare and get parameters    
    nav_drone_util::declare_parameter_if_not_declared(
      this, "map_frame", rclcpp::ParameterValue("map"));
    nav_drone_util::declare_parameter_if_not_declared(
      this, "robot_base_frame", rclcpp::ParameterValue("base_link"));   
    nav_drone_util::declare_parameter_if_not_declared(
      this, "transform_tolerance", rclcpp::ParameterValue(0.1));
      
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("transform_tolerance", transform_tolerance_);
   
    // ROS2 Subscriptions
    map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_drone/map", 10, std::bind(&PlannerServer::map_callback, this, _1));

    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "nav_drone/compute_path_to_pose",
      std::bind(&PlannerServer::handle_goal, this, _1, _2),
      std::bind(&PlannerServer::handle_cancel, this, _1),
      std::bind(&PlannerServer::handle_accepted, this, _1));
  }

protected:
  // Variables for node paramaters
  std::string map_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
    
  // For the plugins
  pluginlib::ClassLoader<nav_drone_core::Planner> loader_;
  std::shared_ptr<nav_drone_core::Planner> planner_;
  
private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::shared_ptr<octomap::OcTree> octomap_;
  
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  
  // MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
  {
    // Convert ROS message to a OctoMap
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
    } 
  }
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
  
   // PLANNER ACTION SERVER //////////////////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request for path to [%.2f;%.2f;%.2f]", 
      goal->goal.pose.position.x, 
      goal->goal.pose.position.y, 
      goal->goal.pose.position.z);
      
    (void)uuid;
    // Consider rejecting goals that does not present in the "map" frame
    // Alternatively, include a transform to the map frame.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PlannerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    
    RCLCPP_DEBUG(this->get_logger(), "Planner server is calculating a path to pose");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePathToPose::Result>();

    auto start_time = steady_clock_.now();
    
    if (octomap_ == NULL) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the first map");
    }
    
    // Wait for the fist map to arrive
    rclcpp::Rate r(100);
    while ( (octomap_ == NULL) && !goal_handle->is_canceling() ) {
      r.sleep();
    }
    
    if (goal_handle->is_canceling()) {
      result->planning_time = steady_clock_.now() - start_time;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled before the first map has been received!");
      return;
    }
    pluginlib::ClassLoader<nav_drone_core::Planner> loader_("nav_drone_core", "nav_drone_core::Planner");
    
    try
    {
      //std::shared_ptr<nav_drone_core::Planner> path_planner = planner_loader.createSharedInstance( goal->planner_id );
      planner_ = loader_.createSharedInstance(goal->planner_id);
      auto node_ptr = shared_from_this(); 
      planner_->configure(node_ptr, goal->planner_id, tf_buffer_, octomap_);

      // If false, use current robot pose as path start, if true, use start above instead
      if(goal->use_start == true) {
        RCLCPP_DEBUG(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f",
          goal->start.pose.position.x, 
          goal->start.pose.position.y, 
          goal->start.pose.position.z);
        result->path = planner_->createPlan( goal->start, goal->goal);
      } else {
        // use the current robot position.
        geometry_msgs::msg::PoseStamped current_pos;
        if( !nav_drone_util::getCurrentPose( current_pos,
              *tf_buffer_, 
              map_frame_, 
              robot_base_frame_, 
              transform_tolerance_) ) {   // From tf2
          RCLCPP_ERROR(this->get_logger(), "Failed to read current position.  Aborting the action.");
          goal_handle->abort(result);      
          return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f",
          current_pos.pose.position.x, 
          current_pos.pose.position.y, 
          current_pos.pose.position.z);
          
        result->path = planner_->createPlan( current_pos, goal->goal);
      }
      if( !validate_path( goal->goal, result->path, goal->planner_id)) {
        // No valid goal was calculated
        goal_handle->abort(result);
        return;
      }
    }
    
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
      goal_handle->abort(result);
      return;
    }
        
    // Check if goal is done
    if (rclcpp::ok()) {
      auto cycle_duration = steady_clock_.now() - start_time;
      result->planning_time = cycle_duration;
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Successfully calculated a path to the target pose in %d ns", cycle_duration);
    }
  } 
  
  bool validate_path(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id)
  {
    if (path.poses.size() == 0) {
      RCLCPP_WARN( this->get_logger(), "Planning algoritm %s failed to generate a valid"
        " path to (%.2f, %.2f, %.2f)", planner_id.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      // Navigation 2 calls a Termnate here.  What function can I use?
      
        return false;
    }
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Found a valid path of size %lu to (%.2f, %.2f, %.2f)",
      path.poses.size(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      
    return true;  
    
  }
};  // class PlannerServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(nav_drone::PlannerServer)