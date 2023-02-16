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
#include <unordered_map>
#include <iterator>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav_drone_planner/planner_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav_drone_planner
{
PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
  : Node("planner_server", options),
    loader_("nav_drone_core", "nav_drone_core::Planner"),
    default_ids_{"DumbPlanner","ThetaStarPlanner"},
    default_types_{"nav_drone_dumb_planner/DumbPlanner","nav_drone_theta_star_planner/ThetaStarPlanner"}
{
    
  // Create a transform listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());      
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
  // Declare and get parameters    
  nav_drone_util::declare_parameter_if_not_declared(
    this, "map_frame", rclcpp::ParameterValue("map"));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue("base_link"));   
  nav_drone_util::declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.3));
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("planner_types", default_types_);
  declare_parameter("expected_planner_frequency", 1.0);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);   
  this->get_parameter("transform_tolerance", transform_tolerance_);
  this->get_parameter("planner_plugins", planner_ids_);
  this->get_parameter("planner_types", planner_types_);
    
  one_off_timer_ = this->create_wall_timer(
    200ms, std::bind(&PlannerServer::init, this));
       
}

  
void PlannerServer::init() {
  // Only run this once.  Stop the timer that triggered this.
  this->one_off_timer_->cancel();
    
    
  // Setup the planners
  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      nav_drone_core::Planner::Ptr planner =
        loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
          
      planner->configure(node, planner_ids_[i], tf_buffer_, octomap_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }    
    
  // ROS2 Subscriptions
  map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "nav_drone/map", 10, std::bind(&PlannerServer::map_callback, this, _1));
      
  // First, make sure that the transform between the robot base frame
  // and the global frame is available

  std::string tf_error;

  rclcpp::Rate r(2);
  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_WARN_ONCE(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), map_frame_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }
  RCLCPP_INFO(
      get_logger(), "Transform from %s to %s is available.",
      robot_base_frame_.c_str(), map_frame_.c_str());

  this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
    this,
    "nav_drone/compute_path_to_pose",
    std::bind(&PlannerServer::handle_goal, this, _1, _2),
    std::bind(&PlannerServer::handle_cancel, this, _1),
    std::bind(&PlannerServer::handle_accepted, this, _1));  
    
}
  
// MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
void PlannerServer::map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
{
  // Convert ROS message to a OctoMap
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
  } else { 
    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
  } 
}
  
  
// PLANNER ACTION SERVER //////////////////////////////////////////////////////////////////////////////////////////

rclcpp_action::GoalResponse PlannerServer::handle_goal(
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

rclcpp_action::CancelResponse PlannerServer::handle_cancel(
  const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PlannerServer::execute, this, _1), goal_handle}.detach();
}

void PlannerServer::execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
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
    
  // Use start pose if provided otherwise use current robot pose
  geometry_msgs::msg::PoseStamped start;
  if (!getStartPose(goal, start)) {
    goal_handle->abort(result);
    return;
  }
    
  result->path = getPlan(start, goal->goal, goal->planner_id);
    
  if( !validate_path( goal->goal, result->path, goal->planner_id)) {
      // No valid goal was calculated
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
  
bool PlannerServer::getStartPose(
  std::shared_ptr<const ComputePathToPose::Goal> goal,
  geometry_msgs::msg::PoseStamped & start) 
{
  if (goal->use_start) {
    start = goal->start;
  } else if( !nav_drone_util::getCurrentPose( start,
              *tf_buffer_, 
              map_frame_, 
              robot_base_frame_, 
              transform_tolerance_) ) {  
    RCLCPP_ERROR(this->get_logger(), "Failed to read current drone position.  Aborting the action.");
    return false;
  }
    
  return true;
}
  
nav_msgs::msg::Path PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Attempting to a find path from (%.2f, %.2f, %.2f) to "
    "(%.2f, %.2f, %.2f).", start.pose.position.x, start.pose.position.y, start.pose.position.z,
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  if (planners_.find(planner_id) != planners_.end()) {
    planners_[planner_id]->updateMap( octomap_ );
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}
  
bool PlannerServer::validate_path(
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN( this->get_logger(), "Planning algoritm %s failed to generate a valid"
      " path to (%.2f, %.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      
      return false;
  }
    
  RCLCPP_DEBUG(
    this->get_logger(),
    "Found a valid path of size %lu to (%.2f, %.2f, %.2f)",
    path.poses.size(),
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      
  return true;  
    
}

}  // namespace nav_drone_planner

RCLCPP_COMPONENTS_REGISTER_NODE(nav_drone_planner::PlannerServer)