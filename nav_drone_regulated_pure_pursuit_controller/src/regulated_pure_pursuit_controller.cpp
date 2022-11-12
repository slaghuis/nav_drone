// Copyright (c) 2022 Eric Slaghuis
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

/* *******************************************************
 * Plugin to the nav_drone controller server
 * Control algorith: Regulated Pure Pursuit Controller
 * Adapted from the original work by incorporating 3D flight, 
 * and the structural anomalies required bythe Nav_Drone 
 * package, the primary being the OctoMap instead of the costmap.
 * Leverages a PID controller to adjust altitude.
 * ******************************************************* */
#include <math.h>     // fabs, std::max
#include <algorithm>  // std::clamp (C++ 17), find_if
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav_drone_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav_drone_core/controller_exceptions.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/angle_utils.hpp"
#include "nav_drone_util/robot_utils.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav_drone_regulated_pure_pursuit_controller
{
  
  
void RegulatedPurePursuitController::configure(const rclcpp::Node::SharedPtr node, 
                                               std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                               std::shared_ptr<octomap::OcTree> costmap )
{
  node_ = node;
    
  tf_ = tf;
  plugin_name_ = name;
  costmap_ = costmap;    
  
  logger_ = node_->get_logger();
  clock_ = node->get_clock();

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.50;  // reasonable default before first update
  
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(1.6));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_velocity_scaling_angle", rclcpp::ParameterValue(0.785));  // 45 degrees
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_regulated_angular_velocity_scaling",
    rclcpp::ParameterValue(true));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".cost_map_size", rclcpp::ParameterValue(500.0));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot", rclcpp::ParameterValue(1.0));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.9));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_rotate_to_heading",
    rclcpp::ParameterValue(true));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".allow_reversing",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, "controller_frequency",
    rclcpp::ParameterValue(control_frequency));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(100.0));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".inscribed_radius", rclcpp::ParameterValue(300.0));
    
  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);

  node_->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node_->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node_->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node_->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node_->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  node_->get_parameter(
    plugin_name_ + ".cost_map_size",
    cost_map_size_);
  
  if (approach_velocity_scaling_dist_ > costmapSize() / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node_->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    max_allowed_time_to_collision_up_to_carrot_);
  node_->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
  node_->get_parameter(
    plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node_->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node_->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node_->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node_->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node_->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius",
    regulated_linear_scaling_min_radius_);
  node_->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node_->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node_->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node_->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node_->get_parameter(plugin_name_ + ".rotation_velocity_scaling_angle", rotation_velocity_scaling_angle_);
  node_->get_parameter(plugin_name_ + ".use_regulated_angular_velocity_scaling", use_regulated_angular_velocity_scaling_);
  node_->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node_->get_parameter("controller_frequency", control_frequency);
  node_->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node_->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);
  node_->get_parameter(
    plugin_name_ + ".inscribed_radius",
    inscribed_radius_);

  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
      "parameter cannot be set to true. By default setting use_rotate_to_heading true");
    allow_reversing_ = false;
  }
  
}
  
double RegulatedPurePursuitController::costmapSize()
{
  double x_size, y_size, z_size;
  if(costmap_ == NULL) {
    return cost_map_size_;
  } else {  
    costmap_->getMetricSize(x_size, y_size, z_size);
    return x_size;
  }  
}  

void RegulatedPurePursuitController::setPath(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}
  
void RegulatedPurePursuitController::updateMap(std::shared_ptr<octomap::OcTree> costmap)
{
  costmap_ = costmap;
}
  
double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Speed should only be achieved in the x or z dimensions.
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = std::max(fabs(speed.linear.x), fabs(speed.linear.z)) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}
 
geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,    // Current position in map frame
      const geometry_msgs::msg::Twist & speed)         // Current velocity in FLU orientation
{
  RCLCPP_INFO(logger_, "Current pos [ %.2f, %.2f, %.2f] frame %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.header.frame_id.c_str() );
  
  // Find look ahead distance and point on path
  double lookahead_dist = getLookAheadDistance(speed);
  auto lookahead_pose = getLookAheadPoint(pose, lookahead_dist);

  //Hack needed during debugging.  Why is the frame empty?
  RCLCPP_INFO(logger_, "Transforming from [%s]", lookahead_pose.header.frame_id.c_str()); 
  lookahead_pose.header.frame_id = "map";
  
  // let's get the lookahead pose in the robot frame
  geometry_msgs::msg::PoseStamped carrot_pose;
  if (!nav_drone_util::transformPoseInTargetFrame(lookahead_pose, carrot_pose, *tf_, "base_link", transform_tolerance_)) {
    throw nav_drone_core::ControllerTFError("Unable to transform lookahead pose into robot's frame");
  }  
  RCLCPP_INFO(logger_, "carrot [ %.2f, %.2f, %.2f] lookahead_dist %.2f", carrot_pose.pose.position.x, carrot_pose.pose.position.y, carrot_pose.pose.position.z, lookahead_dist);

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
  
  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }
    
  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!nav_drone_util::transformPoseInTargetFrame(global_plan_.poses.back(), goal_pose, *tf_, "base_link", transform_tolerance_)) {
      throw nav_drone_core::ControllerTFError("Unable to transform goal pose into robot's frame");
    }      
    // Should I not calculate the yaw to the goal, instead of the yaw of the goal??????
    double angle_to_goal = nav_drone_util::getYaw(goal_pose);
    RCLCPP_INFO(logger_, "Rotating to goal : %.3f", angle_to_goal);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    RCLCPP_INFO(logger_, "Rotating to heading : %.3f", angle_to_heading);
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature,
      costAtPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), 
      pose, 
      linear_vel, sign);
    RCLCPP_INFO(logger_, "Flying");

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }
  
  // Scale vertical speed
  double altitude_error = carrot_pose.pose.position.z; // Remember, carrot_pose is in robot base frame  - pose.pose.position.z;
  double velocity_scaling_factor = 1.0;
  if (std::fabs(altitude_error) < approach_velocity_scaling_dist_) {
    velocity_scaling_factor = std::fabs(altitude_error) / approach_velocity_scaling_dist_;
  }  
  
  double vertical_vel;
  if (altitude_error < 0.0) {
    vertical_vel = -1.0 * desired_linear_vel_ * velocity_scaling_factor;
  } else {  
    vertical_vel = desired_linear_vel_ * velocity_scaling_factor;
  }
  
  // Collision checking on this velocity heading
  //const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  //if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
  //  throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
  //}
  
  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.linear.z = vertical_vel; 
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}
  
bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  
  double velocity_scaling_factor = 1.0;
  if (use_regulated_angular_velocity_scaling_ && (std::fabs(angle_to_path) < rotation_velocity_scaling_angle_)) {
    velocity_scaling_factor = std::fabs(angle_to_path) / rotation_velocity_scaling_angle_;
  }  

  angular_vel = sign * rotate_to_heading_angular_vel_*velocity_scaling_factor; 

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}
  
geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const geometry_msgs::msg::PoseStamped & pose,  // Current position in map frame
  const double & lookahead_dist)                       // Distance in meters
{ 
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(), [&](const auto & ps) {
      return nav_drone_util::euclidean_distance(pose, ps, true) >= lookahead_dist;
    });

  // If the pose is not far enough, take the last pose
  if (goal_pose_it == global_plan_.poses.end()) {
    goal_pose_it = std::prev(global_plan_.poses.end());
    // lookahead_dist = nav_drone_util::euclidean_distance(pose, *goal_pose_it, true);
  } else if (use_interpolation_) {
    // Find the point on the line segment between the current pose and the found point
    // that is exactly the lookahead distance away from the robot pose
    // This can be found with a closed form for the intersection of a segment and a sphere
    auto point = nav_drone_util::sphereSegmentIntersection(
      pose.pose.position,
      goal_pose_it->pose.position, pose.pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = pose.header.frame_id;
    pose.header.stamp = pose.header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;  
} 
    
double RegulatedPurePursuitController::costAtPose(const double & x, const double & y, const double & z)
{
  
  auto node = costmap_->search(x,y,z,0);  // 0 scans the whole octree. Should we scan to a set depth? maybe size of robot?
  
  if (node==NULL) {
    return static_cast<double>(NO_INFORMATION); // Unknown space
      
  if (costmap_->isNodeOccupied(node))
    return static_cast<double>(LETHAL_OBSTACLE);
  } 
  return static_cast<double>(FREE_SPACE);
    
}
  
double RegulatedPurePursuitController::approachVelocityScalingFactor(const geometry_msgs::msg::PoseStamped & pose) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav_drone_util::calculate_path_length(global_plan_);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = global_plan_.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.    
    double distance_to_last_pose = nav_drone_util::euclidean_distance(pose, last, true);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const geometry_msgs::msg::PoseStamped & pose,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(pose);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}  

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature,  const double & pose_cost, 
  const geometry_msgs::msg::PoseStamped & pose, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius_;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  applyApproachVelocityScaling(pose, linear_vel);
  
  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  double x, y, z;
  
  costmap_->getMetricSize(x, y, z);
  
//  const double max_costmap_dim_meters = std::max(
//    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return std::max(x,y) / 2.0;
}  
  
  
  
}  // namespace  nav_drone_regulated_pure_pursuit_controller

// Register this controller as a nav_drone_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav_drone_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav_drone_core::Controller)