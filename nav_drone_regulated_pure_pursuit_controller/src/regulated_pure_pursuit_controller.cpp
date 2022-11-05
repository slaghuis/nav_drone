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

/* *******************************************************
 * Plugin to the navigation_lite controller server
 * Control algorith: Regulated Pure Pursuit Controller
 * ******************************************************* */
#include <math.h>     // fabs
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

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update
  
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
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
    rclcpp::ParameterValue(true));
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
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".pid_z",
    rclcpp::ParameterValue(std::vector<double>{0.7, 0.0, 0.0}));
  
  
  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);

  node_->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node_->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
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

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
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
  
  rclcpp::Parameter pid_z_settings_param = node_->get_parameter(plugin_name_ + ".pid_z");
  std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
  pid_z   = std::make_shared<PID>(control_duration_, desired_linear_vel_, -desired_linear_vel_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);
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
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}
 
geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,    // Current position in map frame
      const geometry_msgs::msg::Twist & speed)         // Current velocity in FLU orientation
{
  
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  // Skip some code here :-)
  
  
  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

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
    double angle_to_goal = nav_drone_util::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
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
  cmd_vel.twist.linear.z = pid_z->calculate(carrot_pose.pose.position.z, 0);
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
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

/*
bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & carrot_dist)

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
*/
  
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

/*
double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
*/
  

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
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

  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
//  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
  if (speed_limit == NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav_drone_core::InvalidPath("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav_drone_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav_drone_util::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav_drone_util::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return nav_drone_util::euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return nav_drone_util::euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose("base_link", stamped_pose, transformed_pose)) {   // place base_frae id in a parameter
        throw nav_drone_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = "base_link";  // place this in a parameter  costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  //global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav_drone_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  double x, y, z;
  
  costmap_->getMetricSize(x, y, z);
  
//  const double max_costmap_dim_meters = std::max(
//    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max(x,y) / 2.0;
}  
  
  
  
}  // namespace  nav_drone_regulated_pure_pursuit_controller

// Register this controller as a nav_drone_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav_drone_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav_drone_core::Controller)