// Copyright (c) 2019 Intel Corporation
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

#include "nav_drone_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
//#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace nav_drone_controller
{

void SimpleProgressChecker::initialize(
  const rclcpp::Node::SharedPtr parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent;

  clock_ = node->get_clock();

  nav_drone_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose.pose))) {
    reset_baseline_pose(current_pose.pose);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void SimpleProgressChecker::reset_baseline_pose(const geometry_msgs::msg::Pose & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose & pose)
{
  return nav_drone_util::euclidean_distance(pose, baseline_pose_) > radius_;
}


}  // namespace nav_drone_controller

PLUGINLIB_EXPORT_CLASS(nav_drone_controller::SimpleProgressChecker, nav_drone_core::ProgressChecker)