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

#pragma once

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_drone_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_drone_controller
{
/**
* @class SimpleProgressChecker
* @brief This plugin is used to check the position of the robot to make sure
* that it is actually progressing towards a goal.
*/

class SimpleProgressChecker : public nav_drone_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp::Node::SharedPtr parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool is_robot_moved_enough(const geometry_msgs::msg::Pose & pose);
  /**
   * @brief Resets baseline pose with the current pose of the robot
   * @param pose Current pose of the robot
   */
  void reset_baseline_pose(const geometry_msgs::msg::Pose & pose);

  rclcpp::Clock::SharedPtr clock_;

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose baseline_pose_;
  rclcpp::Time baseline_time_;

  bool baseline_pose_set_{false};
  std::string plugin_name_;

};
}  // namespace nav_drone_controller