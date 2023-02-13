#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_drone_core/goal_checker.hpp"
#include "nav_drone_controller/plugins/simple_goal_checker.hpp"

namespace nav_drone_controller
{

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that only checks the position difference and velocity
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
 */
class StoppedGoalChecker : public SimpleGoalChecker
{
public:
  StoppedGoalChecker();
  // Standard GoalChecker Interface
  void initialize(
    const rclcpp::Node::SharedPtr parent,
    const std::string & plugin_name) override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double rot_stopped_velocity_, trans_stopped_velocity_;
  std::string plugin_name_;
};

}  // namespace nav_drone_controller
