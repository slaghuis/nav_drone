#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav_drone_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/geometry_utils.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;

namespace nav_drone_controller
{

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void SimpleGoalChecker::initialize(
  const rclcpp::Node::SharedPtr parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent;

  nav_drone_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav_drone_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav_drone_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

}

void SimpleGoalChecker::reset()
{
  check_xy_ = true;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_goal_tolerance_;
}

bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav_drone_util::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav_drone_controller::SimpleGoalChecker, nav_drone_core::GoalChecker)