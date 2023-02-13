#include <memory>
#include <cmath>
#include <string>
#include <limits>
#include <vector>
#include "nav_drone_controller/plugins/stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_drone_util/node_utils.hpp"

using std::placeholders::_1;

namespace nav_drone_controller
{

StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(
  const rclcpp::Node::SharedPtr parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  SimpleGoalChecker::initialize(parent, plugin_name);
  
  auto node = parent;

  nav_drone_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav_drone_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
}

bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

bool StoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  bool rtn = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = trans_stopped_velocity_;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

}  // namespace nav_drone_controller

PLUGINLIB_EXPORT_CLASS(nav_drone_controller::StoppedGoalChecker, nav_drone_core::GoalChecker)