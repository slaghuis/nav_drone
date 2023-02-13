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

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav_drone_util/angle_utils.hpp"

#include "nav_drone_core/exceptions.hpp"
#include "nav_drone_core/controller_exceptions.hpp"

#include "nav_drone_core/controller.hpp"
#include "nav_drone_core/progress_checker.hpp"
#include "nav_drone_core/goal_checker.hpp"

#include "nav_drone_costmap_3d/costmap_server.hpp"

#include "nav_drone_msgs/action/follow_path.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/node_thread.hpp"
#include "nav_drone_util/robot_utils.hpp"
#include "nav_drone_util/visibility_control.h"
#include "nav_drone_util/tf_help.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

namespace nav_drone_controller
{
  
class ProgressChecker;
  
class ControllerServer : public rclcpp::Node
{
public:
  using ControllerMap = std::unordered_map<std::string, nav_drone_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav_drone_core::GoalChecker::Ptr>;

  NAV_DRONE_PUBLIC
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
   /**
   * @brief Destructor for nav_drone_controller::ControllerServer
   */
  ~ControllerServer();
protected:
  /**
   * @brief Initialises member variables
   *
   * Initialises controller, costmap, velocity publisher and follow path action
   * server
   */
  void init();
  
  using FollowPath = nav_drone_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

 // Our action server implements the FollowPath action
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle);
    
  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  
  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  
    /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(
    const std::string & c_name,
    std::string & current_controller); 
  
  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */  
  bool findGoalCheckerId(
    const std::string & c_name,
   std::string & current_goal_checker);

  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  bool publishZeroVelocity(); 

  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
    
    /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }
  
   /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  geometry_msgs::msg::Twist getThresholdedTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::Twist twist_thresh;
    twist_thresh.linear.x = getThresholdedVelocity(twist.linear.x, min_x_velocity_threshold_);
    twist_thresh.linear.y = getThresholdedVelocity(twist.linear.y, min_y_velocity_threshold_);
    twist_thresh.linear.z = getThresholdedVelocity(twist.linear.z, min_y_velocity_threshold_);
    twist_thresh.angular.z = getThresholdedVelocity(twist.angular.z, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  // The controller needs a costmap node
  std::shared_ptr<nav_drone_costmap_3d::CostmapPublisher> costmap_ros_;
  std::unique_ptr<nav_drone_util::NodeThread> costmap_thread_;  

  //Publishers and Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav_drone_core::ProgressChecker> progress_checker_loader_;
  nav_drone_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav_drone_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;
  
  // Controller plugins
  pluginlib::ClassLoader<nav_drone_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  // Variables for node paramaters
  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_z_velocity_threshold_;
  double min_theta_velocity_threshold_;
  
  double max_controller_duration_;

  // Utility global variales
  std::mutex server_mutex;   // Only allow one Action Server to address the drone at a time  
  geometry_msgs::msg::TwistStamped last_velocity_;
  geometry_msgs::msg::PoseStamped end_pose_;
  
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  
private:
  
};  // class ControllerServer

}  // namespace nav_drone_controller
