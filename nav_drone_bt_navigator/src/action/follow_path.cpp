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

#include "nav_drone_bt_navigator/action/follow_path.h"

namespace NavigationNodes
{
 
BT::NodeStatus NavDroneFollowPathAction::tick()
{
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
   
  BT::Optional<std::vector<Pose3D>> msg = getInput<std::vector<Pose3D>>("path");
  BT::Optional<std::string> controller = getInput<std::string>("controller");
  
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [path]: ", 
                             msg.error() );
  }
  
  if (!controller)
  {
    controller = "PurePursuitController";
  }
  

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot address the controller server.  Failing.");
  }
  
  // Call the action server
  auto now = node_->get_clock()->now();
  auto goal_msg = FollowPath::Goal();
  goal_msg.controller_id = controller.value();  
  goal_msg.path.header.stamp = now;
  goal_msg.path.header.frame_id = "map";  
  for(size_t i = 0; i < msg.value().size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = msg.value()[i].x;
    pose.pose.position.y = msg.value()[i].y;
    pose.pose.position.z = msg.value()[i].z;
  
    tf2::Quaternion q;
    q.setRPY( 0, 0, msg.value()[i].theta );  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
  
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];
  
    goal_msg.path.poses.push_back(pose);
  }  
  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavDroneFollowPathAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavDroneFollowPathAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavDroneFollowPathAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleFollowPath::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));
  
  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void NavDroneFollowPathAction::cleanup() 
{

  if( _halt_requested )
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    try {
      goal_handle_ = future_goal_handle_->get();
      this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "[%s] - Exception caught");
    }
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
}
  
void NavDroneFollowPathAction::halt() 
{
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
  
}
  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void NavDroneFollowPathAction::goal_response_callback(std::shared_future<GoalHandleFollowPath::SharedPtr> future)
  {
    //future_goal_handle_ = future;
  
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Goal passes from behaviour [action_follow_waypoints] and  accepted by server controller server, Waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void NavDroneFollowPathAction::feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
  {
    RCLCPP_DEBUG(node_->get_logger(), 
      "Distance to goal: %.2fm  Speed: %2f m/s", 
      feedback->distance_to_goal, 
      feedback->speed);
  }

  void NavDroneFollowPathAction::result_callback(const GoalHandleFollowPath::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_DEBUG(node_->get_logger(), "Navigation path completed successfully.");
        action_status = ActionStatus::SUCCEEDED;
//        } else {
//          action_status = ActionStatus::FAILED;
//        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
        action_status = ActionStatus::ABORTED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
        action_status = ActionStatus::CANCELED;
        return;
      default:
        RCLCPP_WARN(node_->get_logger(), "Unknown result code");
        action_status = ActionStatus::UNKNOWN;
        return;
    }
    
  }  
  
}  // namespace