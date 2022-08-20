// Copyright (c) 2021, 2022 Eric Slaghuis
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

#include "nav_drone_bt_navigator/action/compute_path_to_pose.h"

namespace NavigationNodes
{
 
BT::NodeStatus NavDroneComputePathToPoseAction::tick()
{
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
   
  BT::Optional<Pose3D> msg = getInput<Pose3D>("pose");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [pose]: ", 
                             msg.error() );
  }
  
  BT::Optional<std::string> planner = getInput<std::string>("planner");
  // Check if optional is valid. If not, throw its error
  if (!planner)
  {
      //throw BT::RuntimeError("missing required input [planner]: ", 
      //                       msg.error() );
      planner = "ThetaStar";  //Set as a default
  }

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot address the controller server.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = ComputePathToPose::Goal();
  goal_msg.use_start = false;
  goal_msg.planner_id = planner.value();
  
  goal_msg.goal.header.stamp = node_->now();
  goal_msg.goal.header.frame_id = "map";  
  goal_msg.goal.pose.position.x = msg.value().x;
  goal_msg.goal.pose.position.y = msg.value().y;
  goal_msg.goal.pose.position.z = msg.value().z;
  
  tf2::Quaternion q;
  q.setRPY( 0, 0, msg.value().theta );  // Create this quaternion from roll/pitch/yaw (in radians)
  q.normalize();
  
  goal_msg.goal.pose.orientation.x = q[0];
  goal_msg.goal.pose.orientation.y = q[1];
  goal_msg.goal.pose.orientation.z = q[2];
  goal_msg.goal.pose.orientation.w = q[3];
  
  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavDroneComputePathToPoseAction::goal_response_callback, this, _1);
  send_goal_options.result_callback =
      std::bind(&NavDroneComputePathToPoseAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleComputePathToPose::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));
  
  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  // Formulate a result
  setOutput("path", returned_path.str().c_str());
    
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void NavDroneComputePathToPoseAction::cleanup() 
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
  
void NavDroneComputePathToPoseAction::halt() 
{
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
  
}
  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void NavDroneComputePathToPoseAction::goal_response_callback(std::shared_future<GoalHandleComputePathToPose::SharedPtr> future)
  {  
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Goal accepted by server, Waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void NavDroneComputePathToPoseAction::result_callback(const GoalHandleComputePathToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result->path.poses.size() == 0)
        { 
          RCLCPP_ERROR(node_->get_logger(), "No valid path was returned");
          
          action_status = ActionStatus::FAILED;  // No path could be found.  SLAM is the only way out of this mess.
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "Navigation path calculated successfully.");
          
          // Build out the response.  x;y;z;theta float combinations seperated by |
          for(size_t i=0; i<result.result->path.poses.size(); i++) {
            if( i > 0 )
            {
              returned_path << "|";
            }
            returned_path << result.result->path.poses[i].pose.position.x << ";";
            returned_path << result.result->path.poses[i].pose.position.y << ";";  
            returned_path << result.result->path.poses[i].pose.position.z << ";";             
            
            // Calculate yaw
            // Orientation quaternion
            tf2::Quaternion q(
            result.result->path.poses[i].pose.orientation.x,
            result.result->path.poses[i].pose.orientation.y,
            result.result->path.poses[i].pose.orientation.z,
            result.result->path.poses[i].pose.orientation.w);

            // 3x3 Rotation matrix from quaternion
            tf2::Matrix3x3 m(q);

            // Roll Pitch and Yaw from rotation matrix
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            returned_path << yaw;
          }
        }
          
        action_status = ActionStatus::SUCCEEDED;  
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