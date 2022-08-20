// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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
#include <memory>

#include "nav_drone_util/robot_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav_drone_util
{
  
// Returns the yaw from a quaternion
// Rather use tf2::getYaw(poses.pose.orientation);
double getYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  double roll, pitch, yaw;
  
  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w );
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  
  return yaw;
}	
  

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  return transformPoseInTargetFrame(
    global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}
  
bool getCurrentPose(
  double &x, double &y, double &z, double &w,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp) {
  
  geometry_msgs::msg::PoseStamped global_pose;
  
  if (getCurrentPose(global_pose, tf_buffer, global_frame,
                     robot_frame, transform_timeout, stamp) ) {
    x = global_pose.pose.position.x; 
    y = global_pose.pose.position.y; 
    z = global_pose.pose.position.z;    
    w = getYaw(global_pose.pose.orientation);
    
    return true;
  } 
  return false;
}  

bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose = tf_buffer.transform(
      input_pose, target_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}

  double calculate_path_length( const nav_msgs::msg::Path path, const size_t current_idx ) 
  {
    double distance = 0.0;
    for(size_t i = current_idx; i < path.poses.size()-1; i++) {
      distance += euclidean_distance( path.poses[i], path.poses[i+1] );
    }
    
    return distance;
  }
  
  size_t find_closest_goal_idx(const geometry_msgs::msg::PoseStamped pose, const nav_msgs::msg::Path path)
  {
    size_t closest_pose_idx = 0;
    double curr_min_dist = std::numeric_limits<double>::max();
    
    for (size_t curr_idx = 0; curr_idx < path.poses.size(); ++curr_idx) {
      double curr_dist = euclidean_distance( pose, path.poses[curr_idx]);
      if (curr_dist < curr_min_dist) {
        curr_min_dist = curr_dist;
        closest_pose_idx = curr_idx;
      }
    }
    return closest_pose_idx;
  } 
}