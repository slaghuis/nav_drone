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

/* *********************************************************
 * Plugin to the Nav_Drone Planner Server
 * Planning algorith: None.  Simply return the start and 
 * the provided goal as a result
 * *********************************************************/
#include <nav_drone_core/planner.hpp>
#include <cmath>

namespace nav_drone_dumb_planner
{
  class DumbPlanner : public nav_drone_core::Planner
  {
    public:
      void configure(const rclcpp::Node::SharedPtr parent, 
                     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                     std::shared_ptr<octomap::OcTree> costmap ) override
      {
        parent_node_ = parent;
        name_ = name;
        tf_buffer_ = tf;
        costmap_ = costmap;    
      }
      
      void updateMap(std::shared_ptr<octomap::OcTree> costmap) override
      {
        costmap_ = costmap;
      }
      
      nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                      const geometry_msgs::msg::PoseStamped & goal) override
      {            
        rclcpp::Time now = parent_node_->get_clock()->now();
        
        RCLCPP_DEBUG(parent_node_->get_logger(), "Requested No Planner to return a path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
        start.pose.position.x,
        start.pose.position.y,
        start.pose.position.z,
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z);
        
        
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = now;
        global_path.header.frame_id = start.header.frame_id;
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = start.header.frame_id;
        pose.pose = start.pose;
        global_path.poses.push_back(pose);
        
        pose.pose = goal.pose;
        global_path.poses.push_back(pose);
  
        return global_path;
      }
      
    protected:
      double side_length_;
      rclcpp::Node::SharedPtr parent_node_;
      std::string name_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<octomap::OcTree> costmap_;
  };

}  // namespace nav_drone_dumb_planner

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav_drone_dumb_planner::DumbPlanner, nav_drone_core::Planner)
