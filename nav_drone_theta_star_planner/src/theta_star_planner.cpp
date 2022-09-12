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
 * Planning algorith: Theta Star
 * *********************************************************/
#include <nav_drone_core/planner.hpp>
#include <nav_drone_util/node_utils.hpp>
#include <nav_drone_theta_star_planner/costmap_adaptor.hpp>
#include <cmath>

namespace nav_drone_theta_star_planner
{
  class ThetaStarPlanner : public nav_drone_core::Planner
  {
    public:
      void configure(const rclcpp::Node::SharedPtr parent, 
                     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                     std::shared_ptr<octomap::OcTree> costmap ) override
      {
        node_ = parent;
        plugin_name_ = name;
        tf_buffer_ = tf;
        costmap_ = costmap;
        
        logger_ = node_->get_logger();
        clock_ = node_->get_clock();
    
        nav_drone_util::declare_parameter_if_not_declared(
          node_, plugin_name_ + ".weight", rclcpp::ParameterValue(100.0));
          
        node_->get_parameter(plugin_name_ + ".weight", weight_);     
      }
      
      void updateMap(std::shared_ptr<octomap::OcTree> costmap) override
      {
        costmap_ = costmap;
      }
      
      nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                      const geometry_msgs::msg::PoseStamped & goal) override
      {            
        RCLCPP_DEBUG(logger_, "Requested ThetaStar to plan a path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
        start.pose.position.x,
        start.pose.position.y,
        start.pose.position.z,
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z);
        
        nav_msgs::msg::Path global_path;
        
        // Instantiating our costmap adaptor.  
        // If for example the octree has a depth of 16 and a resolution of 0.2m then a depth of 14 would
        // render a granularity of 0.2*2^(16-14) = 0.8m
        CostmapAdaptor adaptor(costmap_, 14);
        // ... and the path finder
        Pathfinder pathfinder(adaptor, weight_);    //Weight is used to tune search performance
    
        octomap::point3d plan_start(start.pose.position.x, start.pose.position.y, start.pose.position.z);
        octomap::point3d plan_goal(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);   
        
        auto found_path = pathfinder.search( plan_start, plan_goal );
    
        rclcpp::Time now = node_->get_clock()->now();
        global_path.header.stamp = now;
        global_path.header.frame_id = start.header.frame_id;
    
        // If the list only has one member, (typcally the start) no valid path could be found.
        // at least the start and the destination is required to be a valid path.
        for(const auto& waypoint : found_path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = now;
          pose.header.frame_id = start.header.frame_id;
          pose.pose.position.x = (double) waypoint.x();
          pose.pose.position.y = (double) waypoint.y();
          pose.pose.position.z = (double) waypoint.z();      
          global_path.poses.push_back(pose);    
        }
        
        return global_path;
      }
      
    protected:
      rclcpp::Node::SharedPtr node_;
      std::string plugin_name_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<octomap::OcTree> costmap_;
      rclcpp::Logger logger_ {rclcpp::get_logger("ThetaStar")};
      rclcpp::Clock::SharedPtr clock_;
      
      float weight_;
  };

}  // namespace nav_drone_theta_star_planner

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav_drone_theta_star_planner::ThetaStarPlanner, nav_drone_core::Planner)
