#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace nav_drone_core
{
  class Controller
  {
    public:
      using Ptr = std::shared_ptr<Controller>;
    
      virtual void configure(const rclcpp::Node::SharedPtr parent, 
                             std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<octomap::OcTree> costmap ) = 0;
    
      virtual void setPath(const nav_msgs::msg::Path & path) = 0;
      virtual void updateMap(std::shared_ptr<octomap::OcTree> costmap) = 0;
    
      virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & speed) = 0;
//        nav2_core::GoalChecker * goal_checker) = 0;
    
      virtual ~Controller() {}

    protected:
      Controller(){}
  };
} // namespace nav_drone
