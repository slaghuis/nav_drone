#pragma once

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav_drone_bt_navigator/action_status.h"
#include "nav_drone_bt_navigator/pose_3D.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

class NavDroneReadGoalAction : public BT::SyncActionNode
{
  public:
    NavDroneReadGoalAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // A node having ports MUST implement this STATIC method
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<Pose3D>("pose") };
    }
    
    BT::NodeStatus tick() override;
   
    void init(rclcpp::Node::SharedPtr node, Pose3D goal) {
      node_ = node;
      goal_ = goal;
    }
    
  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    Pose3D goal_;
};
       
} // Namespace