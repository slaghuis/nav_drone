#pragma once

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>  //Dealing with a vector of poses to define waypoints

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/path.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav_drone_msgs/action/compute_path_to_pose.hpp"

#include "nav_drone_bt_navigator/action_status.h"
#include "nav_drone_bt_navigator/pose_3D.h"

namespace NavigationNodes
{

class NavDroneComputePathToPoseAction : public BT::AsyncActionNode
{
  public:
    using ComputePathToPose = nav_drone_msgs::action::ComputePathToPose;
    using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    NavDroneComputePathToPoseAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<ComputePathToPose>(
       node_,
      "nav_drone/compute_path_to_pose");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose3D>("pose"), 
                BT::InputPort<std::string>("planner"),
                BT::InputPort<Pose3D>("start"), 
                BT::OutputPort<std::vector<Pose3D>>("path")};
        
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
    void cleanup();

  private:
    std::atomic_bool _halt_requested;
    NavigationNodes::ActionStatus action_status;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    
    rclcpp_action::Client<ComputePathToPose>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleComputePathToPose::SharedPtr>> future_goal_handle_;
    GoalHandleComputePathToPose::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleComputePathToPose::SharedPtr> future);
    void result_callback(const GoalHandleComputePathToPose::WrappedResult & result);
    
        
    // Saved Results for feedback
    std::stringstream returned_path;

};
     
} // Namespace