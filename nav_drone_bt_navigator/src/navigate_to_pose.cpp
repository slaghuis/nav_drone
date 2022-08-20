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

/* **********************************************************************
 * An Action Server Node that forms the main interface to the navigation 
 * stack.  It executes a behaviour tree specified in the Action Server 
 * goal (xml file name).  The action server serves AsyncActionNodes that
 * call Planner, Controller and Recovery Action Servers and Simple Services
 * to move the drone safely in 3D space.
 *
 * Transform listener tf2 for odom->base_link transforms.
 * **********************************************************************/

#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <sstream>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "nav_drone_util/visibility_control.h"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/robot_utils.hpp"
#include "nav_drone_util/angle_utils.hpp"

#include "nav_drone_msgs/action/navigate_to_pose.hpp"

#include "nav_drone_bt_navigator/action/compute_path_to_pose.h"
#include "nav_drone_bt_navigator/action/follow_path.h"
#include "nav_drone_bt_navigator/action/follow_waypoints.h"
#include "nav_drone_bt_navigator/action/read_goal.h"
//#include "nav_drone_bt_navigator/action/spin.h"
//#include "nav_drone_bt_navigator/action/wait.h"

#include "nav_drone_bt_navigator/control/round_robin.h"
#include "nav_drone_bt_navigator/control/pipeline_sequence.h"
#include "nav_drone_bt_navigator/control/recovery.h"
#include "nav_drone_bt_navigator/decorator/rate_controller.h"
#include "nav_drone_bt_navigator/action_status.h"
#include "nav_drone_bt_navigator/pose_3D.h"

using namespace std::chrono_literals;
using namespace BT;

namespace nav_drone
{
class NavigationServer : public rclcpp::Node
{
public:
  using NavigateToPose = nav_drone_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  NAV_DRONE_PUBLIC
  explicit NavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigation_server", options)
  {
    using namespace std::placeholders;
    
    // Read parameters
    nav_drone_util::declare_parameter_if_not_declared(
      this, "map_frame", rclcpp::ParameterValue("map"));
    nav_drone_util::declare_parameter_if_not_declared(
      this, "minimum_battery_voltage", rclcpp::ParameterValue(13.6));  
 
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("minimum_battery_voltage", minimum_battery_voltage_);
    current_battery_voltage_ = 14.8;  // Full LiPo S4
    
    // Subscribe to some topics
    subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "drone/battery", 5, std::bind(&NavigationServer::battery_callback, this, std::placeholders::_1));
    
    // Create a transform listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "nav_drome/navigate_to_pose",
      std::bind(&NavigationServer::handle_goal, this, _1, _2),
      std::bind(&NavigationServer::handle_cancel, this, _1),
      std::bind(&NavigationServer::handle_accepted, this, _1));
      
      RCLCPP_DEBUG(this->get_logger(), "Action Serever [nav_drone/navigate_to_pose] started");
  }
  
private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  
  std::string map_frame_;
  float minimum_battery_voltage_;
  float current_battery_voltage_;

  BT::NodeStatus CheckBattery()
  {      
    return (current_battery_voltage_ >= minimum_battery_voltage_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }  
  
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) 
  {
     current_battery_voltage_ = msg->voltage;
  }
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
    
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received request with behaviour tree %s",goal->behavior_tree.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Received goal request to fly to [%.2f; %.2f; %.2f]", goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&NavigationServer::execute, this, _1), goal_handle}.detach();
  }

  /* std::string pose3D_string(Pose3D bt_goal_msg) {
    std::stringstream ss;
    ss << bt_goal_msg.x << ";" << bt_goal_msg.y << ";" << bt_goal_msg.z << ";" << bt_goal_msg.theta;
    return ss.str();
  }
  */
  
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1);    // Executing at 1 Hz
    const auto goal = goal_handle->get_goal();
    const auto wp = goal->pose;
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto & current_pose = feedback->current_pose;
    auto & navigation_time = feedback->navigation_time;
    auto & estimated_time_remaining = feedback->estimated_time_remaining;
    auto & number_of_recoveries = feedback->number_of_recoveries;
    auto & distance_remaining = feedback->distance_remaining;
    auto result = std::make_shared<NavigateToPose::Result>();
    
    Pose3D bt_goal_msg;
    bt_goal_msg.x     = wp.pose.position.x;
    bt_goal_msg.y     = wp.pose.position.y;
    bt_goal_msg.z     = wp.pose.position.z;
    bt_goal_msg.theta = nav_drone_util::getYaw(wp.pose.orientation);
            
    BehaviorTreeFactory factory;
    Tree tree;
    using namespace NavigationNodes; 
    factory.registerSimpleCondition("BatteryOK", std::bind(&NavigationServer::CheckBattery, this));
    factory.registerNodeType<RoundRobinNode>("RoundRobin"); 
    factory.registerNodeType<PipelineSequence>("PipelineSequence"); 
    factory.registerNodeType<RecoveryNode>("RecoveryNode"); 
    factory.registerNodeType<RateController>("RateController");
    factory.registerNodeType<NavDroneReadGoalAction>("ReadGoal");  //  10,0;1,0;5,0;0.0
//    factory.registerNodeType<NavDroneWaitAction>("Wait");
//    factory.registerNodeType<NavDroneSpinAction>("Spin");
    factory.registerNodeType<NavDroneFollowPathAction>("FollowPath");
    factory.registerNodeType<NavDroneComputePathToPoseAction>("ComputePathToPose");

    //BT::Blackboard::Ptr blackboard;
    //tree = factory.createTreeFromFile(goal->behavior_tree, blackboard);    
    //blackboard->set("pose", pose3D_string(bt_goal_msg));
    tree = factory.createTreeFromFile(goal->behavior_tree);
    
    RCLCPP_DEBUG(this->get_logger(), "Tree Loaded");
    
    auto node_ptr = shared_from_this();              
    // Initialise the BT Nodes  
    // Iterate through all the nodes and call init() if it is an Action_B
    for( auto& node: tree.nodes )
    {
      // Not a typo: it is "=", not "=="
      if( auto read_goal_action = dynamic_cast<NavDroneReadGoalAction *>( node.get() ))
      {
        read_goal_action->init( node_ptr, bt_goal_msg );
//      } else if( auto wait_action = dynamic_cast<NavDroneWaitAction *>( node.get() ))
//      {
//        wait_action->init( node_ptr );
//      } else if( auto spin_action = dynamic_cast<NavDroneSpinAction *>( node.get() ))
//      {
//        spin_action->init( node_ptr );
      } else if( auto follow_path_action = dynamic_cast<NavDroneFollowPathAction *>( node.get() ))
      {
        follow_path_action->init( node_ptr );
      } else if( auto compute_path_to_pose_action = dynamic_cast<NavDroneComputePathToPoseAction *>( node.get() ))
      {
        compute_path_to_pose_action->init( node_ptr );
      }
    }
        
    auto start_time = now();
    
    while( ( tree.tickRoot() == NodeStatus::RUNNING) && rclcpp::ok() )
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        // Result is std_msgs/Empty.  Send nothing
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      
      // Publish Feedback
      nav_drone_util::getCurrentPose(current_pose, *tf_buffer_);
      
      navigation_time = now() - start_time;                         // builtin_interfaces/Duration navigation_time
      estimated_time_remaining.sec = 0;                             // builtin_interfaces/Duration estimated_time_remaining 
      estimated_time_remaining.nanosec = 0;
      number_of_recoveries = 0;                                     // int16 number_of_recoveries
      
      distance_remaining = nav_drone_util::euclidean_distance(current_pose, wp, true);
      
      goal_handle->publish_feedback(feedback);
      
      loop_rate.sleep();
    }
    
    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Goal succeeded");
    }
  }
     
};  // class NavigationServer

}  // namespace nav_drone

RCLCPP_COMPONENTS_REGISTER_NODE(nav_drone::NavigationServer)