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

/* **********************************************************************
 * Subscribe to tf2 map->base_link for position and pose data
 * Subscribe to nav_lite/map [octomap_msgs::msg::Octomap] 
 *
 * Action Server responding to nav_drone_msgs/action/FollowPath
 *   typically called only by the Navigation Server
 * Publishes cmd_vel as geometry_msgs/msg/Twist to effect motion.
 * The motion strategy would be determined by the specified PLUGIN.
 *
 * ***********************************************************************/
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <unordered_map>
#include <iterator>
#include <vector>
#include <limits>       // std::numeric_limits
#include <cmath>        // std::hypot
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "nav_drone_msgs/action/follow_path.hpp"

#include "nav_drone_util/angle_utils.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/robot_utils.hpp"
#include "nav_drone_util/visibility_control.h"
#include "nav_drone_util/holddown_timer.hpp"
#include "nav_drone_core/exceptions.hpp"
#include <nav_drone_core/controller.hpp>

#include <pluginlib/class_loader.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include "octomap_msgs/msg/octomap.hpp"

static const float DEFAULT_WAYPOINT_RADIUS_ERROR = 0.3; // Acceptable XY distance to waypoint deemed as close enough
static const float DEFAULT_YAW_THRESHOLD = 0.025;       // Acceptible YAW deemed as close enough
static const int DEFAULT_HOLDDOWN = 2;                  // Time to ensure stability in flight is attained

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav_drone
{
  
class ControllerServer : public rclcpp::Node
{
public:
  using FollowPath = nav_drone_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
  
  using ControllerMap = std::unordered_map<std::string, nav_drone_core::Controller::Ptr>;

  NAV_DRONE_PUBLIC
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controller_server", options),
    loader_("nav_drone_core", "nav_drone_core::Controller"),
    default_ids_{"PIDController", "RegulatedPurePursuitController"},
    default_types_{"nav_drone_pid_controller/PIDController", "nav_drone_regulated_pure_pursuit_controller/RegulatedPurePursuitController"}
//    default_ids_{"MPCController", "PIDController", "PurePursuitController"},
//    default_types_{"nav_drone_mpc_controller/MPCController", "nav_drone_pid_controller/PIDController", "nav_drone_regulated_pure_pursuit_controller/RegulatedPurePursuitController"}
  {
  
    // Create a transform listener
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Declare and get parameters    
    nav_drone_util::declare_parameter_if_not_declared(
      this, "control_frequency", rclcpp::ParameterValue(10.0));   // Control frequency in Hz.  Must be bigger than 2 Hz
    nav_drone_util::declare_parameter_if_not_declared(
      this, "waypoint_radius_error", rclcpp::ParameterValue(DEFAULT_WAYPOINT_RADIUS_ERROR)); 
    nav_drone_util::declare_parameter_if_not_declared(
      this, "yaw_threshold", rclcpp::ParameterValue(DEFAULT_YAW_THRESHOLD)); 
    nav_drone_util::declare_parameter_if_not_declared(
      this, "holddown", rclcpp::ParameterValue(DEFAULT_HOLDDOWN));       
    nav_drone_util::declare_parameter_if_not_declared(
      this, "map_frame", rclcpp::ParameterValue("map"));
    nav_drone_util::declare_parameter_if_not_declared(
      this, "robot_base_frame", rclcpp::ParameterValue("base_link"));      
    nav_drone_util::declare_parameter_if_not_declared(
      this, "transform_tolerance", rclcpp::ParameterValue(0.1));      
    declare_parameter("controller_plugins", default_ids_);
    declare_parameter("controller_types", default_types_);  
      
    this->get_parameter("control_frequency", controller_frequency_);
    this->get_parameter("waypoint_radius_error", waypoint_radius_error_);
    this->get_parameter("yaw_threshold", yaw_threshold_);
    this->get_parameter("holddown", holddown_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("transform_tolerance", transform_tolerance_); 
    this->get_parameter("controller_plugins", controller_ids_);
    this->get_parameter("controller_types", controller_types_);
    
    one_off_timer_ = this->create_wall_timer(
      200ms, std::bind(&ControllerServer::init, this));
  }

protected:
  // Variables for node paramaters
  float waypoint_radius_error_;
  float yaw_threshold_;
  int holddown_;
  double controller_frequency_;
  double transform_tolerance_;
  std::string map_frame_;
  std::string robot_base_frame_;


  // Controller
  ControllerMap controllers_;
  pluginlib::ClassLoader<nav_drone_core::Controller> loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  double max_controller_duration_;
  std::string controller_ids_concat_, current_controller_;

  // Utility global variales
  std::mutex server_mutex;   // Only allow one Action Server to address the drone at a time  
  geometry_msgs::msg::TwistStamped last_velocity_;
  geometry_msgs::msg::PoseStamped end_pose_;
  std::shared_ptr<nav_drone_util::HolddownTimer> holddown_timer_;
  std::shared_ptr<octomap::OcTree> octomap_;
  
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::TimerBase::SharedPtr one_off_timer_;
    
private:
// VELOCITY PUBLISHER /////////////////////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  

// TF2 VARIABLES
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

void init()
{
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
       
    // Setup the controllers
    auto node = shared_from_this();

    for (size_t i = 0; i != controller_ids_.size(); i++) {
      try {
        nav_drone_core::Controller::Ptr controller =
          loader_.createUniqueInstance(controller_types_[i]);
        RCLCPP_INFO(
          get_logger(), "Created controller plugin %s of type %s",
          controller_ids_[i].c_str(), controller_types_[i].c_str());
          
        controller->configure(node, controller_ids_[i], tf_buffer_, octomap_);
        controllers_.insert({controller_ids_[i], controller});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          get_logger(), "Failed to create controller. Exception: %s",
          ex.what());
      }
    }

    for (size_t i = 0; i != controller_ids_.size(); i++) {
      controller_ids_concat_ += controller_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s controllers available.", controller_ids_concat_.c_str());
    
    holddown_timer_ = std::make_shared<nav_drone_util::HolddownTimer>(holddown_);    
    
    // Create drone velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);

    // ROS2 Subscriptions
    map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_drone/map", 10, std::bind(&ControllerServer::map_callback, this, _1));
      
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "drone/odom", 10, std::bind(&ControllerServer::odom_callback, this, _1));
  
    // Create the action server
    this->action_server_ = rclcpp_action::create_server<FollowPath>(
      this,
      "nav_drone/follow_path",
      std::bind(&ControllerServer::handle_goal, this, _1, _2),
      std::bind(&ControllerServer::handle_cancel, this, _1),
      std::bind(&ControllerServer::handle_accepted, this, _1));

}

// MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
  {
    // Convert ROS message to a OctoMap
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
    } 
  }
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
// ODOM SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    // The twist in the odometry message is published in a frame (FRD) with orientation matching the flight controller,
    // We need to tranform the speed to the robot base frame (FLU). Do that by flipping left <-->right, and up <--> down.  
    
    last_velocity_.twist.linear.x = msg->twist.twist.linear.x;
    last_velocity_.twist.linear.y = - msg->twist.twist.linear.y;
    last_velocity_.twist.linear.x = - msg->twist.twist.linear.z;
    last_velocity_.twist.angular.z = - msg->twist.twist.angular.z;
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

// FLIGHT CONTROL ////////////////////////////////////////////////////////////////////////////////////////////////
  bool stop_movement() 
  {    
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;  
    setpoint.linear.z = 0.0;  
    
    publisher_->publish(setpoint);        
    
    return true;
  }
  
// FollowWayPoint Action Server /////////////////////////////////////////////////////////////////
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal)
  {
    (void)uuid;
    if(goal->path.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid path, Path is empty.  Rejecting request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }
    RCLCPP_INFO(this->get_logger(), "Controller Server received request to follow %d waypoints", goal->path.poses.size());
    
    if(server_mutex.try_lock()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Another thread is commanding the drone now.  Rejecting request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }  
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto & distance_to_goal = feedback->distance_to_goal;
    auto & speed = feedback->speed;
    auto result = std::make_shared<FollowPath::Result>();
    
    auto start_time = this->now();
    end_pose_ = goal->path.poses.back();
    if (octomap_ == NULL) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the first map");
    }
    
    // Wait for the fist map to arrive
    rclcpp::Rate r(100);
    while ( (octomap_ == NULL) && !goal_handle->is_canceling() ) {
      r.sleep();
    }
    
    if (goal_handle->is_canceling()) {
      // result->planning_time = this->now() - start_time;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled before the first map has been received!");
      return;
    }
    
    try
    {    
      std::string c_name = goal->controller_id;
      std::string current_controller;
      if (findControllerId(c_name, current_controller)) {
        current_controller_ = current_controller;
      } else {
        throw nav_drone_core::DroneException("Failed to load the controller.");
      }
      
      controllers_[current_controller_]->setPath( goal->path );
      controllers_[current_controller_]->updateMap( octomap_ );
            
      rclcpp::WallRate loop_rate( controller_frequency_ );
      while ( rclcpp::ok() ) {
         // Check if there is a cancelling request
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          stop_movement();
          server_mutex.unlock();
          return;
        }
        
        // Compute and publish velocity
        geometry_msgs::msg::PoseStamped pose;   
        
        if (!nav_drone_util::getCurrentPose(pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_)) {
          throw nav_drone_core::DroneException("Failed to obtain robot pose.");
        }

        // Check if the robot is stuck here (for longer than a timeout)
        
        
        try {
          geometry_msgs::msg::TwistStamped setpoint;
          setpoint = controllers_[current_controller_]->computeVelocityCommands( pose, last_velocity_.twist);      

          RCLCPP_INFO(this->get_logger(), "Publishing velocity [%.2f, %.2f, %.2f, %.4f]", 
            setpoint.twist.linear.x, 
            setpoint.twist.linear.y, 
            setpoint.twist.linear.z,
            setpoint.twist.angular.z);
            
          publisher_->publish( setpoint.twist ); 

          // Publish feedback
          speed = std::hypot( double(last_velocity_.twist.linear.x), double(last_velocity_.twist.linear.y), double(last_velocity_.twist.linear.z) );
          size_t current_idx = nav_drone_util::find_closest_goal_idx( pose, goal->path); 
          distance_to_goal = nav_drone_util::calculate_path_length( goal->path, current_idx );
          goal_handle->publish_feedback(feedback);
        } catch (nav_drone_core::DroneException & e) {
          RCLCPP_WARN(this->get_logger(), e.what());
        }

        
        if (is_goal_reached()) {
          break;
        }  
                                
        if (!loop_rate.sleep()) {
          RCLCPP_WARN(
            get_logger(), "Control loop missed its desired rate of %.4fHz",
            controller_frequency_);
        }
      }
      
      stop_movement();      
      if (rclcpp::ok() ) { 
        goal_handle->succeed(result);
      }
      stop_movement();    // Just to make sure.
    
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The controller plugin failed to load for some reason: %s", ex.what());
      goal_handle->abort(result);      
      server_mutex.unlock();
      return;
    }
    catch(nav_drone_core::DroneException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The controller threw an exception: %s", ex.what());
      goal_handle->abort(result);
      server_mutex.unlock();
      return;
    }
        
    server_mutex.unlock();  
  }
  
  bool findControllerId(
    const std::string & c_name,
    std::string & current_controller) 
  {
    if (controllers_.find(c_name) == controllers_.end()) {
      if (controllers_.size() == 1 && c_name.empty()) {
        RCLCPP_WARN_ONCE(
          get_logger(), "No controller was specified in action call."
          " Server will use only plugin loaded %s. "
          "This warning will appear once.", controller_ids_concat_.c_str());
        current_controller = controllers_.begin()->first;
      } else {
        RCLCPP_ERROR(
          get_logger(), "FollowPath called with controller name %s, "
          "which does not exist. Available controllers are: %s.",
          c_name.c_str(), controller_ids_concat_.c_str());
        return false;
      }
    } else {
      RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
      current_controller = c_name;
    }

    return true;
  }
  
// UTILITIES THAT SHOULD BE IN THE GOAL CHECKER PLUGIN  
  bool is_goal_reached() {
    geometry_msgs::msg::PoseStamped pose;
    
    if( nav_drone_util::getCurrentPose(pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_) ) {   
      double position_error = nav_drone_util::euclidean_distance(pose, end_pose_);      
      double yaw_error = nav_drone_util::getDiff2Angles( nav_drone_util::getYaw(pose.pose.orientation), 
                                                         nav_drone_util::getYaw(end_pose_.pose.orientation), PI);  
      
      return holddown_timer_->test( (position_error < waypoint_radius_error_ ) &&
                                    ( yaw_error < yaw_threshold_)  );       
    } 
    return false;
    
  }
  
};  // class ControllerServer

}  // namespace nav_drone

RCLCPP_COMPONENTS_REGISTER_NODE(nav_drone::ControllerServer)