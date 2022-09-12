#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav_drone_core/controller.hpp"
#include "nav_drone_mpc_controller/histogram.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "nav_drone_util/odometry_utils.hpp"
//#include "nav_drone_util/geometry_utils.hpp"

#include <dlib/control.h>

namespace nav_drone_mpc_controller
{
class MPCController : public nav_drone_core::Controller
{
  public:
  
    MPCController() = default;
    ~MPCController() override = default;
  
    void configure(
      const rclcpp::Node::SharedPtr node, 
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<octomap::OcTree> costmap ) override;
    
    void setPath(const nav_msgs::msg::Path & path) override;
    void updateMap(std::shared_ptr<octomap::OcTree> costmap) override;
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & speed) override;   
        
//        nav2_core::GoalChecker * goal_checker) = 0;
  protected:
    rclcpp::Node::SharedPtr node_;
    std::string plugin_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<octomap::OcTree> costmap_;
    rclcpp::Logger logger_ {rclcpp::get_logger("MPCController")};
    rclcpp::Clock::SharedPtr clock_;
  
    nav_msgs::msg::Path global_plan_;
      
    double lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;
    bool use_velocity_scaled_lookahead_dist_;
    double robot_radius_;
    double safety_radius_;
    
    double desired_linear_vel_;
  private:  
    double last_e_angle_, last_z_angle_;
    
    //variables for the DLib MPC Controller
    static const int STATES = 6;
    static const int CONTROLS = 3;
    static const int HORIZON = 30;

    dlib::matrix<double,STATES,STATES> A;
    dlib::matrix<double,STATES,CONTROLS> B;
    dlib::matrix<double,STATES,1> C;
    dlib::matrix<double,STATES,1> Q;
    dlib::matrix<double,CONTROLS,1> R, lower, upper;
    std::shared_ptr<dlib::mpc<STATES,CONTROLS,HORIZON>> controller;
    
    double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);
    std::pair<int, int> get_ez_grid_pos(const octomap:: point3d & goal);
    std::pair<double, double> get_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                     const geometry_msgs::msg::PoseStamped & target_pose);
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist,
                                                      const geometry_msgs::msg::PoseStamped & current_pose);

};
  
}  // namespace  nav_drone_mpc_controller