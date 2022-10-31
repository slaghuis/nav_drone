#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav_drone_core/controller.hpp"
#include "nav_drone_pid_controller/histogram.hpp"
#include "nav_drone_pid_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "nav_drone_util/odometry_utils.hpp"
//#include "nav_drone_util/geometry_utils.hpp"

namespace nav_drone_pid_controller
{
class PIDController : public nav_drone_core::Controller
{
  public:
  
    PIDController() = default;
    ~PIDController() override = default;
  
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
    rclcpp::Logger logger_ {rclcpp::get_logger("PIDController")};
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
    double desired_angular_vel_;
    
    double freq_;
  private:  
    double last_e_angle_, last_z_angle_;
    
    // PID Controllers  
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_y;
    std::shared_ptr<PID> pid_z;
    std::shared_ptr<PID> pid_yaw;
    
    geometry_msgs::msg::TwistStamped previous_setpoint_;
        
    double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);
    std::pair<int, int> get_ez_grid_pos(const octomap:: point3d & goal);
    std::pair<double, double> get_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                     const geometry_msgs::msg::PoseStamped & target_pose);
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist,
                                                      const geometry_msgs::msg::PoseStamped & current_pose);

};
  
}  // namespace  nav_drone_pid_controller