#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav_drone_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_drone_util/geometry_utils.hpp"

namespace nav_drone_regulated_pure_pursuit_controller
{  
    
class RegulatedPurePursuitController : public nav_drone_core::Controller
{
  public:
  
    RegulatedPurePursuitController() = default;
    ~RegulatedPurePursuitController() override = default;
  
    void configure(
      const rclcpp::Node::SharedPtr node, 
      std::string name, //std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav_drone_costmap_3d::CostmapPublisher> costmap ) override;
    
    void setPath(const nav_msgs::msg::Path & path) override;
    void updateMap(std::shared_ptr<nav_drone_costmap_3d::CostmapPublisher> costmap) override;
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & speed) override;   

  protected:
    /**
     * @brief Get lookahead distance
     * @param cmd the current speed to use to compute lookahead point
     * @return lookahead distance
     */
    //double getLookAheadDistance(const geometry_msgs::msg::Twist &);

    /**
     * @brief Whether robot should rotate to rough path heading
     * @param carrot_pose current lookahead point
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @return Whether should rotate to path heading
     */
    bool shouldRotateToPath(
      const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

    /**
     * @brief Whether robot should rotate to final goal orientation
     * @param carrot_pose current lookahead point
     * @return Whether should rotate to goal heading
     */
    bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

    /**
     * @brief Create a smooth and kinematically smoothed rotation command
     * @param linear_vel linear velocity
     * @param angular_vel angular velocity
     * @param angle_to_path Angle of robot output relatie to carrot marker
     * @param curr_speed the current robot speed
     */
    void rotateToHeading(
      double & linear_vel, double & angular_vel,
      const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);
  
    double approachVelocityScalingFactor(
      const geometry_msgs::msg::PoseStamped & pose
    ) const;

    void applyApproachVelocityScaling(
      const geometry_msgs::msg::PoseStamped & pose,
      double & linear_vel
    ) const;

    /**
     * @brief apply regulation constraints to the system
     * @param lookahead_dist optimal lookahead distance
     * @param curvature curvature of path
     * @param pose_cost cost at this pose
     * @param pose current pose of the robot
     * @param linear_vel robot command linear velocity input
     * @param sign Moving foreward +1 or reversing -1
     */
    void applyConstraints(
      const double & curvature, 
      const unsigned char & pose_cost, 
      const geometry_msgs::msg::PoseStamped & pose,
      double & linear_vel, double & sign);
      
    /**
     * @brief Get lookahead point
     * @param pose Current position in map frame
     * @param lookahead_dist Optimal lookahead distance
     * @return Lookahead point
     */
    geometry_msgs::msg::PoseStamped getLookAheadPoint(
      const geometry_msgs::msg::PoseStamped & pose,
      const double & lookahead_dist);


    /**
     * Get the greatest extent of the costmap in meters from the center.
     * @return max of distance from center in meters to edge of costmap
     */
    double getCostmapMaxExtent() const;

  
    rclcpp::Node::SharedPtr node_;
    std::string plugin_name_;
//    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav_drone_costmap_3d::CostmapPublisher> costmap_;
    rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;
  
    nav_msgs::msg::Path global_plan_;
      
    double desired_linear_vel_; 
    double base_desired_linear_vel_;
//    double lookahead_dist_;
    double rotate_to_heading_angular_vel_;
//    double max_lookahead_dist_;
//    double min_lookahead_dist_;
//    double lookahead_time_;
//    bool use_velocity_scaled_lookahead_dist_;
//    double transform_tolerance_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double control_duration_;
    bool use_collision_detection_;
    bool use_regulated_linear_velocity_scaling_;
    bool use_cost_regulated_linear_velocity_scaling_;
    double cost_scaling_dist_;
    double cost_scaling_gain_;
    double inflation_cost_scaling_factor_;
    double regulated_linear_scaling_min_radius_;
    double regulated_linear_scaling_min_speed_;
    bool use_rotate_to_heading_;
    double max_angular_accel_;
    double rotate_to_heading_min_angle_;
    double rotation_velocity_scaling_angle_;  
    bool use_regulated_angular_velocity_scaling_;
    double goal_dist_tol_;
    bool allow_reversing_;
    double max_robot_pose_search_dist_;
    bool use_interpolation_;
    double inscribed_radius_;
  private:  
    
//    double costmapSize();
            
};
  
}  // namespace  nav_drone_regulated_pure_pursuit_controller