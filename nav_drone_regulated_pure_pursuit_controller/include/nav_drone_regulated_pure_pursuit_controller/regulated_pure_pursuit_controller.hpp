#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav_drone_core/controller.hpp"
//#include "nav_drone_regulated_pure_pursuit_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "nav_drone_util/odometry_utils.hpp"
#include "nav_drone_util/geometry_utils.hpp"

namespace nav_drone_regulated_pure_pursuit_controller
{
  
static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;  
  
  
static constexpr double NO_SPEED_LIMIT = 0.0;  
  
class RegulatedPurePursuitController : public nav_drone_core::Controller
{
  public:
  
    RegulatedPurePursuitController() = default;
    ~RegulatedPurePursuitController() override = default;
  
    void configure(
      const rclcpp::Node::SharedPtr node, 
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<octomap::OcTree> costmap ) override;
    
    void setPath(const nav_msgs::msg::Path & path) override;
    void updateMap(std::shared_ptr<octomap::OcTree> costmap) override;
    
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & speed) override;   
      
    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void setSpeedLimit(const double & speed_limit, const bool & percentage); // override;  
        
  protected:
    /**
     * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
     * Points ineligible to be selected as a lookahead point if they are any of the following:
     * - Outside the local_costmap (collision avoidance cannot be assured)
     * @param pose pose to transform
     * @return Path in new frame
     */
    nav_msgs::msg::Path transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped & pose);

    /**
     * @brief Transform a pose to another frame.
     * @param frame Frame ID to transform to
     * @param in_pose Pose input to transform
     * @param out_pose transformed output
     * @return bool if successful
     */
    bool transformPose(
      const std::string frame,
      const geometry_msgs::msg::PoseStamped & in_pose,
      geometry_msgs::msg::PoseStamped & out_pose) const;

    /**
     * @brief Get lookahead distance
     * @param cmd the current speed to use to compute lookahead point
     * @return lookahead distance
     */
    double getLookAheadDistance(const geometry_msgs::msg::Twist &);

    /**
     * @brief Creates a PointStamped message for visualization
     * @param carrot_pose Input carrot point as a PoseStamped
     * @return CarrotMsg a carrot point marker, PointStamped
     */
    std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
      const geometry_msgs::msg::PoseStamped & carrot_pose);

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
  
    /**
     * @brief Cost at a point
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @return Cost of pose in costmap
     */
    double costAtPose(const double & x, const double & y, const double & z);

    //double approachVelocityScalingFactor(
    //  const nav_msgs::msg::Path & path
    //) const;

    //void applyApproachVelocityScaling(
    //  const nav_msgs::msg::Path & path,
    //  double & linear_vel
    //) const;

    /**
     * @brief apply regulation constraints to the system
     * @param linear_vel robot command linear velocity input
     * @param lookahead_dist optimal lookahead distance
     * @param curvature curvature of path
     * @param speed Speed of robot
     * @param pose_cost cost at this pose
     */
    void applyConstraints(
      const double & curvature, const geometry_msgs::msg::Twist & speed,
      const double & pose_cost, const nav_msgs::msg::Path & path,
      double & linear_vel, double & sign);
      
    /**
     * @brief Find the intersection a sphere and a line segment.
     * This assumes the circle is centered at the origin.
     * If no intersection is found, an exceprion will be thrown.
     * @param p1 first endpoint of line segment
     * @param p2 second endpoint of line segment
     * @param r radius of sphere
     * @return point of intersection
     */
     static geometry_msgs::msg::Point sphereSegmentIntersection(
      const geometry_msgs::msg::Point & p1,
      const geometry_msgs::msg::Point & p2,
      double r);  

    /**
     * @brief Get lookahead point
     * @param lookahead_dist Optimal lookahead distance
     * @param path Current global path
     * @return Lookahead point
     */
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

    /**
     * @brief checks for the cusp position
     * @param pose Pose input to determine the cusp position
     * @return robot distance from the cusp
     */
    double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

    /**
     * Get the greatest extent of the costmap in meters from the center.
     * @return max of distance from center in meters to edge of costmap
     */
    double getCostmapMaxExtent() const;

  
    rclcpp::Node::SharedPtr node_;
    std::string plugin_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<octomap::OcTree> costmap_;
    rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;
  
    nav_msgs::msg::Path global_plan_;
      
    double desired_linear_vel_; 
    double base_desired_linear_vel_;
    double lookahead_dist_;
    double rotate_to_heading_angular_vel_;
    double max_lookahead_dist_;
    double min_lookahead_dist_;
    double lookahead_time_;
    bool use_velocity_scaled_lookahead_dist_;
    double cost_map_size_;
    tf2::Duration transform_tolerance_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double control_duration_;
    double max_allowed_time_to_collision_up_to_carrot_;
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
    
    double costmapSize();
            
};
  
}  // namespace  nav_drone_regulated_pure_pursuit_controller