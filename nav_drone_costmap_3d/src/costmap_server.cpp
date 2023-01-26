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

/* *************************************************************
 * Simplified Costmap publisher
 * Reads robot velocity to determine lookahead distance
 * Reads Octomap messages and updates a costmap structure based on 
 * a 2D Polar histogram aroun the robot, scaled according to the
 * robot velocity.
 * Publishes this costmap on a regular interval
 * *************************************************************/
#include "nav_drone_costmap_3d/costmap_server.hpp"
#include "nav_drone_costmap_3d/costmap_3d.hpp"
#include "nav_drone_costmap_3d/cost_values.hpp"

#include "nav_drone_util/robot_utils.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/costmap_utils.hpp"
#include "nav_drone_core/costmap_exceptions.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav_drone_costmap_3d {

CostmapPublisher::CostmapPublisher(const std::string & name)
  : CostmapPublisher(name, "/") {}


CostmapPublisher::CostmapPublisher()
  : Node("costmap_publisher"),
  name_("costmap")
{
    declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    init();
}
  
CostmapPublisher::CostmapPublisher(
    const std::string & name,
    const std::string & parent_namespace)
  : Node(name),
  name_(name),
  parent_namespace_(parent_namespace)
{
  declare_parameter(
    "map_topic", rclcpp::ParameterValue(
      (parent_namespace_ == "/" ? "/" : parent_namespace_ + "/") + std::string("map")));
  init();

}  
  
void CostmapPublisher::init()
{
  costmap_ = std::make_shared<Costmap3D>(ALPHA_RES);
  last_costmap_update_ = this->get_clock()->now();
       
  // Declare and get parameters    
  nav_drone_util::declare_parameter_if_not_declared(
    this, "map_frame", rclcpp::ParameterValue("map"));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue("base_link"));   
  nav_drone_util::declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.3));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "lookahead_dist", rclcpp::ParameterValue(2.0));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "min_lookahead_dist", rclcpp::ParameterValue(0.8));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "max_lookahead_dist", rclcpp::ParameterValue(4.0));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "lookahead_time", rclcpp::ParameterValue(1.5));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "robot_radius", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    this, "safety_radius", rclcpp::ParameterValue(0.5));
  
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  this->get_parameter("lookahead_dist", lookahead_dist_);
  this->get_parameter("min_lookahead_dist", min_lookahead_dist_);
  this->get_parameter("max_lookahead_dist", max_lookahead_dist_);
  this->get_parameter("lookahead_time", lookahead_time_);
  this->get_parameter("use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  this->get_parameter("robot_radius", robot_radius_);
  this->get_parameter("safety_radius", safety_radius_);
   
  // Create a transform listener
  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(this->get_clock());      
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
  // Make sure that the transform between the robot base frame
  // and the global frame is available

  std::string tf_error;

  RCLCPP_INFO(get_logger(), "Checking transform");
  rclcpp::Rate r(2);
  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(
      map_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), map_frame_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }

  
  // ROS2 Subscriptions
  map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "nav_drone/map", 10, std::bind(&CostmapPublisher::map_callback, this, _1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "drone/odom", 10, std::bind(&CostmapPublisher::odom_callback, this, _1));  
      
  // ROS2 Publihers
  costmap_publisher_ = this->create_publisher<nav_drone_msgs::msg::Costmap>("nav_drone/costmap", 10);  
  timer_ = this->create_wall_timer(
    1000ms, std::bind(&CostmapPublisher::publish_costmap, this));   
  
  current_ = false;
}
// MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
void CostmapPublisher::map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
{
  // Convert ROS message to a OctoMap
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
    last_octomap_update_ = this->get_clock()->now();

  } else {
    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
  } 
}
rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
// ODOM SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
void CostmapPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
  last_velocity_ = msg->twist.twist;
}
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  
// MAP UPDATE COSTMAP ////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<int, int> CostmapPublisher::get_ez_grid_pos(const octomap::point3d & goal)
{
  // Now we want to work in the base_link frame to incorporate the yaw of the drone.  
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = rclcpp::Time();
  goal_pose.pose.position.x = goal.x();
  goal_pose.pose.position.y = goal.y();
  goal_pose.pose.position.z = goal.z();
    
  geometry_msgs::msg::PoseStamped voxel;
  //nav_drone_util::transformPoseInTargetFrame(goal_pose, voxel, *tf_buffer_, robot_base_frame_);
  try {
    nav_drone_util::transformPoseInTargetFrame(goal_pose, voxel, *tf_buffer_, robot_base_frame_);
  } catch (const tf2::LookupException &ex) {
     RCLCPP_INFO(this->get_logger(), "Oh dear : %s", ex.what());
  }  
    
  geometry_msgs::msg::PoseStamped source_pose;
  goal_pose.header.frame_id = robot_base_frame_;
  goal_pose.pose.position.x = 0.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.position.z = 0.0;
        
  auto ez = nav_drone_util::calculate_ez(source_pose, voxel);
    
  // Moving the values into positive whole numbers, scaled to fit into our matrix
  double e = floor( (90.0 + nav_drone_util::rad_to_deg( ez.first) ) / ALPHA_RES);
  double z = floor( (180.0 + nav_drone_util::rad_to_deg( ez.second ) ) / ALPHA_RES);
    
  return std::pair<int, int>(e,z);
} 
  
unsigned char CostmapPublisher::cost_at_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped voxel;
  try {
    nav_drone_util::transformPoseInTargetFrame(pose, voxel, *tf_buffer_, robot_base_frame_);
  } catch (const tf2::LookupException &ex) {
     RCLCPP_WARN(this->get_logger(), ex.what());
    return NO_INFORMATION;
  }  
  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header.frame_id = robot_base_frame_;
  source_pose.pose.position.x = 0.0;
  source_pose.pose.position.y = 0.0;
  source_pose.pose.position.z = 0.0;
        
  auto ez = nav_drone_util::calculate_ez(source_pose, voxel);
    
  // Moving the values into positive whole numbers, scaled to fit into our matrix
  double e = floor( (90.0 + nav_drone_util::rad_to_deg( ez.first) ) / ALPHA_RES);
  double z = floor( (180.0 + nav_drone_util::rad_to_deg( ez.second ) ) / ALPHA_RES);
  
  auto weight = costmap_->get_weight(e, z);
  if( weight < 0.01) {
    return FREE_SPACE;
  } else {
    return LETHAL_OBSTACLE;
  }
  
}

bool CostmapPublisher::update_costmap(const geometry_msgs::msg::PoseStamped & current_pose,
                      const double bounding_box_radius) {
    
  if(octomap_ == nullptr){ 
    throw nav_drone_core::CostmapUpdateError("No octomap has been received. Is the octomap publishing data?");
  }
    
  if (bounding_box_radius < (robot_radius_ + safety_radius_ + octomap_->getResolution()) ) { 
    throw nav_drone_core::CostmapUpdateError("Bounding box is smaller than the robot and its safety margin.");
  }  
    
  octomap:: point3d start_point(current_pose.pose.position.x,
                                current_pose.pose.position.y,
                                current_pose.pose.position.z);

  octomap::point3d min(current_pose.pose.position.x - bounding_box_radius,  
                       current_pose.pose.position.y - bounding_box_radius,
                       current_pose.pose.position.z - bounding_box_radius);  
  octomap::point3d max(current_pose.pose.position.x + bounding_box_radius,
                       current_pose.pose.position.y + bounding_box_radius,
                       current_pose.pose.position.z + bounding_box_radius);
                                
  double const_b = 5.0;    // Just a random number
  double const_a = 1.0 + const_b * pow((bounding_box_radius - 1.0) / 2, 2);
        
  costmap_mutex.lock();
  costmap_->set_zero();
    
  for(octomap::OcTree::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(min,max),
    end=octomap_->end_leafs_bbx(); it!= end; ++it) {
             
    octomap::point3d end_point(it.getCoordinate());
    double distance = start_point.distance(end_point);
    double l = distance - (robot_radius_ + safety_radius_ + octomap_->getResolution());
    if ( (distance <= bounding_box_radius) && (l > 0.001) )  {   // Work in the drone radius, to minimuse calculation.
      // The point is within a sphere around the drone, thus an active cell
      std::pair<int, int> coords = get_ez_grid_pos(end_point);    
      int lamda = floor(nav_drone_util::rad_to_deg( asin((robot_radius_ + safety_radius_ ) / distance) / ALPHA_RES));
                
      double weight = pow( it->getOccupancy(), 2) * (const_a - (const_b * l));
             
      if( weight > 0.001 ) {    // Small optimization.  Don't iterate for zero weight or if the target is on top of itself.
        // Incorporate the size of the robot in the calculation.  All these points in the costmap are influenced
        for(int e = std::max(0, coords.first-lamda); e <= coords.first+lamda; e++) {
          for(int z = std::max(0, coords.second-lamda); z <= coords.second+lamda; z++) {  
            // Trim all values of e & z that are out of bounds, because we add or subtract lamda
            if ((e < costmap_->e_dim()) && ( z < costmap_->z_dim() )) {  
              costmap_->add_weight(e, z, weight);
            }  
          }             
        }
      }  
    };  // else discard the point.
  }  
  costmap_mutex.unlock();
  return true; 
} 
        
// COSTMAP PUBLISHER ////////////////////////////////////////////////////////////////////////////////////////////////
double CostmapPublisher::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = std::max(fabs(speed.linear.x), fabs(speed.linear.z)) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
} 
  
void CostmapPublisher::publish_costmap() {
  
  // Only update the costmap after the first octomap has been received
  if ( (octomap_ != nullptr)  ) { //&& (last_octomap_update_ > last_costmap_update_) ) {
    geometry_msgs::msg::PoseStamped current_pose;    
    if(!nav_drone_util::getCurrentPose(current_pose, *tf_buffer_, map_frame_, robot_base_frame_, transform_tolerance_)) {
      throw nav_drone_core::CostmapTFError("Failed to obtain robot pose.");
    }
    last_update_pose_ = current_pose.pose;    
    double bounding_box_radius = getLookAheadDistance(last_velocity_);
    
    try {
      update_costmap(current_pose, bounding_box_radius);  
      last_costmap_update_ = this->get_clock()->now();
      last_costmap_radius_ = bounding_box_radius;

    } catch (nav_drone_core::CostmapUpdateError & e) {
      RCLCPP_WARN(this->get_logger(), e.what());
    }
  }
    
  costmap_mutex.lock();
  // Read the costmap into a costmap message.
  auto message = nav_drone_msgs::msg::Costmap();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = robot_base_frame_;
  message.metadata.map_load_time = last_octomap_update_;
  message.metadata.update_time = last_costmap_update_;
  message.metadata.layer = "octomap";
  message.metadata.radius = last_costmap_radius_;
  message.metadata.size_x = costmap_->z_dim();
  message.metadata.size_y = costmap_->e_dim();
  message.metadata.origin = last_update_pose_;
    
  message.data.resize(costmap_->z_dim() * costmap_->z_dim());
  for(int e = 0; e < costmap_->e_dim(); e++) {
    for(int z = 0; z < costmap_->z_dim(); z++) {
      auto weight = costmap_->get_weight(e, z);
      if( weight < 0.01) {
        message.data[e * z] = FREE_SPACE;
      } else {
        message.data[e * z] = LETHAL_OBSTACLE;
      }
    }
  }
  
  costmap_mutex.unlock();

  current_ = true;
    
  costmap_publisher_->publish(message);    
}
  
bool CostmapPublisher::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) { 
  return nav_drone_util::getCurrentPose(
    global_pose, *tf_buffer_,
    map_frame_, robot_base_frame_, transform_tolerance_);
}
  
}  // namespace nav_drone_costmap_3d
