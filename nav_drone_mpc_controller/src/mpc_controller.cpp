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

/* *******************************************************
 * Plugin to the navigation_lite controller server
 * Planning algorith: Linear Model Predictive Control
 * ******************************************************* */
#include <math.h>     // fabs
#include <algorithm>  // std::clamp (C++ 17), find_if
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav_drone_mpc_controller/mpc_controller.hpp"
#include "nav_drone_mpc_controller/histogram.hpp"
#include "nav_drone_core/exceptions.hpp"
#include "nav_drone_util/node_utils.hpp"
#include "nav_drone_util/angle_utils.hpp"
#include "nav_drone_util/robot_utils.hpp"

#include "pluginlib/class_list_macros.hpp"

// Costants for path weights as per Ulrich et al.
const double MU1 = 5;   // Target angle vs Candidate direction
const double MU2 = 2;   // Rotation Theta of the robot and the candidate direction
const double MU3 = 2;   // Previous delected direction and candidate direction

namespace nav_drone_mpc_controller
{
  
//   MPCController() = default;
//  ~MPCController() override = default;
  
void MPCController::configure(const rclcpp::Node::SharedPtr node, 
                              std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<octomap::OcTree> costmap )
{
  node_ = node;
    
  tf_ = tf;
  plugin_name_ = name;
  costmap_ = costmap;    
  
  logger_ = node_->get_logger();
  clock_ = node->get_clock();

  
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".robot_radius", rclcpp::ParameterValue(0.5));
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".safety_radius", rclcpp::ParameterValue(0.5));
  
  nav_drone_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));

  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
  node_->get_parameter(plugin_name_ + ".safety_radius", safety_radius_);
 
  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
 
  
  last_e_angle_ = 0.0;
  last_z_angle_ = 0.0;
  // Setup the dlib MPC Controller
  
  // The first thing we do is setup our vehicle dynamics model (A*x + B*u + C).
  // Our state space (the x) will have 4 dimensions, the 2D vehicle position
  // and also the 2D velocity.  The control space (u) will be just 2 variables
  // which encode the amount of force we apply to the vehicle along each axis.
  // Therefore, the A matrix defines a simple constant velocity model.
  //matrix<double,STATES,STATES> A;
  A = 1, 0, 0, 1, 0, 0,  // next_pos = pos + velocity
      0, 1, 0, 0, 1, 0,  // next_pos = pos + velocity
      0, 0, 1, 0, 0, 1,  // next_pos = pos + velocity
      0, 0, 0, 1, 0, 0,  // next_velocity = velocity
      0, 0, 0, 0, 1, 0,  // next_velocity = velocity
      0, 0, 0, 0, 0, 1;  // next_velocity = velocity

  // Here we say that the control variables effect only the velocity. That is,
  // the control applies an acceleration to the vehicle.
  //matrix<double,STATES,CONTROLS> B;
  B = 0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      1, 0, 0,
      0, 1, 0,
      0, 0, 1;

  // Let's also say there is a small constant acceleration in one direction.
  // This is the force of gravity in our model. 
  //matrix<double,STATES,1> C;
  C = 0,
      0,
      0,
      0.1,
	    0,
	    0;

  //const int HORIZON = 30;
  // Now we need to setup some MPC specific parameters.  To understand them,
  // let's first talk about how MPC works.  When the MPC tool finds the "best"
  // control to apply it does it by simulating the process for HORIZON time
  // steps and selecting the control that leads to the best performance over
  // the next HORIZON steps.
  //  
  // To be precise, each time you ask it for a control, it solves the
  // following quadratic program:
  //   
  //     min     sum_i trans(x_i-target_i)*Q*(x_i-target_i) + trans(u_i)*R*u_i 
  //    x_i,u_i
  //
  //     such that: x_0     == current_state 
  //                x_{i+1} == A*x_i + B*u_i + C
  //                lower <= u_i <= upper
  //                0 <= i < HORIZON
  //
  // and reports u_0 as the control you should take given that you are currently
  // in current_state.  Q and R are user supplied matrices that define how we
  // penalize variations away from the target state as well as how much we want
  // to avoid generating large control signals.  We also allow you to specify
  // upper and lower bound constraints on the controls.  The next few lines
  // define these parameters for our simple example.

  //matrix<double,STATES,1> Q;
  // Setup Q so that the MPC only cares about matching the target position and
  // ignores the velocity.  
  Q = 1, 1, 1, 0, 0, 0;

  //matrix<double,CONTROLS,1> R, lower, upper;
  R = 1, 1, 1;
  lower = -0.5, -0.5, -0.5;
  upper =  0.5,  0.5,  0.5;

  //Fnally, create the MPC controller
  controller = std::make_shared<dlib::mpc<STATES,CONTROLS,HORIZON>>(A,B,C,Q,R,lower,upper);
}
  
    
void MPCController::setPath(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}
  
void MPCController::updateMap(std::shared_ptr<octomap::OcTree> costmap)
{
  costmap_ = costmap;
}
  
    
geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & speed) 
{
  double lookahead_dist = getLookAheadDistance(speed);
  auto goal_pose = getLookAheadPoint(lookahead_dist, pose);
  // Transform the goal_pose to the base_link frame.  Now flight is realtive to the 
  // current position.
  //geometry_msgs::msg::PoseStamped carrot_pose;
  //nav_drone_util::transformPoseInTargetFrame(goal_pose, carrot_pose, *tf_, "base_link" );

  //  Calculate velocity commands using the MPC controller
  dlib::matrix<double,STATES,1> target;
  target = goal_pose.pose.position.x, 
           goal_pose.pose.position.y, 
           goal_pose.pose.position.z,  
           0, 0, 0; // Position, at zero velocity
  controller->set_target(target);

  // Find the best control action given our current state.
  dlib::matrix<double,STATES,1> current_state;
  current_state = pose.pose.position.x,
                  pose.pose.position.y,
                  pose.pose.position.z,
                  speed.linear.x,
                  speed.linear.y,
                  speed.linear.z;
  dlib::matrix<double,CONTROLS,1> action = (*controller)(current_state);
  geometry_msgs::msg::TwistStamped setpoint = geometry_msgs::msg::TwistStamped();
  setpoint.header.frame_id = "map";         //carrot_pose.header.frame_id;   // "base_link";
  setpoint.header.stamp = clock_->now();
  
  setpoint.twist.linear.x = action(0);
  setpoint.twist.linear.y = action(1);
  setpoint.twist.linear.z = action(2);
  
  return setpoint;
  
}
  
  
double MPCController::getLookAheadDistance(
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
  
std::pair<int, int> MPCController::get_ez_grid_pos(const octomap:: point3d & goal)
{
  // Now we want to work in the base_link frame to incorporate the yaw of the drone.  This is the trick in the algorithm I think.
  // The VCP will be at 0,0,0 and the target will be positioned apropriately.  Straight flight will take one to the middle of the
  // histogram.  This way section 4.3 of the algoritm becomes easy to impliment.
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = rclcpp::Time();
  goal_pose.pose.position.x = goal.x();
  goal_pose.pose.position.y = goal.y();
  goal_pose.pose.position.z = goal.z();
  goal_pose.pose.orientation.x = 0.0;    // Just specify a neutral yaw.  Not really relevant.
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;
    
  geometry_msgs::msg::PoseStamped voxel;
//  nav_drone_util::transformPoseInTargetFrame("base_link", goal_pose, voxel);
  nav_drone_util::transformPoseInTargetFrame(goal_pose, voxel, *tf_, "base_link" );
    
  geometry_msgs::msg::PoseStamped source_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.position.z = 0.0;
        
  auto ez = get_ez(source_pose, voxel);
    
  // Moving the values into positive whole numbers, scaled to fit into our matrix
  double e = floor( (90.0 + nav_drone_util::rad_to_deg( ez.first) ) / ALPHA_RES);
  double z = floor( (180.0 + nav_drone_util::rad_to_deg( ez.second ) ) / ALPHA_RES);
    
  return std::pair<int, int>(e,z);
}  

  
std::pair<double, double> MPCController::get_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                                const geometry_msgs::msg::PoseStamped & target_pose)
{    
  // Using direction cosines as discussed
  // https://gis.stackexchange.com/questions/108547/how-to-calculate-distance-azimuth-and-dip-from-two-xyz-coordinates
  // by https://gis.stackexchange.com/users/2581/gene
  double distance = std::hypot(target_pose.pose.position.x - current_pose.pose.position.x, 
                               target_pose.pose.position.y - current_pose.pose.position.y, 
                               target_pose.pose.position.z - current_pose.pose.position.z);
  double cosalpha = (target_pose.pose.position.x - current_pose.pose.position.x) / distance;
  double cosbeta = (target_pose.pose.position.y - current_pose.pose.position.y) / distance;
  double cosgamma = (target_pose.pose.position.z - current_pose.pose.position.z) / distance;
  double plunge = asin(cosgamma);   // # the resulting dip_plunge is positive downward if z2 > z1
    
  // prevent division by zero
  if ( (cosalpha == 0.0) && (cosbeta == 0.0) ) {
    cosalpha = 0.000001;
  }
  double azimuth =  atan2(cosalpha, cosbeta); 
    
  return std::pair<double, double>(plunge, azimuth);
}

// This applies the 3DVFH+ algorithm for local planning
// The input and result is in the map frame  
geometry_msgs::msg::PoseStamped MPCController::getLookAheadPoint(
  const double & lookahead_dist,
  const geometry_msgs::msg::PoseStamped & current_pose)
{    
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(), [&](const auto & ps) {
      return (nav_drone_util::euclidean_distance(ps, current_pose) >= lookahead_dist);
    });
    
  double bounding_box_radius = lookahead_dist;
    
  // If the pose is not far enough, take the last pose
  if (goal_pose_it == global_plan_.poses.end()) {
    goal_pose_it = std::prev(global_plan_.poses.end());
    bounding_box_radius = nav_drone_util::euclidean_distance(current_pose, *goal_pose_it, true);
  }    
                   
  if (bounding_box_radius < 0.5 ) {    // Too close to calculate anything usable
    return *goal_pose_it;
  }  

  // 4.1 Iterate through the Octomap and 4.2 populate the 2D Primary Historam        
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
     
  Histogram histogram(ALPHA_RES);
  histogram.set_zero();
//  RCLCPP_INFO(logger_, "Two - Four - One");
  
  if(costmap_ == nullptr){ 
    RCLCPP_ERROR(logger_, "Costmap is null.  Did you provide an updated map? ");
    return *goal_pose_it;
  } 
    
  for(octomap::OcTree::leaf_bbx_iterator it = costmap_->begin_leafs_bbx(min,max),
    end=costmap_->end_leafs_bbx(); it!= end; ++it) {
    RCLCPP_INFO(logger_, "Two - Four - Two");   
             
    octomap::point3d end_point(it.getCoordinate());
    double distance = start_point.distance(end_point);
    double l = distance - (robot_radius_ + safety_radius_ + costmap_->getResolution());
    if ( (distance <= bounding_box_radius) && (l > 0.001) )  {   // Work in the drone radius, to minimuse calculation.
      RCLCPP_INFO(logger_, "Two - Four - Three");  
      // The point is within a sphere around the drone, thus an active cell
      std::pair<int, int> coords = get_ez_grid_pos(end_point);   // NOTE this point is in the base_link frame.  Remember when translating to the lookahead point    
      int lamda = floor(nav_drone_util::rad_to_deg( asin((robot_radius_ + safety_radius_ + costmap_->getResolution()) / distance) / ALPHA_RES));
                
      double weight = pow( it->getOccupancy(), 2) * (const_a - (const_b * l));
             
      if( weight > 0.001 ) {    // Small optimization.  Don't iterate for zero weight or if the target is on top of itself.
        // Incorporate the size of the robot in the calculation.  All these points in the histogram are influenced
        for(int e = std::max(0, coords.first-lamda); e <= coords.first+lamda; e++) {
          for(int z = std::max(0, coords.second-lamda); z <= coords.second+lamda; z++) {
            RCLCPP_INFO(logger_, "Two - Four - Four");  
            // Trim all values of e & z that are out of bounds, because we add or subtract lamda
            if ((e < histogram.e_dim()) && ( z < histogram.z_dim() )) {  
              histogram.add_weight(e, z, weight);
              RCLCPP_INFO(logger_, "Two - Four - Five");
            }  
          }             
        }
      }  
    };  // else discard the point.
  }  
    
  // 4.4 Fourth Stage: 2D Binary Polar Histogram
  /* IMPROVEMENT REQUIRED:  
       Because the 3DVFH+ algorithm uses a 2D polar histogram, the thresholds
       high and low need to change when using a different elevation angle Be. The cells
       of the 2D polar histogram do not have the same size (as shown in Figure 2) so
       to compensate for this, different thresholds are required for different elevation
       angles, see (18). The size of these thresholds depends on the robot, the robot's
       speed, the window size of stage  ve, the octomap resolution and the bounding
       sphere size.
  */

    RCLCPP_INFO(logger_, "Two - Five");

  histogram.go_binary(0.03, 0.3);      // See note above.  Using constants for now
    
  // 4.5 Fith Stage: Path detection and selection
  int best_e, best_z;
  double best_score = std::numeric_limits<double>::max();   // Default to the worst possible score
    
  // Calculate some numbers to optimise the algorithm
  auto target_ez = get_ez(current_pose, *goal_pose_it); 
  double target_elevation = nav_drone_util::rad_to_deg( target_ez.first );
  double target_azimuth = nav_drone_util::rad_to_deg( target_ez.second );
    
  for(int e = 0; e < histogram.e_dim(); e++) {
    for(int z = 0; z < histogram.z_dim(); z++) {
        
      if( histogram.path_available(e, z) ) {

        // Keep in mind:
        // The histogram is in the base_link frame with the robot at 0,0,0.  
        // The histogram is in DEGREES
        // e and z are indexes relative to ALPHA_RES.  Don't forget to multiply out
        // histogram indexes have been manipulated to be positive (e += 90, z+=180 degrees)

        double z_angle = (z * ALPHA_RES) - 180; // z angle is the direction of azimuth (horisontal) 
          
        // "The first path weight is the difference between the target angle and this candidate direction"      
        double delta_vk = fabs(nav_drone_util::getDiff2Angles(target_azimuth, z_angle, 180));
          
        // "The second path weight is the difference between the rotation of the robot" (yaw in map frame) " and the candidate direction" 
        // But the matrix is in the base_link frame.  current Yaw is 0.  Thus the penalty is simply the azimuth to the target.  
        double delta_vtheta = fabs(z_angle);

        // "The last path weight is the difference between the previous selected direction and the candidate direction"
        double delta_vk1 = fabs(nav_drone_util::getDiff2Angles(last_z_angle_, z_angle, 180));
          
        double z_score = MU1*delta_vk + MU2*delta_vtheta + MU3*delta_vk1;           
          
        // Repeat these now for the elevation
        double e_angle = (e * ALPHA_RES) - 90;  // e_angle is the elevation in degrees  
        delta_vk = fabs(nav_drone_util::getDiff2Angles(e_angle, target_elevation, 180));          
        delta_vtheta = fabs(e_angle); 
        delta_vk1 = fabs(nav_drone_util::getDiff2Angles(last_e_angle_, e_angle, 180));               
                    
        double e_score = MU1*delta_vk + MU2*delta_vtheta + MU3*delta_vk1;
                     
        if ((z_score + e_score) < best_score) {
          best_z = z;
          best_e = e;
          best_score = z_score + e_score;
        }  
      }
    }             
  }
            
  double plunge = best_e * ALPHA_RES - 90.0;
  double azimuth = best_z * ALPHA_RES - 180.0;

  geometry_msgs::msg::PoseStamped point;
  point.header.frame_id = "map";              
  point.pose.position.x = bounding_box_radius * cos(nav_drone_util::deg_to_rad(azimuth)) + current_pose.pose.position.x;
  point.pose.position.y = bounding_box_radius * sin(nav_drone_util::deg_to_rad(azimuth)) + current_pose.pose.position.y;
  point.pose.position.z = bounding_box_radius * sin(nav_drone_util::deg_to_rad(plunge)) + current_pose.pose.position.z;
    
  RCLCPP_DEBUG(logger_, "Fly to point [%.2f, %.2f, %.2f] to avoid obstacle.", 
               point.pose.position.x,
               point.pose.position.y,
               point.pose.position.z);
    
  return point;    
}       

  
}  // namespace  nav_drone_mpc_controller

// Register this controller as a nav_drone_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav_drone_mpc_controller::MPCController,
  nav_drone_core::Controller)