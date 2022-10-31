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
#pragma once

#include <cmath>  // floor
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

const double PI  =3.141592653589793238463;

namespace nav_drone_util 
{

/*
// In C the modulo operation returns a value with the same sign as the dividend.
// Hence a custom modulo function
inline double modulo(const double a, const double n) {
  return a - floor(a/n) * n;
}

// Returns the difference between two angles x and y as a number 
// between -180 and 180.  c can be PI for radians, or 180 for degrees. 
inline double getDiff2Angles(const double x, const double y, const double c)
{
  double a = x-y;
  return modulo( a+c, 2*c) - c;
}
*/  

inline double getDiff2Angles(const double x, const double y, const double c)
{
  double d = fabs(fmod(fabs(x-y), 2*c));
  double r = d > c ? c*2-d : d;

  double sign = ((x-y >= 0.0) && (x-y <= c)) || ((x-y <= c) && (x-y> -2*c)) ? 1.0 : -1.0;
  return sign * r;
}

	
	
// Calculate the angle between a line defined by two points and the coordinate axes.
// result is in RADIANS
double angle(const double x1, const double y1, const double x2, const double y2);

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = true)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}  
  
/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = true)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}
  
inline double rad_to_deg(const double rad) { return (rad * 180.0) / PI; }
inline double deg_to_rad(const double deg) { return (deg * PI) / 180.0; }
  
  
}    // namespace nav_drone_util
