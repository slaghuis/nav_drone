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

// In ROS Foxy there is a angles/angles.h with most of these functions,
// but for some reason the file was not present on my Raspberry PI install.

#pragma once

#include <algorithm>
#include <cmath>  // floor

#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

const double PI  =3.141592653589793238463;

namespace nav_drone_util 
{
  
    /*!
   * \brief Convert degrees to radians
   */

  static inline double from_degrees(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  /*!
   * \brief Convert radians to degrees
   */
  static inline double to_degrees(double radians)
  {
    return radians * 180.0 / M_PI;
  }


  /*!
   * \brief normalize_angle_positive
   *
   *        Normalizes the angle to be 0 to 2*M_PI
   *        It takes and returns radians.
   */
  static inline double normalize_angle_positive(double angle)
  {
    const double result = fmod(angle, 2.0*M_PI);
    if(result < 0) return result + 2.0*M_PI;
    return result;
  }

  /*!
   * \brief normalize
   *
   * Normalizes the angle to be -M_PI circle to +M_PI circle
   * It takes and returns radians.
   *
   */
  static inline double normalize_angle(double angle)
  {
    const double result = fmod(angle + M_PI, 2.0*M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
  }


  /*!
   * \function
   * \brief shortest_angular_distance
   *
   * Given 2 angles, this returns the shortest angular
   * difference.  The inputs and ouputs are of course radians.
   *
   * The result
   * would always be -pi <= result <= pi.  Adding the result
   * to "from" will always get you an equivelent angle to "to".
   */

  static inline double shortest_angular_distance(double from, double to)
  {
    return normalize_angle(to-from);
  }
 

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
  
inline double rad_to_deg(const double rad) { return (rad * 180.0) / PI; }
inline double deg_to_rad(const double deg) { return (deg * PI) / 180.0; }
  
  
}    // namespace nav_drone_util
