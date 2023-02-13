// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/path.hpp"

namespace nav_drone_util
{

class GeometryException : public std::runtime_error
{
public:
  explicit GeometryException(const std::string & description)
  : std::runtime_error(description) {}
};

class GeometryBoundsError : public GeometryException
{
public:
  explicit GeometryBoundsError(const std::string & description)
  : GeometryException(description) {}
};    
  
  
/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2,
  const bool is_3d = false)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

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
  const bool is_3d = false)
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
  const bool is_3d = false)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
/*inline double euclidean_distance(
  const geometry_msgs::msg::Pose2D & pos1,
  const geometry_msgs::msg::Pose2D & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}
*/
/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1), true);
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}
  
inline geometry_msgs::msg::Point sphereSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & cen,
  double r)
{
  // Formula for intersection of a line with a sphere centered at cen,
  // modified to allways return the point that is on the segment between the two points.
  // If one of the points is not inside the sphere, an exception will be thrown
  // https://stackoverflow.com/questions/6533856/ray-sphere-intersection
  
  double xA = p1.x;
  double yA = p1.y;
  double zA = p1.z;

  double xB = p2.x;
  double yB = p2.y;
  double zB = p2.z;
  
  double xC = cen.x;
  double yC = cen.y;
  double zC = cen.z;
  
  //a = (xB-xA)²+(yB-yA)²+(zB-zA)²
  double a = pow(xB-xA,2) + pow(yB-yA,2) + pow(zB-zA,2); 
  //b = 2*((xB-xA)(xA-xC)+(yB-yA)(yA-yC)+(zB-zA)(zA-zC))
  double b = 2 * ((xB-xA)*(xA-xC) + (yB-yA)*(yA-yC) + (zB-zA)*(zA-zC));  
  //c = (xA-xC)²+(yA-yC)²+(zA-zC)²-r²
  double c = pow(xA-xC,2) + pow(yA-yC,2) + pow(zA-zC,2) - pow(r,2); 
  
  double delta = pow(b,2)-4*a*c;
  if (delta == 0.0) {
    double d = -b / 2*a;
    geometry_msgs::msg::Point p;
    p.x = xA + d*(xB-xA);
    p.y = yA + d*(yB-yA);
    p.z = zA + d*(zB-zA);
    return p;
  } 
  
  if (delta > 0.0) {
    double d1 = (-b-sqrt(delta))/(2*a);
    double d2 = (-b+sqrt(delta))/(2*a);
  
    geometry_msgs::msg::Point r1;
    r1.x = xA + d1*(xB-xA);
    r1.y = yA + d1*(yB-yA);
    r1.z = zA + d1*(zB-zA);

    geometry_msgs::msg::Point r2;
    r2.x = xA + d2*(xB-xA);
    r2.y = yA + d2*(yB-yA);
    r2.z = zA + d2*(zB-zA);
    
    if (euclidean_distance(r1,p2,true) < euclidean_distance(r2,p2,true)) {    
      return r1;
    } else {   
      return r2;
    }
  }
  
  throw GeometryBoundsError("Line segment does not intersect sphere");      
}  
  
}  // namespace nav_drone_util

#endif  // NAV_DRONE_UTIL__GEOMETRY_UTILS_HPP_