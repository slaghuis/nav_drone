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

#include <cmath>
#include "nav_drone_util/costmap_utils.hpp"

namespace nav_drone_util {

std::pair<double, double> calculate_ez(const geometry_msgs::msg::PoseStamped & current_pose,
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
  
} // namespace nav_drone