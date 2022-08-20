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

#include "nav_drone_util/angle_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav_drone_util {
	
	
// Calculate the angle between a line defined by two points and the coordinate axes.
// result is in RADIANS
double angle(const double x1, const double y1, const double x2, const double y2)
{
  if (x2 == x1) 
    return PI / 2;
  
  return atan( (y2-y1) /(x2-x1));
}




}