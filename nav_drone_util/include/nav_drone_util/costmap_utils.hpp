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

#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_drone_util {
std::pair<double, double> calculate_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                       const geometry_msgs::msg::PoseStamped & target_pose);
	
} // namespace nav_drone