# Copyright (c) 2022 Eric Slaghuis
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Nodes launching commands
    costmap_publisher = Node(
            package='nav_drone_costmap_3d',
            name='costmap_publisher',
            executable='costmap_publisher',
            output='screen',
            emulate_tty=True, 
            parameters=[
                        {'map_frame'                          : 'map'},
                        {'robot_base_frame'                   : 'base_link'},   
                        {'transform_tolerance'                : 0.1},
                        {'lookahead_dist'                     : 2.0},
                        {'min_lookahead_dist'                 : 0.8},
                        {'max_lookahead_dist'                 : 4.0},
                        {'lookahead_time'                     : 1.5},
                        {'use_velocity_scaled_lookahead_dist' : False},
                        {'robot_radius'                       : 0.5},
                        {'safety_radius'                      : 0.3}
                       ]
    )


    ld = LaunchDescription()

    ld.add_action(costmap_publisher)

    return ld
