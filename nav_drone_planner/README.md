# Planner Server
An action server for ``nav_drone_msg::action::ComputePathToPose``. Typically called by the ``nav_drone_planner``. Implements a plugin interface for the ``nav_drone_core::planner`` class

## Depencencies
  - Only the packages listed in CMakeLists.txt

## Testing
Whilst running the:
  - [Drone](https://github.com/slaghuis/drone_mavsdk.git) node to publish odometry 
  - Octomap Server to publish an octomap
  - Something to populate the Octomap, either by file or by publishing a pointcloud. See [Sensor Pointcloud](https://github.com/slaghuis/sensor_pointcloud.git) for an example
  - A Planner plugin of your choice.  Here I run the dumb planner from this repository
  
Run the following and see a path returned
```
ros2 action send_goal /nav_drone/compute_path_to_pose "nav_drone_msgs/action/ComputePathToPose" '{planner_id: "DumbPlanner", use_start: True, start: {pose: {position: {x: 0, y: 0, z: 5}}}, goal: {pose: {position: {x: 1, y: 7, z: 5}}}}' 
```

# Code Status
This is still a **work in progress**.  This code has not flown in a simulator yet. (02/10/2022)
