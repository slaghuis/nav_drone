# Planner Server
An action server for ``nav_drone_msg::action::ComputePathToPose``. Typically called by the `nav_drone_bt_navigator` `ComputePathToPose` action from this repository. Implements a plugin interface for the ``nav_drone_core::planner`` class

## Depencencies
### Octomap Server
This nodes listens to messages from the Octomap Server to obtain a view of all obstacles in the environment.
```
sudo apt-get install ros-foxy-octomap-server
``` 
Be sure to configure the Octomap server and run the required nodes to publish the octomap.

## Testing
Whilst running the:
  - [Drone](https://github.com/slaghuis/drone_mavsdk.git) node to publish odometry 
  - Octomap Server to publish an octomap
  - Something to populate the Octomap, either by file or by publishing a pointcloud. See [Sensor Pointcloud](https://github.com/slaghuis/sensor_pointcloud.git) for an example
  - A Planner plugin of your choice.  Here I run the dumb planner from this repository
  
Run the following and see a path returned
```
ros2 action send_goal /nav_drone/compute_path_to_pose "nav_drone_msgs/action/ComputePathToPose" '{planner_id: "DumbPlanner", use_start: True, start: {header: {frame_id: 'map'}, pose: {position: {x: 0, y: 0, z: 5}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 1, y: 7, z: 5}}}}'

```

## Code Status
This is still a **work in progress**.  This code has flown in a simulator. (31/01/2023)
