# Controller Server
An action server for `nav_drone_msg::action::FollowPath`.  Typically called by the `nav_drone_bt_navigator` `FollowPath` action from this repository.  Implements a plugin interface for the `nav_drone_core::controller` class

## Dependencies
  - Only the dependencies listed in the CMakeLists.txt

## Testing
Whilst running a simulator of your choice, run:
  - Drone node to publish odometry
  - Publish a suitible tf2 tree - See teh Drone node documentation
  - Run an Octomap server to publish an octomap
  - Something to pupulate the Ocotomap server, either by loadinga file or publishing a pointcloud
  - A controller plugin.  In this case we use the MPC controller from this repository

After issuing a takeoff command, run this in a seperate terminal.  If all goes well, the drone should move along the path.
```
ros2 action send_goal nav_drone/follow_path "nav_drone_msgs/action/FollowPath" '{controller_id: MPCController, path: {header: {frame_id: 'map'}, poses: [{pose: {position: {x: 0,y: 0,z: 2}, orientation: {x: 0, y: 0, z: 0, w: 1}}, header: {frame_id: 'map'}}, {pose: {position: {x: 8,y: 5,z: 2}, orientation: {x: 0, y: 0, z: 0, w: 1}}, header: {frame_id: 'map'}}]}}'
```
# Code Status
This code is under testing as from 09 Sept 2022.
