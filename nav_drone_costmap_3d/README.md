## Costmap 3D ##

Builds a 2D Polar Histogram around the robot by looking for obstacles around the robot and recording the altitude and azimuth.  The obstacles are picked up from a Octomap. The Polar histogram is simplidied to bytes and published to Controller nodes.

This node is a vast simplification on the Costmap that we got used to in the  [ROS2 Navigation Framework and System](https://github.com/ros-planning/navigation2).  This node does not use layers and filters.  There surely is scope to extend the node to this level, but for now the code has been kept simple to ensure performance on the single onboard computer (Typicaly a Raspberry PI 4) that has to run the whole navigation stack.  
## Dependencies
### Octomap Server
This nodes listens to messages from the Octomap Server to obtain a view of all obstacles in the environment.
```
sudo apt-get install ros-foxy-octomap-server
``` 

### Eigen
The costmap is built on a matrix leveraging the Eigen libraries.  Eigen is a lightweight C++ template library for linear algebra.
```
sudo apt-get install libeigen3-dev
```
## Status
This code is under developent. (2022-11-worldcup)
