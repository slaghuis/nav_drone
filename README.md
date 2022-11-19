# Drone Navigation
Based on the [ROS2 Navigation Framework and System](https://github.com/ros-planning/navigation2) this is an attempt at building full 3D navigation.  The primary differences are:
 - All navigation is done in 3D space.  This is achieved by mainaining a dome of varying size around the robot that is informed from a 3D pointcould.  The initial case will run a Octomap Server.  This is a fundamental change the local costmap.  All planners and controllers are 3D spactially enabled.  This allows the drone the flu over or under an obstacle, rather than the long way around.
 - This packages still does not use Lifecycle Nodes.  One day...
 
All coding was started from scratch, but I referrenced a lot of the work done by the contributors to the [ROS2 Navigation Framework and System](https://github.com/ros-planning/navigation2).  Where appropriate I retained all licenase agreements.  I do believe this effort is in the spirit of open source code.  Please feel free to copy, use and alter my code.  If you sell it, please consider me.
 
## Depencencies
### BehaviorTree.CPP
The navigation server depends on the [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) library

### Octomap Server
All mapping activities is done using the Octomap server.
```
sudo apt-get install ros-foxy-octomap-server
```
The map can be initiated through a file upload at startup, an empty map can be populated by providing a pointcloud publisher, or both.  This allows for SLAM capability.

### Eigen
The costmap is built on a matrix leveraging the Eigen libraries.  Eigen is a lightweight C++ template library for linear algebra.
```
sudo apt-get install libeigen3-dev
```

# Code Status
This is still a **work in progress**.  This code has flown in a simulator using the Regulated Pure Pursuit Controller.  The code has not flown in real life  yet. (18/11/2022)
