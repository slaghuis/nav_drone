# Drone Navigation
Refactored the navigation lite package, interface definitions and plugins into a single ROS2 Package.  This is still work in progress.  Please refer back in a month or so.


## Depencencies
### BehaviorTree.CPP
The navigation server depends on the [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) library

### Octomap Server
All mapping activities is doen using the Octomap server.
```
sudo apt-get install ros-foxy-octomap-server
```

# Code Status
This is still a **work in progress**.  This code has not flown in a simulator yet. (29/09/2022)
