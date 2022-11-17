# BT Navigator
The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its Configuration Guide Page for additional parameter descriptions, as well as the Nav2 Behavior Tree Explanation pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic BehaviorTreeEngine class which uses the Behavior-Tree.CPP library to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

## Depencencies
### BehaviorTree.CPP
The navigation server depends on the [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) library

## Usage
Call the action server 
```
ros2 action send_goal nav_drone/navigate_to_pose "nav_drone_msgs/action/NavigateToPose" '{pose: {pose: {position: {x: 5.0, y: 1.0, z: 2.0}, orientation: { x: 0, y: 0, z:0, w: 1}}, header: {frame_id: `map`}}, behavior_tree: sample.xml}'
```

# Code Status
This is still a **work in progress**.  This code has not flown in a simulator yet. (17/11/2022)
