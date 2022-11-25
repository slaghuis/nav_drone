add_library(${library_name} SHARED
  angle_utils.cpp
  costmap_utils.cpp
  node_thread.cpp
  node_utils.cpp
  robot_utils.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  nav_drone_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

