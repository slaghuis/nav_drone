cmake_minimum_required(VERSION 3.5)
project(nav_drone_costmap_3d)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_drone_core REQUIRED)
find_package(nav_drone_util REQUIRED)
find_package(nav_drone_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_library(costmap_publisher_lib SHARED
  src/costmap_3d.cpp
  src/costmap_server.cpp)
target_include_directories(costmap_publisher_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(costmap_publisher_lib
  PRIVATE "NAV_DRONE_COSTMAP_PUBLISHER_BUILDING_DLL")
ament_target_dependencies(costmap_publisher_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_drone_core
  nav_drone_util
  nav_drone_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  octomap
  octomap_msgs)
rclcpp_components_register_node(costmap_publisher_lib PLUGIN "nav_drone::Costmap_publisher" EXECUTABLE costmap_publisher)
install(TARGETS
  costmap_publisher_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
