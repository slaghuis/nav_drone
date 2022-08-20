cmake_minimum_required(VERSION 3.5)
project(nav_drone_planner)

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
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_library(planner_action_server SHARED
  src/planner_server.cpp)
target_include_directories(planner_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(planner_action_server
  PRIVATE "NAV_DRONE_BUILDING_DLL")
ament_target_dependencies(planner_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_drone_core"
  "nav_drone_util"
  "nav_drone_msgs"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs"
  "pluginlib"
  "octomap"
  "octomap_msgs")
rclcpp_components_register_node(planner_action_server PLUGIN "nav_drone::PlannerServer" EXECUTABLE planner_server)
install(TARGETS
  planner_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
