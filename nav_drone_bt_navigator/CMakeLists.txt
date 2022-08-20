cmake_minimum_required(VERSION 3.5)
project(nav_drone_bt_navigator)

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
find_package(nav_drone_util REQUIRED)
find_package(nav_drone_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(BehaviorTreeV3 REQUIRED)

add_library(navigate_to_pose SHARED
  src/navigate_to_pose.cpp
  src/action/compute_path_to_pose.cpp
  src/action/follow_path.cpp
  src/action/follow_waypoints.cpp
  src/action/read_goal.cpp
  src/control/pipeline_sequence.cpp
  src/control/recovery.cpp
  src/control/round_robin.cpp
  src/decorator/rate_controller.cpp)
target_include_directories(navigate_to_pose PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(navigate_to_pose BT::behaviortree_cpp_v3)  
target_compile_definitions(navigate_to_pose
  PRIVATE "NAV_DRONE_BUILDING_DLL")
ament_target_dependencies(navigate_to_pose
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav_drone_util"
  "nav_drone_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs")
rclcpp_components_register_node(navigate_to_pose PLUGIN "nav_drone::NavigationServer" EXECUTABLE bt_navigator)
install(TARGETS
  navigate_to_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS navigate_to_pose
  DESTINATION lib/${PROJECT_NAME})

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
