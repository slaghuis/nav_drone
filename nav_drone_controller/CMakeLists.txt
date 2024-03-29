cmake_minimum_required(VERSION 3.5)
project(nav_drone_controller)

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
find_package(nav_drone_costmap_3d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_drone_core
  nav_drone_util
  nav_drone_msgs
  nav_drone_costmap_3d
  std_msgs
  geometry_msgs
  nav_msgs
  pluginlib)
  
include_directories(
  include
)  

add_library(controller_action_server SHARED
  src/controller_server.cpp)
target_include_directories(controller_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(controller_action_server
  PRIVATE "NAV_DRONE_BUILDING_DLL")
ament_target_dependencies(controller_action_server ${dependencies} )

add_library(simple_progress_checker SHARED plugins/simple_progress_checker.cpp)
ament_target_dependencies(simple_progress_checker ${dependencies})

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${dependencies})  

add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
target_link_libraries(stopped_goal_checker simple_goal_checker)
ament_target_dependencies(stopped_goal_checker ${dependencies})

rclcpp_components_register_node(controller_action_server PLUGIN "nav_drone_controller::ControllerServer" EXECUTABLE controller_server)

install(TARGETS
  simple_progress_checker simple_goal_checker stopped_goal_checker controller_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY include/
  DESTINATION include/
)  

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


ament_export_include_directories(include)
ament_export_libraries(simple_progress_checker
  simple_goal_checker
  stopped_goal_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav_drone_core plugins.xml)

ament_package()
