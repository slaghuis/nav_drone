cmake_minimum_required(VERSION 3.5)
project(nav_drone_costmap_3d)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

include_directories(include)

set(costmap_publisher_executable costmap_publisher)

set(library_name ${costmap_publisher_executable}_lib)  

add_library(${library_name} SHARED
  src/costmap_3d.cpp
  src/costmap_server.cpp
)
  
set(dependencies
  rclcpp
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
  octomap_msgs
)

ament_target_dependencies( ${library_name}
  ${dependencies}
)

add_executable(costmap_publisher
  src/main.cpp
)
  
target_link_libraries(costmap_publisher
  ${library_name}
)

ament_target_dependencies( costmap_publisher
  ${dependencies}
)
    
rclcpp_components_register_nodes(${library_name} "nav_drone_costmap_3d::CostmapPublisher")

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  costmap_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries( ${library_name})
ament_export_dependencies( ${dependencies})

ament_package()
