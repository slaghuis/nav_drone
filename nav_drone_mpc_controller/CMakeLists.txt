cmake_minimum_required(VERSION 3.5)
project(nav_drone_mpc_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_drone_core REQUIRED)
find_package(nav_drone_util REQUIRED)
find_package(nav_drone_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dlib REQUIRED)


include_directories(
  include
)

set(library_name ${PROJECT_NAME})
set(dependencies 
  ament_cmake
  pluginlib
  nav_drone_core
  nav_drone_util
  nav_drone_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  rclcpp
)

add_library(${library_name} SHARED
  src/mpc_controller.cpp
  src/histogram.cpp)
target_link_libraries(${library_name} dlib::dlib)  
ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(nav_drone_core nav_drone_mpc_controller.xml)

install(TARGETS ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
        
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav_drone_mpc_controller.xml
  DESTINATION share/${PROJECT_NAME}
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
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
