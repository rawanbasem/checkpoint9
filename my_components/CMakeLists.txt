cmake_minimum_required(VERSION 3.8)
project(my_components)

# Default to C++14 or newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(attach_shelf REQUIRED) 

# 1. Create the Component Shared Library
add_library(pre_approach_component SHARED
  src/pre_approach_component.cpp
)

# 2. Add include directories
target_include_directories(pre_approach_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Add dependencies to the target
ament_target_dependencies(pre_approach_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  attach_shelf
)

# 4. Set visibility control macro
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")

# 5. Register the component with the container
rclcpp_components_register_nodes(pre_approach_component 
  "my_components::PreApproach"
)

# Install shared library
install(TARGETS pre_approach_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and rviz directories
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()