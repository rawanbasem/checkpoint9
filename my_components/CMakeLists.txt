cmake_minimum_required(VERSION 3.8)
project(my_components)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(attach_shelf REQUIRED) 

# --- PreApproach Component ---
add_library(pre_approach_component SHARED src/pre_approach_component.cpp)

target_include_directories(pre_approach_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pre_approach_component rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros attach_shelf)

target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")

# --- AttachClient Component ---
add_library(attach_client_component SHARED src/attach_client_component.cpp)

target_include_directories(attach_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(attach_client_component rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros attach_shelf)

target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")


install(TARGETS 
  pre_approach_component 
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()