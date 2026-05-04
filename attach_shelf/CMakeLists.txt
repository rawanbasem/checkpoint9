cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES geometry_msgs
)

# --- Task 1 Executable ---
add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

# --- Task 2 Client Executable ---
add_executable(pre_approach_v2_node src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2_node
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)
# Link custom service
rosidl_target_interfaces(pre_approach_v2_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- Task 2 Server Executable ---
add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)
# Link custom service
rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 3. Install section
install(TARGETS
  pre_approach_node
  pre_approach_v2_node
  approach_service_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()