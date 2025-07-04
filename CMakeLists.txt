cmake_minimum_required(VERSION 3.8)
project(my_components)

# Default to C++14
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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(attach_shelf REQUIRED)

# Include directories
include_directories(include)

# Create the pre_approach component library
add_library(pre_approach_component SHARED
  src/pre_approach_component.cpp
)
target_compile_definitions(pre_approach_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_nodes(pre_approach_component 
  "my_components::PreApproach"
)

# Create the attach_server component library
add_library(attach_server_component SHARED
  src/attach_server_component.cpp
)
target_compile_definitions(attach_server_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  attach_shelf
)
rclcpp_components_register_nodes(attach_server_component 
  "my_components::AttachServer"
)

# Create the attach_client component library
add_library(attach_client_component SHARED
  src/attach_client_component.cpp
)
target_compile_definitions(attach_client_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  rclcpp
  rclcpp_components
  attach_shelf
)
rclcpp_components_register_nodes(attach_client_component 
  "my_components::AttachClient"
)

# Install libraries
install(TARGETS
  pre_approach_component
  attach_server_component
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz files
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()