cmake_minimum_required(VERSION 3.5)
project(local_zeroing_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(local_zeroing_node src/local_zeroing_node.cpp)
ament_target_dependencies(local_zeroing_node rclcpp geometry_msgs nav_msgs mavros_msgs)

install(TARGETS
  local_zeroing_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
