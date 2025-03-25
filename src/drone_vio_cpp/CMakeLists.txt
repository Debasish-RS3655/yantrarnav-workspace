cmake_minimum_required(VERSION 3.8)
project(drone_vio_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(odom_to_vision_pose src/odom_to_vision_pose.cpp)

# Link dependencies
ament_target_dependencies(odom_to_vision_pose
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  odom_to_vision_pose
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()