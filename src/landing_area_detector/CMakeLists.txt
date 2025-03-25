cmake_minimum_required(VERSION 3.5)
project(landing_area_detector)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories
include_directories(include)

# Create executable
add_executable(flat_area_detector src/flat_area_detector.cpp)
ament_target_dependencies(flat_area_detector rclcpp sensor_msgs pcl_conversions pcl_ros visualization_msgs)

install(TARGETS
  flat_area_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
