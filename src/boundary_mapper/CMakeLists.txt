cmake_minimum_required(VERSION 3.12)  # Updated to 3.12 to handle CMP0074 policy
project(boundary_mapper)

# Set CMP0074 policy to NEW to use <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}   # Include OpenCV directories
)

add_executable(boundary_mapper_node src/boundary_mapper_node.cpp)

ament_target_dependencies(boundary_mapper_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  visualization_msgs
  cv_bridge  # Added cv_bridge dependency
  OpenCV     # Added OpenCV dependency
)

target_link_libraries(boundary_mapper_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}  # Link OpenCV libraries
)

install(TARGETS
  boundary_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
