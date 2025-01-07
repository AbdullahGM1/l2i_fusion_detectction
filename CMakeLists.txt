cmake_minimum_required(VERSION 3.5)
project(ros2_lidar_camera_fusion_with_detection_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Declare the executable
add_executable(lidar_camera_fusion_with_detection src/lidar_camera_fusion_with_detection.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(lidar_camera_fusion_with_detection
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  cv_bridge
  image_geometry
  OpenCV
  yolov8_msgs
  message_filters
)

# Link additional libraries
target_link_libraries(lidar_camera_fusion_with_detection
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

# Install the executable
install(TARGETS
  lidar_camera_fusion_with_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_package()