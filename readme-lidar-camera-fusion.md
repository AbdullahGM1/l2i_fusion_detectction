# ROS2 LiDAR-Camera Fusion Node

## Overview
This ROS2 node performs sensor fusion between LiDAR point cloud data and camera imagery, leveraging YOLO object detection for advanced spatial awareness and object tracking.

## Features
- LiDAR point cloud filtering and transformation
- Camera-LiDAR point projection
- YOLO object detection integration
- Multiple output topics:
  - `/image_lidar_fusion`: Annotated camera image with LiDAR points
  - `/detected_object_pose`: 3D poses of detected objects
  - `/detected_object_point_cloud`: Point clouds for each detected object

## Prerequisites
- ROS2 Humble
- Dependencies:
  - rclcpp
  - sensor_msgs
  - image_geometry
  - cv_bridge
  - pcl_conversions
  - tf2_ros
  - yolov8_msgs
  
## Installation

### Dependencies
```bash
sudo apt update
sudo apt install ros-humble-cv-bridge \
                 ros-humble-image-transport \
                 ros-humble-pcl-ros
```

### Clone and Build
```bash
# Assume you're in your ROS2 workspace
cd ~/ros2_ws/src
git clone <your-repository-url>
cd ..
colcon build --packages-select lidar_camera_fusion
```

## Configuration Parameters
- `min_depth` (default: 0.2): Minimum depth filter for point cloud
- `max_depth` (default: 10.0): Maximum depth filter for point cloud

## Subscribed Topics
- `/scan/points`: LiDAR point cloud
- `/interceptor/gimbal_camera_info`: Camera calibration info
- `/interceptor/gimbal_camera`: Camera image
- `/yolo/tracking`: YOLO object detections

## Published Topics
- `/image_lidar_fusion`: Annotated camera image
- `/detected_object_pose`: Object 3D poses
- `/detected_object_point_cloud`: Object-specific point clouds

## Node Workflow
1. Receive LiDAR point cloud
2. Filter points based on depth
3. Transform points to camera coordinate frame
4. Project 3D points to 2D image
5. Match points with YOLO bounding boxes
6. Compute object poses and point clouds

## Usage
```bash
ros2 run lidar_camera_fusion lidar_camera_fusion_node
```

## Customization
Modify node parameters in launch file or via command line:
```bash
ros2 run lidar_camera_fusion lidar_camera_fusion_node \
    --ros-args -p min_depth:=0.5 -p max_depth:=8.0
```

## Troubleshooting
- Ensure correct TF frames are published
- Verify camera and LiDAR calibration
- Check YOLO detection performance

## Performance Notes
- Computationally intensive for real-time processing
- Recommended for systems with good computational resources

## Future Improvements
- Add configurable confidence thresholds
- Implement multi-object tracking
- Enhance point cloud filtering techniques

## License
[Your License Here - e.g., MIT, Apache 2.0]

## Maintainer
[Your Name]
[Your Contact/Email]
