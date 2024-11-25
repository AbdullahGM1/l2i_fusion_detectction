# ROS2 Camera-Lidar Fusion Package

## Description

This ROS2 package fuses 360-degree lidar and camera data for enhanced object tracking. It transforms lidar point clouds from the lidar frame to the camera frame and overlays the points within the detected object bounding boxes onto the image. The system estimates the 3D positions of the detected objects by averaging the point cloud (x, y, z) within the bounding box and publishes these points as point cloud data. This enables real-time tracking and position estimation for multiple objects. 

Previously, I created a similar package in Python: [ros2_lidar_camera_fusion_with_detection](https://github.com/AbdullahGM1/ros2_lidar_camera_fusion_with_detection/tree/main). In this new version, developed in C++ for improved performance with large lidar point cloud data, I use the PCL library to handle the point clouds efficiently. Additionally, I have enhanced the code to handle static transforms (tf) for the sensors better, optimizing overall system performance.---

## Table of Contents
1. [Demonstration](#demonstration)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Node](#node)
6. [Contributing](#contributing)

---
## Demonstration

### Lidar-Camera Fusion in Action


<p align="center">
- Overlays the detected object point cloud points onto the image:
</p>


<p align="center">
  <img src="images/1.png" alt="Lidar-Camera Fusion Example 1" width="500"/>
  <img src="images/2.png" alt="Lidar-Camera Fusion Example 2" width="500"/>
</p>

<p align="center">
  <img src="images/Camera_Lidar_Fusion.gif" alt="Lidar-Camera Fusion in Action gif" width="500"/>
</p>

<p align="center">
- Publish the points within the detected object bounding boxes as point cloud points.
</p>


<p align="center">
  <img src="images/4.png" alt="Lidar-Camera Fusion Example 1" width="500"/>
  <img src="images/Camera_Lidar_Fusion_point_cloud_.gif" alt="Lidar-Camera Fusion Example 2" width="500"/>
</p>

---

## Features

- **Dynamic Transform Handling**: Employs a tf2 buffer and listener to manage and apply transformations between coordinate frames dynamically, ensuring data consistency across different sensor inputs.
- **3D Position Estimation**: Calculates the average (x, y, z) of point clouds within object bounding boxes to estimate 3D positions.
- **Interactive Visualization**: Offers real-time visual feedback by projecting lidar points of the detected objects onto camera images, enhancing the visual assessment of alignment and accuracy in the sensor fusion process.
- **Detected Object Point Cloud Streaming**: Publishes the points within bounding boxes (BB) as distinct point clouds for each detected object.
- **Multi-Object Detection and Localization**: Simultaneously detects and estimates positions for multiple detected objects in real-time.
---

## Installation

### Prerequisites
- **ROS2 Humble**: Ensure you have ROS2 Humble installed on your machine. [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **YOLOvX**: Follow the instructions to set up YOLOvX in ROS2 for object detection. [Installation Guide](https://github.com/mgonzs13/yolov8_ros) 

### Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2_lidar_camera_fusion_with_detection.git
```
### Install Dependencies
Run `rosdep` to install any missing dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
### Build the Package
After cloning and installing dependencies, build your package:
```bash
colcon build --packages-select ros2_lidar_camera_fusion_with_detection
```
---

## Usage

### Modifying the Package for Your Setup
Before running the package, make sure to modify the `setup_config.yaml` file located in the `/ros2_lidar_camera_fusion_with_detection/config` directory to match your setup:

1. **Configure the Transformation Matrix**: Modify the transformation matrix between the Lidar and the camera to match your setup by updating
   Example:
```
transformation_matrix:
  - [0, -1, 0, 0.1]
  - [0, 0, -1, 0]
  - [1, 0, 0, 0]
  - [0, 0, 0, 1]
```

3. **Specify the Depth Range**: Set the depth range for points that should be transformed. In this example, only points between 0.5 and 10 meters are considered.
   Example:
   
```
  depth_range:
  min: 0.5  # Minimum depth
  max: 10   # Maximum depth
```
### Build the Package
After modifying the code, build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_camera_fusion_with_detection
```
### Source the Workspace
Before running the package, ensure you source the workspace to have access to the built packages:

For **Bash**:
```bash
source ~/ros2_ws/install/setup.bash
```

### Run the Node
To run the package with your custom launch file (make sure you specify it):
```bash
ros2 run ros2_lidar_camera_fusion_with_detection lidar_camera_fusion_with_detection
```

---
## Node

### `lidar_camera_fusion_node`

This node fuses lidar point cloud data onto the camera image frame and overlays the points within detected object bounding boxes onto the image. Also, publishing the points within the bounding box as point cloud points.

#### Subscribed Topics:

-**`/scan/points`**: Lidar point cloud data.

-**`/interceptor/gimbal_camera`**: Camera image output.

-**`/interceptor/gimbal_camera_info`**: Camera info for the Camera Intrinsic.

-**`/yolo/tracking`**: Detected objects with bounding boxes.


#### Published Topics:

-**`/image_lidar`**: Image with projected Lidar points.

-**`/detected_object_distance`**: Average distance from the detected objects to the camera frame.

-**`/detected_object_pointcloud`**: Points inside the bounding boxes are published as point cloud points.

---
## Contributing

Feel free to contribute to this project by creating pull requests or opening issues.
