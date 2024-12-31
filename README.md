# 🤖 ROS2 Camera-Lidar Fusion Package

## 📝 Description

This ROS2 package provides advanced sensor fusion capabilities by combining 360-degree lidar and camera data for enhanced object tracking and localization. The package transforms lidar point clouds from the lidar frame to the camera frame, overlaying detected object point clouds onto the image. By averaging the point cloud coordinates within object bounding boxes, the system enables real-time 3D position estimation for multiple objects.

## 🚀 Key Improvements from Previous Version
- Reimplemented in C++ for improved performance with large lidar point cloud datasets
- Utilizes PCL (Point Cloud Library) for efficient point cloud processing
- Enhanced static transform (tf) handling for optimized sensor integration

## 📋 Table of Contents
1. [Demonstration](#demonstration)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Node Details](#node)
6. [Contributing](#contributing)

## 🎥 Demonstration

### 📸 Lidar-Camera Fusion Visualization

<p align="center">
Overlaying Detected Object Point Cloud Points onto Camera Image:
</p>

<p align="center">
  <img src="images/1.png" alt="Lidar-Camera Fusion Example 1" width="500"/>
  <img src="images/2.png" alt="Lidar-Camera Fusion Example 2" width="500"/>
  <img src="images/Camera_Lidar_Fusion.gif" alt="Lidar-Camera Fusion in Action" width="500"/>
</p>

<p align="center">
Publishing Points Within Detected Object Bounding Boxes:
</p>

<p align="center">
  <img src="images/4.png" alt="Lidar-Camera Fusion Point Cloud" width="500"/>
  <img src="images/Camera_Lidar_Fusion_point_cloud_.gif" alt="Lidar-Camera Fusion Point Cloud Demonstration" width="500"/>
</p>

## ✨ Features

- **Dynamic Transform Management**: Advanced tf2 buffer and listener for seamless coordinate frame transformations
- **Precise 3D Position Estimation**: Calculates average (x, y, z) coordinates of point clouds within object bounding boxes
- **Real-time Visualization**: Interactive projection of lidar points onto camera images
- **Comprehensive Object Tracking**:
  - Detected object point cloud streaming
  - Simultaneous multi-object detection and localization
- **Full ROS2 Integration**: Seamless compatibility with ROS2 robotics ecosystem

## 🛠️ Installation

### 📋 Prerequisites
- **🤖 ROS2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **🕵️ YOLOvX ROS** ([Setup Instructions](https://github.com/mgonzs13/yolov8_ros))
- **💻 C++ Compiler**: GCC 8 or newer
- **📚 Required Libraries**: PCL, OpenCV, and standard ROS2 dependencies

### 📦 Dependency Installation
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev
```

### 📂 Repository Setup
```bash
cd ~/ros2_ws/src
git clone https://github.com/AbdullahGM1/ros2_lidar_camera_fusion_with_detection_cpp.git
```

### 🏗️ Build Process
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_camera_fusion_with_detection_cpp
source install/setup.bash
```

## 🚀 Usage

The launch file is provided:
- `lidar_camera_fusion_yolo.launch.py`: Runs fusion node with YOLO detection node. 

### ⚙️ Configuration Steps

Before running, modify the launch `lidar_camera_fusion_yolo.launch.py` file in `ros2_lidar_camera_fusion_with_detection_cpp/launch`:

1. **Depth Range Configuration**:
```python
parameters=[
    {'min_depth': 0.2, 'max_depth': 10.0}  # Adjust lidar depth range
]
```

2. **Sensor Frame Setup**:
   Add your camera frame "Target Frame". 
```python
'camera_frame': 'interceptor/gimbal_camera'
```

3. **Topic Remapping**:
```python
remappings=[
    ('/scan/points', '/lidar/topic/name'),
    ('/interceptor/gimbal_camera_info', '/camerainfo/topic/name'),
    ('/interceptor/gimbal_camera', '/camera/topic/name'),
    ('/yolo/tracking', '/yolo/tracking/topic/name')
]
```

4. **YOLO Configuration**:
```python
launch_arguments={
    'model': '/path/to/model.pt',
    'threshold': '0.5',
    'input_image_topic': '/interceptor/gimbal_camera',
    'device': 'cuda:0'
}.items()
```

### Execution
```bash
#Run with YOLO detection
ros2 launch ros2_lidar_camera_fusion_with_detection_cpp lidar_camera_fusion_yolo.launch.py
```

## 🧩 Node Details: `lidar_camera_fusion_node`

### 📡 Subscribed Topics
- `/scan/points`: Lidar point cloud data
- `/interceptor/gimbal_camera`: Camera image
- `/interceptor/gimbal_camera_info`: Camera information
- `/yolo/tracking`: Detected object bounding boxes

### 📤 Published Topics
- `/image_lidar`: Image with projected Lidar points
- `/detected_object_distance`: Average distance from detected objects
- `/detected_object_pointcloud`: Point cloud of objects within bounding boxes

## 🤝 Contributing

Feel free to contribute to this project by creating pull requests or opening issues! 🌟
