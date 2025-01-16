#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    ns = 'observer'

    lidar_camera_fusion_node = Node(
        package='l2i_fusion_detection',
        executable='lidar_camera_fusion_with_detection',
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_range': 0.2, 'max_range': 5.0,
             'lidar_frame': 'camera_depth_optical_frame',
             'camera_frame': 'camera_color_optical_frame'}
        ],
        remappings=[
            ('/scan/points', '/camera/camera/depth/color/points'),
            ('/observer/gimbal_camera_info', '/camera/camera/color/camera_info'),
            ('/observer/gimbal_camera', '/camera/camera/color/image_raw'),
            ('/rgb/tracking', '/rgb/tracking')
        ]
    )

    # YOLO Launch for Lidar-Camera Fusion
    yolo_launch_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch/yolov11.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/l2i_fusion_detection/config/rgb.pt',
            'threshold': '0.5',
            'input_image_topic': '/camera/camera/color/image_raw',
            'namespace': 'rgb',
            'device': 'cuda:0'
        }.items()
    )


    # Add all nodes and launches to the launch description
    ld.add_action(lidar_camera_fusion_node)
    ld.add_action(yolo_launch_fusion)


    return ld
