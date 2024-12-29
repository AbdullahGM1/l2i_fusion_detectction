from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    lidar_camera_fusion_node = Node(
        package='ros2_lidar_camera_fusion_with_detection_cpp',
        executable='lidar_camera_fusion_with_detection_test',
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_depth': 0.2, 'max_depth': 10.0, # Setup your min and max depth range, where (x-axis) is the depth
             'camera_frame': 'interceptor/gimbal_camera'}  # Default target frame
        ],
        remappings=[
            # Replace with actual topic names
            ('/scan/points', '/scan/points'), # The lidar point cloud topic - replace the second topic to your topic
            ('/interceptor/gimbal_camera_info', '/interceptor/gimbal_camera_info'),# The camera info topic - replace the second topic to your topic
            ('/interceptor/gimbal_camera', '/interceptor/gimbal_camera'), # The camera image topic - replace the second topic to your topic
            ('/rgb/tracking', '/rgb/tracking') # The YOLO BB tracking topic - replace the second topic to your topic
        ]
    )

    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov11.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/test01.pt',
            'threshold': '0.5',
            'input_image_topic': '/interceptor/gimbal_camera',
            'namespace': 'rgb',  
            'device': 'cuda:0'
        }.items()
    )

    return LaunchDescription([
        lidar_camera_fusion_node,
        yolov8_launch
    ])
