from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    lidar_camera_fusion_node = Node(
        package='ros2_lidar_camera_fusion_with_detection_cpp',
        executable='lidar_camera_fusion_with_detection',
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_depth': 0.2, 'max_depth': 10.0}
        ],
        remappings=[
            ('/scan/points', '/scan/points'),
            ('/interceptor/gimbal_camera_info', '/interceptor/gimbal_camera_info'),
            ('/interceptor/gimbal_camera', '/interceptor/gimbal_camera'),
            ('/yolo/tracking', '/yolo/tracking')
        ]
    )

    point_cloud_to_depth_map_node = Node(
        package='ros2_depth_map_pcl',
        executable='depth_map',
        name='point_cloud_to_depth_map',
        parameters=[
            {'width': 650, 'height': 650, 'scale': 50, 'MinDepth': 0.2, 'MaxDepth': 30.0}
        ],
        remappings=[
            ('/scan/points', '/scan/points'),
            ('/yolo/tracking', '/yolo/tracking')
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
            'model': '/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/yolo11s.pt',
            'threshold': '0.5',
            'input_image_topic': '/interceptor/gimbal_camera',
            'device': 'cuda:0'
        }.items()
    )

    return LaunchDescription([
        lidar_camera_fusion_node,
        point_cloud_to_depth_map_node,
        yolov8_launch
    ])

