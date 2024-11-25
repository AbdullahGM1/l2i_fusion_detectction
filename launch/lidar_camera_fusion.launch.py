from launch import LaunchDescription
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

    return LaunchDescription([
        lidar_camera_fusion_node,
    ])

