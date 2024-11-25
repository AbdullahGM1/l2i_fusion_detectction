from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_camera_fusion_node = Node(
        package='ros2_lidar_camera_fusion_with_detection_cpp',  
        executable='lidar_camera_fusion_with_detection',  
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_depth': 0.2, 'max_depth': 10.0}  # Setup your min and max depth range
        ],
        remappings=[
            # Replace with actual topic names
            ('/scan/points', '/scan/points'),  # The lidar point cloud topic
            ('/interceptor/gimbal_camera_info', '/interceptor/gimbal_camera_info'),  # The camera info topic
            ('/interceptor/gimbal_camera', '/interceptor/gimbal_camera'),  # The camera image topic 
            ('/yolo/tracking', '/yolo/tracking')  # The YOLO BB tracking topic
        ]
    )

    return LaunchDescription([
        lidar_camera_fusion_node,
    ])
