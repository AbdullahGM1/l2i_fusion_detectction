#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians
def generate_launch_description():
    ld = LaunchDescription()

    ns='observer'

    # Node for Drone 1
    world = {'gz_world': 'default'}
    # world = {'gz_world': 'ihunter_world'}
    model_name = {'gz_model_name': 'x500_camera_lidar'}
    autostart_id = {'px4_autostart_id': '4022'}
    instance_id = {'instance_id': '1'}
    # for 'default' world
    xpos = {'xpos': '0.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.1'}
    # For 'ihunter_world'
    # xpos = {'xpos': '-24.0'}
    # ypos = {'ypos': '8.0'}
    # zpos = {'zpos': '1.0'}
    headless= {'headless' : '0'}

    # PX4 SITL + Spawn x500_d435
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('smart_track'),
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_world': world['gz_world'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
        }.items()
    )

    # MAVROS
    file_name = 'observer_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('smart_track')
    plugins_file_path = os.path.join(package_share_directory, file_name)
    file_name = 'observer_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory, file_name)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('smart_track'),
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace' :ns+'/mavros',
            'tgt_system': '2',
            'fcu_url': 'udp://:14541@127.0.0.1:14558',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'observer/base_link',
            'odom_frame': 'observer/odom',
            'map_frame': 'map',
            'use_sim_time' : 'True'

        }.items()
    )

    odom_frame = 'odom'
    base_link_frame=  'base_link'

    # Static TF map/world -> local_pose_ENU
    map_frame='map'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), str(zpos['zpos']), '0.0', '0', '0', map_frame, ns+'/'+odom_frame],
    )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.0
    cam_y = 0.0
    cam_z = 0.0
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns+'_base2depth_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['child_frame'], ns+'/depth_camera'],
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link_frame, "camera_link"],
        
    )
       # Static TF base_link -> Lidar
    # 0 0 0.055 0 0 0
    lidar_x = 0.0
    lidar_y = 0.0
    lidar_z = 0.0
    lidar_roll = 0.0
    lidar_pitch = 0.0
    lidar_yaw = 0.0
    lidar_tf_node = Node(
        package='tf2_ros',
        name=ns+'_base2lidar_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['child_frame'], ns+'/depth_camera'],
        arguments=[str(lidar_x), str(lidar_y), str(lidar_z), str(lidar_yaw), str(lidar_pitch), str(lidar_roll), ns+'/'+base_link_frame,'x500_camera_lidar_1/lidar_link/gpu_lidar'],
    )
    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_sensors',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/rgb_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/gimbal/cmd_yaw@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gimbal/cmd_roll@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gimbal/cmd_pitch@std_msgs/msg/Float64]ignition.msgs.Double',
            '/imu_gimbal@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            
            '--ros-args',
            '-r', '/scan/points:='+ns+'/lidar_points',
            '-r', '/scan:='+ns+'/scan',

            '-r', '/rgb_image:='+ns+'/rgb_image',
            '-r', '/camera_info:='+ns+'/camera_info'
        ],
    )


    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov11.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/smart_track/config/test01.pt',
            'threshold': '0.5',
            'input_image_topic': 'observer/rgb_image',
            'namespace': '',  
            'device': 'cuda:0'
        }.items()
    )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('smart_track'), 'smart_track.rviz')]
    )

    lidar_camera_fusion_node = Node(
        package='ros2_lidar_camera_fusion_with_detection_cpp',
        executable='lidar_camera_fusion_with_detection',
        name='lidar_camera_fusion_node',
        parameters=[
            {'min_depth': 0.2, 'max_depth': 10.0, # Setup your min and max depth range, where (x-axis) is the depth
             'lidar_frame': 'x500_camera_lidar_1/lidar_link/gpu_lidar',  # Default source frame
             'camera_frame': 'camera_link'}  # Default target frame
        ],
        remappings=[
            # Replace with actual topic names
            ('/observer/lidar_points', '/observer/lidar_points'), # The lidar point cloud topic - replace the second topic to your topic
            ('/observer/camera_info', '/observer/camera_info'),# The camera info topic - replace the second topic to your topic
            ('/observer/rgb_image', '/observer/rgb_image'), # The camera image topic - replace the second topic to your topic
            ('/tracking', '/tracking') # The YOLO BB tracking topic - replace the second topic to your topic
        ]
    )

    gimbal_node = Node(
        package='smart_track',
        executable='gimbal_stabilizer',
        name='gimbal_stabilizer',
        output='screen',
        )

    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(ros_gz_bridge)
    # ld.add_action(kf_launch) # Estimates target's states based on position measurements( Reqiures yolov8_launch & yolo2pose_launch OR gt_target_tf)
    ld.add_action(yolov8_launch)
    ld.add_action(lidar_camera_fusion_node)
    ld.add_action(gimbal_node)
    ld.add_action(lidar_tf_node)

    # ld.add_action(yolo2pose_launch) # Comment this if you want to use the target ground truth (gt_target_tf.launch.py)
    ld.add_action(mavros_launch)
    ld.add_action(rviz_node)
    # ld.add_action(quadcopter_marker_launch)

    return ld