#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # YDLidar parameters
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    param_file = LaunchConfiguration('params_file')
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_x3.yaml'),
        description='Path to YDLidar parameters file'
    )

    # LiDAR node
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    # Static TF from base_link -> laser
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame']
    )

    # SLAM Toolbox online node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Online SLAM
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[('/scan','/scan')]
    )

    return LaunchDescription([
        declare_params,
        lidar_node,
        tf_node,
        slam_node
    ])
