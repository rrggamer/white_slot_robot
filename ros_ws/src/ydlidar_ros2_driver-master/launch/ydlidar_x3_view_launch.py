#!/usr/bin/python3
# Launch file for YDLidar X3 with RViz visualization

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
import os

def generate_launch_description():
    # Package directories
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config', 'ydlidar.rviz')

    # Launch configuration for parameters file
    parameter_file = LaunchConfiguration('params_file')

    # Declare the launch argument for params file
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_x3.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # YDLidar driver node (lifecycle)
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file]
    )

    # Static TF for the laser frame
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
    )

    # RViz node to visualize the laser
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        rviz_node
    ])
