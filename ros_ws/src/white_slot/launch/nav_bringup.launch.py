import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # ---------------------------
    # Paths
    # ---------------------------
    pkg_white_slot = get_package_share_directory('white_slot')
    map_file = os.path.expanduser('~/maps/my_map.yaml')
    nav2_params_file = os.path.join(pkg_white_slot, 'config', 'nav2_params.yaml')

    # ---------------------------
    # YDLidar Node
    # ---------------------------
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[{
            'port': '/dev/ydlidar',
            'frame_id': 'laser_frame',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 4,
            'resolution_fixed': True,
            'inverted': False,
            'auto_reconnect': True,
            'isSingleChannel': True,
            'intensity': False,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 64.0,
            'range_min': 0.01,
            'frequency': 10.0
        }],
        output='screen'
    )

    # ---------------------------
    # Static Transform: base_link → laser_frame
    # ---------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # ---------------------------
    # Fuse Odometry Node
    # ---------------------------
    fuse_odom_node = Node(
        package="white_slot",
        executable="fuse_odom_node",
        output="screen",
    )

    # ---------------------------
    # Nav2 / AMCL Bringup
    # ---------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'  # or localization_launch.py for just AMCL
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # ---------------------------
    # Add all nodes to LaunchDescription
    # ---------------------------
    ld.add_action(ydlidar_node)
    ld.add_action(static_tf_node)
    ld.add_action(fuse_odom_node)
    # ld.add_action(nav2_launch)

    return ld
