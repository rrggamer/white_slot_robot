from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR driver
        Node(
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
        ),

        # Static TF from base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),
    ])
