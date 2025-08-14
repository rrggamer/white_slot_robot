from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # YDLidar driver
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                                 'params', 'ydlidar_x3.yaml')]
    )

    # Static TF from base_link to laser
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0','0','0.02','0','0','0','1','base_link','laser_frame']
    )

    # SLAM Toolbox in online async mode
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'slam_mode': 'mapping',
            'scan_topic': '/scan',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'publish_map': True
        }]
    )
    
    odom_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_pub_odom',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
)

    return LaunchDescription([lidar_node, tf_node, slam_node,odom_tf_node])
