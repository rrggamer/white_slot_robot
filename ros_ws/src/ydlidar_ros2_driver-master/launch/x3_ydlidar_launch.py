from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_x3.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file]
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',  # <--- updated argument
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser']
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        # tf2_node,
    ])
