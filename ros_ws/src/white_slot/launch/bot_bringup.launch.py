import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()



    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB1"],
    )

    odom_node = Node(
        package="white_slot",
        executable="odom_node",
        output="screen",
        
    )

    no_imu_odom_node = Node(
        package="white_slot",
        executable="no_imu_odom_node",
        output="screen",
        
    )

        
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'slam.launch.py'
            )
        ])
    )
    

    ld.add_action(node_microros_1)
    # ld.add_action(odom_node)
    ld.add_action(no_imu_odom_node)
    ld.add_action(slam_launch)



    return ld