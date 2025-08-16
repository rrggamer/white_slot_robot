from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map', default='/home/papa/maps/my_map.yaml')

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),

        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'min_particles': 500,
                'max_particles': 2000,
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'scan_topic': 'scan'
            }]
        ),
    ])
