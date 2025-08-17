# white_slot_robot

ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Configure the nodes first
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /bt_navigator configure

# Then activate them
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /bt_navigator activate


# 1 microros
ros2 launch white_slot joystick_launch.launch.py 

# 2 Lidar Start
ros2 launch white_slot nav_bringup.launch.py

# 3 Map Server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/papa/maps/my_map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# 4 amcl localization
ros2 launch nav2_bringup localization_launch.py map:=/home/papa/maps/my_map.yaml use_sim_time:=false

ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

# 5 navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# 6  nav --> /cmd_vel
ros2 run whiteslot nav_to_cmd_vel

# 7 

ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "poses:
- header: {frame_id: 'map'}
  pose: {position: {x: 0.595, y: -0.51, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 1.0, y: -0.53, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 2.16, y: -0.496, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 2.25, y: -1.43, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 2.23, y: -1.78, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 1.09, y: -1.86, z: 0.0}, orientation: {w: 1.0}}
- header: {frame_id: 'map'}
  pose: {position: {x: 0.728, y: -1.85, z: 0.0}, orientation: {w: 1.0}}"




room 1 0.595,-0.51,0 
room 2 1,-0.53,0
room 3 2.16,-0.496,0
room 4 2.25,-1.43,0
room 5 2.23,-1.78,0
room 6 1.09,-1.86,0
room 7 0.728,-1.85,0
 

