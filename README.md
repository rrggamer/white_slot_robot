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

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false


