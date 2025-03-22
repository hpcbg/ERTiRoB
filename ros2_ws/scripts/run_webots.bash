source scripts/run_ros.bash
gnome-terminal --tab --title="Webots" -- webots ./src/board_webots_sim/worlds/door_with_probe_ROS2.wbt
ros2 launch ./launch/webots_simulation_launch.py