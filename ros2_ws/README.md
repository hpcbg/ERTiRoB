# Task Board ROS 2 Packages

This folder contains all the related packages. Packages are developed for ROS 2 Jazzy for Ubuntu 24.

## Install

The initial install requires execution of the script `./scripts/initial_build.bash` from the `ros2_ws` folder.

## Launch

After the initial build the packages can be launched in different configurations. The launch directory contains various launch configurations.

The complete configuration which also includes Rosbridge WebSocket server can be started with the following command `ros2 launch ./launch/complete_launch.py`