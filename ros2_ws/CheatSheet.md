# Commands Cheat Sheet 

## Installation & Set-Up

1. Install the latest Ubuntu 24

2. Install ROS 2 Jazzy and create ROS 2 workspace: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

3. Install Arduino IDE and other tools and libraries for the M5StickC PLUS2: https://docs.m5stack.com/en/arduino/arduino_ide

4. Install micro-ROS library for the Arduino IDE: https://github.com/micro-ROS/micro_ros_arduino?tab=readme-ov-file#arduino-ide

5. Initial build of the micro-ROS agent: https://micro.ros.org/docs/tutorials/core/first_application_linux/
```

```


## Arduino Upload

If the Arduino IDE cannot see the serial port you can either add your user to the correct dialout group or you can execute this when there is a problem: `sudo chmod 777 /dev/ttyACM0`


## ROS 2 Terminal Commands

1. Start micro-ROS node
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

2. Create a new Python package
```
source /opt/ros/jazzy/setup.bash
ros2 pkg create --build-type ament_python <pkg_name>
```

3. Build the workspace
```
. ./scripts/build.bash
```

4. Start the Board Recorder
```
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run board_recorder board_recorder
```

5. Start a new recording
```
ros2 action send_goal --feedback record board_recorder_interfaces/action/Record "{recording_name: '<name>'}"
```

6. Stop an existing recording
```
ros2 action send_goal --feedback stop board_recorder_interfaces/action/Stop "{recording_id: 1}"
```

7. Fetch recorded data
```
ros2 service call /fetch_recording board_recorder_interfaces/srv/FetchRecording "{recording_id: 1}"
```
