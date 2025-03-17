<!-- omit in toc -->

# ROS 2 Task Board Recorder

This part of the repo contains all the packages of the develop ROS 2 system for communication and recording of the task board sensor events published by the task board to ROS 2 topics.

The updated version of the firmware of the task board is supposed to publish all of the sensor events to a specific ROS 2 topic for each task board sensor. The ROS 2 task board recorder node will provide the user interface for control of the recording process as actions and services.

Packages are developed and running for ROS 2 Jazzy for Ubuntu 24.04.1 LTS.

Note: the current task board recorder node records only events when a new (different) value is published to the corresponding topic.

<!-- omit in toc -->

## Contents

- [ROS 2 Task Board Recorder](#ros-2-task-board-recorder)
  - [Contents](#contents)
  - [Installation](#installation)
  - [Task board configuration](#task-board-configuration)
  - [Launch](#launch)
  - [Additional packages compilation or rebuild](#additional-packages-compilation-or-rebuild)
  - [Actions provided by the task board recorder](#actions-provided-by-the-task-board-recorder)
    - [Task board management](#task-board-management)
    - [Recordings control](#recordings-control)
  - [Services provided by the task board recorder](#services-provided-by-the-task-board-recorder)
    - [Task board management](#task-board-management-1)
    - [Fetching recordings data](#fetching-recordings-data)
    - [Fetching task board sensors data](#fetching-task-board-sensors-data)

## Installation

Ubuntu 24.04.1 LTS is required. ROS 2 Jazzy Jalisco must be istalled as described in the official ROS 2 documentation: [https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html).

After that this repo can be cloned and the initial install requires the script [`./scripts/initial_build.bash`](./scripts/build.bash) to be executed from the `ros2_ws` folder. This script will install the `micro_ros_setup` package, set up a new `micro_ros_node` and build all of the packages.

The `rosbridge_suite` which is useful for a WebSocket communication will also be installed. Please, note that modern browsers reuqire a Web Socket Secure (WSS) communication. So, a self-signed certificate is provided in the folder [./launch/rosbridge_certificate](./launch/rosbridge_certificate/). If this certificate is used you might be required to add it as an exception into your browser by loading `https://<rosbridge-address>:<rosbridge-port>`.

## Task board configuration

The developed ROS 2 node can be easily configured with a JSON configuration file. Sample configuration file which is used by the default launch file and startup scripts is stored in [./board_configs/default.json](./board_configs/default.json).

Each configuration file should contain a list of JSON objects for each task board sensor with the following set of name/value pairs:

| name       | value                                                                                                                                                                                                                | sample value           |
| ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- |
| `name`     | String with the name of the sensor. It is not supposed to contain any whitespace characters.                                                                                                                         | `"button_A"`           |
| `type`     | String with a ROS 2 type.                                                                                                                                                                                            | `"std_msgs/msg/Int32"` |
| `topic`    | String with a ROS 2 topic.                                                                                                                                                                                           | `"/button/A_status"`   |
| `initial`  | The initial value of the sensor.                                                                                                                                                                                     | `-1`                   |
| `deadzone` | Integer which defines the dead zone of the sensor. Useful for analog sensors like potentiometers to limit the amount of transmitted data. If this value is set to `0` any new data from the sensor will be recorded. | `50`                   |
| `timeout`  | Integer which defines the sampling time of the sensor in ms. If this value is set to `0` any new data from the sensor will be recorded without any timeout.                                                          | `10000`                |

## Launch

The complete automated way to start the task board recorder node for the default test board configuration is to `cd` to the `ros2_ws` folder and execute the script [`./scripts/start.bash`](./scripts/start.bash).

The complete automated way to start the task board recorder node for Task Board version 2023 (TBv2023) is to `cd` to the `ros2_ws` folder and execute the script [`./scripts/start_TBv2023.bash`](./scripts/start_TBv2023.bash).

You can play a recording stored in a JSON file by using the `recording_player` node. Sample configuration for the node is stored in the launch file [`./launch/play_test_recording_launch.py`](./launch/play_test_recording_launch.py). The `timeout` parameter sets the seconds for which the start of the recording is delayed. Useful if you need to wait for the loading of the visualization software. You ca run the launch file with the following command `ros2 launch ./launch/play_test_recording_launch.py`. You can also play a recording of the execution of the ROBOTHON_2023 protocol executed on a Task Board version 2023 with the following command `ros2 launch ./launch/play_robothon_2023_recording_launch.py`. For this recording the angle sensor is mounted, so, that it shows 4035 when door is fully closed, 4030 - 0 degrees, 2770 - 90 degrees, 2365 - fully opened. Yon can visualize the recording with `ros2 launch ./launch/display_TBv2023_launch.py`.

If you want to execute only a specific package you can source the ROS 2 environment by execution of [`./scripts/run_ros.bash`](./scripts/run_ros.bash) from the `ros2_ws` folder. Packages can be launched in different configurations. The [./launch](./launch/) directory contains various launch configurations. The complete configuration which also starts a rosbridge WebSocket server can be started with the following command `ros2 launch ./launch/complete_launch.py`.

## Additional packages compilation or rebuild

If you need to compile additional packages the script [`./scripts/build.bash`](./scripts/build.bash) is provided. It will skip packages related to the `micro_ros` and also the `board_recorder_interfaces` package.

The `board_recorder_interfaces` package can be compiled with `colcon build --packages-select board_recorder_interfaces`.

## Actions provided by the task board recorder

There actions for task board manaagement and for recordings control.

### Task board management

The developed packages provide the following action for task board management:

1. `/remove_task_board` of type `board_recorder_interfaces/action/RemoveTaskBoard` - action for removing all of the task board from the local database.

   - This action requires the parameter `string task_board_id` which is the unique id of the task board.

   - This action will return as a result `bool success` which will be `true` if the data for the provided `task_board_id` were removed successfully.

   - This action will return as a feedback `bool is_authorized`. This field is reserved if some kind of user authentication is implemented in the future.

   - Sample call: `ros2 action send_goal --feedback /remove_task_board board_recorder_interfaces/action/RemoveTaskBoard "{task_board_id: 'e8b4b12f2b14'}"`

### Recordings control

The developed packages provide the following actions for control of the recording process:

1. `/record` of type `board_recorder_interfaces/action/Record` - action for starting a new recording.

   - This action requires the following parameters:

     - `string task_board_id` - the unique id of the task board;
     - `string protocol` - the name of the current protocol executed by the robot.

   - This action will return as a result `int32 recording_id` which will be the id of the started recording.

   - This action will return as a feedback `bool is_busy` which will be `true` if the recording was already started.

   - Note: if the recording was already started the id of the current recording will be returned.

   - Sample call: `ros2 action send_goal --feedback /record board_recorder_interfaces/action/Record "{task_board_id: 'e8b4b12f2b14', protocol: 'protocol_name'}"`

2. `/stop` of type `board_recorder_interfaces/action/Stop` - action for stopping of the corresponding recording.

   - This action requires the parameters:

     - `string task_board_id` - the unique id of the task board;
     - `int32 recording_id` - this should be the id of the current recording.

   - This action will return as a result `bool success` which will be `true` if the provided `recording_id` is equal to the current recording id.

   - This action will return as a feedback `bool is_authorized`. This field is reserved if some kind of user authentication is implemented in the future.

   - Sample call: `ros2 action send_goal --feedback /stop board_recorder_interfaces/action/Stop "{task_board_id: 'e8b4b12f2b14', recording_id: 1739219462}"`

## Services provided by the task board recorder

There are services for task board management, fetching recordings data and fetching task board sensors data.

### Task board management

The developed packages provide the following services for task board management:

1. `/fetch_task_boards` of type `board_recorder_interfaces/srv/FetchTaskBoards` - service for retrieving the list of the task boards.

   - This sevice has no request parameter.

   - This service returns `string task_boards_json` which contains the current list of the task boards as a JSON.

   - Sample call: `ros2 service call /fetch_task_boards board_recorder_interfaces/srv/FetchTaskBoards "{}"`

2. `/fetch_task_board_recordings` of type `board_recorder_interfaces/srv/FetchTaskBoardRecordings` - service for retrieving the id of the current recording.

   - This sevice has a request parameter `string task_board_id` which is the unique id of the task board.

   - This service returns `string task_board_recordings_json` which contains the id of the task board and all recordings for this task board.

   - Sample call: `ros2 service call /fetch_task_board_recordings board_recorder_interfaces/srv/FetchTaskBoardRecordings "{task_board_id: 'e8b4b12f2b14'}"`

3. `/fetch_task_board_protocols` of type `board_recorder_interfaces/srv/FetchTaskBoardProtocols` - service for retrieving the id of the current recording.

   - This sevice has a request parameter `string task_board_id` which is the unique id of the task board.

   - This service returns `string task_board_protocols_json` which contains list with all of the recorded protocols for this task board.

   - Sample call: `ros2 service call /fetch_task_board_protocols board_recorder_interfaces/srv/FetchTaskBoardProtocols "{task_board_id: 'e8b4b12f2b14'}"`

### Fetching recordings data

The developed packages provide the following services for fetching the recordings data:

1. `/fetch_current_recording_id` of type `board_recorder_interfaces/srv/FetchCurrentRecordingId` - service for retrieving the id of the current recording.

   - This sevice has a request parameter `string task_board_id` which is the unique id of the task board.

   - This service returns `int32 recording_id` which contains the id of the current recording. Value of `-1` will be returned if there is no active recording.

   - Sample call: `ros2 service call /fetch_current_recording_id board_recorder_interfaces/srv/FetchCurrentRecordingId "{task_board_id: 'e8b4b12f2b14'}"`

2. `/fetch_newest_recordings` of type `board_recorder_interfaces/srv/FetchNewestRecordings` - service for retrieving the info of the newest recordings which are stored in the database.

   - This sevice has the following request parameters:

     - `string task_board_id` - the unique id of the task board;
     - `string protocol` - the name of the current protocol for the recordings or leave empty for fetching recordings with any protocol;
     - `int32 count` - for limiting the number of the returned recordings. If this parameter is set to `0` the service will return all recordings.

   - This service returns `string recordings_list_json` which contains JSON object with the info of the corresponding recordings.

   - Sample call: `ros2 service call /fetch_newest_recordings board_recorder_interfaces/srv/FetchNewestRecordings "{task_board_id: 'e8b4b12f2b14', protocol: 'protocol_name', count: 10}"`

3. `/fetch_recording` of type `board_recorder_interfaces/srv/FetchRecording` - service for retrieving a specific recording.

   - This sevice has the following request parameters:

     - `string task_board_id` - the unique id of the task board;
     - `int32 recording_id` which is the id of the requested recording.

   - This service returns `string recording_json` which contains JSON object with the requested recording.

   - Sample call: `ros2 service call /fetch_recording board_recorder_interfaces/srv/FetchRecording "{task_board_id: 'e8b4b12f2b14', recording_id: 1739220569}"`

4. `/fetch_recording_events` of type `board_recorder_interfaces/srv/FetchRecordingEvents` - service for retrieving only specific events from a specific recording.

   - This sevice has two request parameters:

     - `string task_board_id` - the unique id of the task board;
     - `int32 recording_id` which is the id of the requested recording;
     - `float32 from_time` which is the time in seconds from which the events are requested. Note: the time interval is inclusive.

   - This service returns `string events_json` which contains JSON object with the requested events.

   - Sample call: `ros2 service call /fetch_recording_events board_recorder_interfaces/srv/FetchRecordingEvents "{task_board_id: 'e8b4b12f2b14', recording_id: 1739220569, from_time: 5}"`

### Fetching task board sensors data

The developed packages provide the following services for fetching the current data of any sensor:

1. `/fetch_sensor_names` of type `board_recorder_interfaces/srv/FetchSensorNames` - service for retrieving all of the sensor names.

   - This sevice has a request parameter `string task_board_id` which is the unique id of the task board.

   - This service returns `string sensor_names_json` which contains list of all sensor names as a JSON.

   - Sample call: `ros2 service call /fetch_sensor_names board_recorder_interfaces/srv/FetchSensorNames "{task_board_id: 'e8b4b12f2b14'}"`

2. `/fetch_sensor_data` of type `board_recorder_interfaces/srv/FetchSensorData` - service for retrieving the current data of a specific sensor.

   - This sevice has the following request parameters:

     - `string task_board_id` - the unique id of the task board;
     - `string sensor_name` - the name of the requested sensor.

   - This service returns `string data_json` which contains JSON object with the corresponding sensor data.

   - Sample call: `ros2 service call /fetch_sensor_data board_recorder_interfaces/srv/FetchSensorData "{task_board_id: 'e8b4b12f2b14', sensor_name: 'voltage'}"`
