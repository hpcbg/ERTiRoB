import json
from pydoc import locate

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from board_recorder_interfaces.srv import FetchRecording, FetchSensorNames, FetchSensorData
from board_recorder_interfaces.action import Record, Stop


class BoardRecorder(Node):

    def __init__(self):
        super().__init__('board_recorder')

        from rcl_interfaces.msg import ParameterDescriptor
        board_config_descriptor = ParameterDescriptor(
            description='Path to the board configuration')
        self.declare_parameter('board_config', './board_configs/default.json',
                               board_config_descriptor)

        self.init_board()

        self.init_recordings_db()

        self.init_actions()

        self.init_services()

    def fetch_recording_callback(self, request, response):
        id = request.recording_id

        recording = {
            'recording_id': id,
            'status': 'Not Found',
            'data': ''
        }

        if id in self.recordings:
            recording['status'] = self.recordings[id]['status']
            recording['data'] = self.recordings[id]
        response.recording_json = json.dumps(recording)

        self.get_logger().info(f'Fetch request for recording id {id}')

        return response

    def fetch_sensor_names_callback(self, _, response):
        response.sensor_names_json = json.dumps(list(self.subs.keys()))
        self.get_logger().info(f'Fetch request for the sensor names')

        return response

    def fetch_sensor_data_callback(self, request, response):
        if request.sensor_name in self.subs:
            response.data_json = json.dumps(
                {'data': self.subs[request.sensor_name]['value']})
        else:
            response.data_json = ""

        self.get_logger().info(
            f'Fetch request for the sensor data of {request.sensor_name}')

        return response

    def execute_record_callback(self, goal_handle):
        self.get_logger().info(f'Requested Record for new recording with a name {
            goal_handle.request.recording_name}')

        feedback_msg = Record.Feedback()
        feedback_msg.is_busy = self.is_recording
        goal_handle.publish_feedback(feedback_msg)

        result = Record.Result()
        result.recording_id = 0
        if self.is_recording:
            goal_handle.abort()
        else:
            self.is_recording = True
            self.current_recording_id = self.next_recording_id
            self.next_recording_id = self.next_recording_id + 1
            result.recording_id = self.current_recording_id
            self.recordings[self.current_recording_id] = {
                'id': self.current_recording_id,
                'name': goal_handle.request.recording_name,
                'status': 'Recording',
                'start_time': self.get_clock().now().nanoseconds * 1e-9,
                'events': []
            }
            goal_handle.succeed()
        return result

    def execute_stop_callback(self, goal_handle):
        self.get_logger().info(f'Requested Stop for recording with an id {
            goal_handle.request.recording_id}')

        feedback_msg = Stop.Feedback()
        feedback_msg.is_authorized = self.current_recording_id == goal_handle.request.recording_id
        goal_handle.publish_feedback(feedback_msg)

        result = Stop.Result()
        result.success = feedback_msg.is_authorized
        if result.success:
            self.is_recording = False
            self.recordings[self.current_recording_id]['status'] = 'Completed'
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def generate_subscription(self, config):
        def listener_callback(msg):
            new = msg.data
            name = config['name']
            time = self.get_clock().now().nanoseconds * 1e-9
            if new != self.subs[name]['value']:
                if self.subs[name]['timeout'] > 0 and self.subs[name]['timeout'] * 1e-3 >= time - self.subs[name]['time']:
                    return
                if self.is_recording:
                    self.recordings[self.current_recording_id]['events'].append({
                        'time': time - self.recordings[self.current_recording_id]['start_time'],
                        'name': name,
                        'data': new
                    })
                self.get_logger().info(f'{time:.2f}: {name} -> {new}')
                self.subs[name]['value'] = new
                self.subs[name]['time'] = time

        self.subscription = self.create_subscription(
            config['type'], config['topic'], listener_callback, 1)

    def init_board(self):
        config_file = self.get_parameter(
            'board_config').get_parameter_value().string_value

        try:
            with open(config_file, 'r') as jsonfile:
                config = json.load(jsonfile)
            for i in range(len(config)):
                config[i]['type'] = locate(config[i]['type'].replace('/', '.'))
                assert (config[i]['type'])
        except:
            self.get_logger().error(f'Invalid config file: {config_file}')
            config = []

        self.subs = {}
        for i in range(len(config)):
            self.subs[config[i]['name']] = {
                'value': config[i]['initial'],
                'sub': self.generate_subscription(config[i]),
                'timeout': config[i]['timeout'],
                'time': - (config[i]['timeout'] + 1)
            }

    def init_recordings_db(self):
        self.recordings = {}
        self.current_recording_id = 0
        self.next_recording_id = 1
        self.is_recording = False

    def init_actions(self):
        self._record_action_server = ActionServer(
            self,
            Record,
            'record',
            self.execute_record_callback)

        self._stop_action_server = ActionServer(
            self,
            Stop,
            'stop',
            self.execute_stop_callback)

    def init_services(self):
        self._fetch_recording_srv = self.create_service(
            FetchRecording, 'fetch_recording', self.fetch_recording_callback)

        self._fetch_sensor_names_srv = self.create_service(
            FetchSensorNames, 'fetch_sensor_names', self.fetch_sensor_names_callback)

        self._fetch_sensor_data_srv = self.create_service(
            FetchSensorData, 'fetch_sensor_data', self.fetch_sensor_data_callback)


def main(args=None):
    rclpy.init(args=args)

    board_recorder = BoardRecorder()

    rclpy.spin(board_recorder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    board_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
