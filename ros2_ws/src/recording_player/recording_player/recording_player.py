import json
from pydoc import locate

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String


class RecordingPlayer(Node):

    def __init__(self):
        super().__init__('recording_player')

        from rcl_interfaces.msg import ParameterDescriptor
        task_board_id_descriptor = ParameterDescriptor(
            description='The Task Board ID')
        self.declare_parameter('task_board_id', 'test_board_0',
                               task_board_id_descriptor)
        board_config_descriptor = ParameterDescriptor(
            description='Path to the board configuration')
        self.declare_parameter('board_config', './board_configs/default.json',
                               board_config_descriptor)
        recording_descriptor = ParameterDescriptor(
            description='Path to the recording')
        self.declare_parameter('recording', './recordings/test_recording.json',
                               recording_descriptor)
        timeout_descriptor = ParameterDescriptor(
            description='Play recording after the defined seconds')
        self.declare_parameter('timeout', 5,
                               timeout_descriptor)

        self.heartbeat_publisher = self.create_publisher(
            String, '/task_board_heartbeat', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(
            timer_period, self.heartbeat_timer_callback)

        config_file = self.get_parameter(
            'board_config').get_parameter_value().string_value
        try:
            with open(config_file, 'r') as jsonfile:
                self.board_config = json.load(jsonfile)
            for i in range(len(self.board_config)):
                self.board_config[i]['type'] = locate(
                    self.board_config[i]['type'].replace('/', '.'))
                assert (self.board_config[i]['type'])
        except:
            self.get_logger().error(f'Invalid config file: {config_file}')
            self.board_config = []

        recording_file = self.get_parameter(
            'recording').get_parameter_value().string_value
        try:
            with open(recording_file, 'r') as jsonfile:
                self.recording = json.load(jsonfile)
        except:
            self.get_logger().error(
                f'Invalid recording file: {recording_file}')
            self.recording = {'events': []}
        self.recording['event_id'] = 0

        self.timeout = self.get_parameter(
            'timeout').get_parameter_value().integer_value
        self.recording['start_time'] = self.get_clock(
        ).now().nanoseconds * 1e-9 + self.timeout

        self.pubs = {}
        board_id = self.get_parameter(
            'task_board_id').get_parameter_value().string_value
        for i in range(len(self.board_config)):
            self.pubs[self.board_config[i]['name']] = self.create_publisher(
                self.board_config[i]['type'], f'/task_board_{board_id}/{self.board_config[i]['topic']}', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            timer_period, self.sensors_timer_callback)

        self.heartbeat_timer_callback()

    def heartbeat_timer_callback(self):
        msg = String()
        msg.data = self.get_parameter(
            'task_board_id').get_parameter_value().string_value
        self.heartbeat_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def sensors_timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9 - \
            self.recording['start_time']
        msg = Int32()
        while self.recording['event_id'] < len(self.recording['events']) and \
                self.recording['events'][self.recording['event_id']]['time'] <= now:
            event = self.recording['events'][self.recording['event_id']]
            msg.data = int(event['data'])
            self.pubs[event['name']].publish(msg)

            self.recording['event_id'] = self.recording['event_id'] + 1


def main(args=None):
    rclpy.init(args=args)

    recording_player = RecordingPlayer()

    rclpy.spin(recording_player)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    recording_player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
