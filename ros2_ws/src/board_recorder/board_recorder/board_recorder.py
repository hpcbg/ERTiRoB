import sqlite3
import json
import os.path
from pydoc import locate

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from std_msgs.msg import String

from board_recorder_interfaces.srv import FetchCurrentRecordingId, FetchRecording, FetchLatestRecordings, FetchRecordingEvents, FetchSensorNames, FetchSensorData
from board_recorder_interfaces.action import Record, Stop


class BoardRecorder(Node):

    def __init__(self):
        super().__init__('board_recorder')

        from rcl_interfaces.msg import ParameterDescriptor
        board_config_descriptor = ParameterDescriptor(
            description='Path to the board configuration')
        self.declare_parameter('board_config', './board_configs/default.json',
                               board_config_descriptor)

        self.init_boards_monitor()
        self.init_recordings_db()
        self.init_actions()
        self.init_services()

    def fetch_latest_recordings_callback(self, request, response):
        count = request.count
        recordings = []
        cur = self.db_con.cursor()
        rows = cur.execute(
            f'SELECT * FROM recordings ORDER BY start_time DESC{'' if count == 0 else f' LIMIT {count}'}')
        row = rows.fetchone()
        while row:
            recordings.append({
                'id': row[0],
                'name': row[1],
                'start_time': row[2],
                'status': row[3]
            })
            row = rows.fetchone()

        response.recordings_list_json = json.dumps(recordings)

        self.get_logger().info(
            f'Fetch request for latest recordings with count {count}')

        return response

    def fetch_recording_callback(self, request, response):
        id = request.recording_id

        cur = self.db_con.cursor()
        row = cur.execute(
            f'SELECT * FROM recordings WHERE id = \'{id}\' LIMIT 1')
        row = row.fetchone()
        if row:
            recording = {
                'id': id,
                'name': row[1],
                'start_time': row[2],
                'status': row[3],
                'events': []
            }
            events = cur.execute(
                f'SELECT time, name, data FROM events WHERE recording_id = \'{id}\' ORDER BY time ASC')
            event = events.fetchone()
            while event:
                recording['events'].append({
                    'time': event[0],
                    'name': event[1],
                    'data': event[2]
                })
                event = events.fetchone()
        else:
            recording = {
                'id': id,
                'name': 'unknown',
                'start_time': 0,
                'status': 'Not Found',
                'events': []
            }

        response.recording_json = json.dumps(recording)

        self.get_logger().info(f'Fetch request for recording id {id}')

        return response

    def fetch_recording_events_callback(self, request, response):
        events = []
        cur = self.db_con.cursor()
        rows = cur.execute(
            f'SELECT time, name, data FROM events WHERE recording_id = {request.recording_id} AND time >= {request.from_time} ORDER BY time ASC')
        row = rows.fetchone()
        while row:
            events.append({
                'time': row[0],
                'name': row[1],
                'data': row[2]
            })
            row = rows.fetchone()

        response.events_json = json.dumps(events)

        self.get_logger().info(
            f'Fetch request for events from recording with id {request.recording_id} from time {request.from_time}')

        return response

    def fetch_current_recording_id_callback(self, _, response):
        response.recording_id = self.current_recording_id if self.is_recording else -1

        self.get_logger().info(f'Fetch request for current recording id')

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
        board_id = goal_handle.request.task_board_id
        protocol = goal_handle.request.protocol

        self.get_logger().info(
            f'Requested Record for new recording on board {board_id} for protocol {protocol}')

        if not board_id in self.boards:
            feedback_msg = Record.Feedback()
            feedback_msg.is_busy = True
            goal_handle.publish_feedback(feedback_msg)

            result = Record.Result()
            result.recording_id = 0
            goal_handle.abort()
            return result

        feedback_msg = Record.Feedback()
        feedback_msg.is_busy = self.boards[board_id]['is_recording']
        goal_handle.publish_feedback(feedback_msg)

        result = Record.Result()
        result.recording_id = 0
        if self.boards[board_id]['is_recording']:
            result.recording_id = self.boards[board_id]['current_recording_id']
            goal_handle.succeed()
        else:
            self.boards[board_id]['current_recording_start_time'] = self.get_clock(
            ).now().nanoseconds * 1e-9
            recording_id = int(
                self.boards[board_id]['current_recording_start_time'])
            self.boards[board_id]['current_recording_id'] = recording_id
            self.boards[board_id]['is_recording'] = True
            recording_id
            cur = self.db_con.cursor()
            cur.execute(f'INSERT INTO recordings (task_board_id, protocol, id, start_time, status) VALUES \
                        (?, ?, {recording_id}, {self.boards[board_id]['current_recording_start_time']}, \
                        \'Recording\')', [board_id, protocol])
            self.db_con.commit()
            result.recording_id = recording_id
            goal_handle.succeed()
        return result

    def execute_stop_callback(self, goal_handle):
        board_id = goal_handle.request.task_board_id
        recording_id = goal_handle.request.recording_id

        self.get_logger().info(
            f'Requested Stop for recording with an id {recording_id} on task board {board_id}')

        feedback_msg = Stop.Feedback()
        feedback_msg.is_authorized = True
        goal_handle.publish_feedback(feedback_msg)

        result = Stop.Result()
        result.success = feedback_msg.is_authorized
        cur = self.db_con.cursor()
        row = cur.execute(
            'SELECT id, status FROM recordings WHERE task_board_id = ? AND id = ?',
            [board_id, recording_id])
        row = cur.fetchone()
        if row:
            cur.execute(
                'UPDATE recordings SET status = \'Completed\' WHERE task_board_id = ? AND id = ?',
                [board_id, recording_id])
            self.db_con.commit()
            self.is_recording = False
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def init_board(self, board_id):
        def generate_subscription(config):
            def listener_callback(msg):
                new = msg.data
                name = config['name']
                time = self.get_clock().now().nanoseconds * 1e-9
                if new != self.boards[board_id]['subs'][name]['value']:
                    if self.boards[board_id]['subs'][name]['timeout'] > 0 and \
                        self.boards[board_id]['subs'][name]['timeout'] * 1e-3 >= \
                            time - self.boards[board_id]['subs'][name]['time']:
                        return
                    if self.boards[board_id]['is_recording']:
                        cur = self.db_con.cursor()
                        cur.execute(
                            f'INSERT INTO events (task_board_id, recording_id, name, data, time) VALUES \
                              (\'{board_id}\', \
                               {self.boards[board_id]['current_recording_id']}, \
                               ?, \
                               ?, \
                               {time - self.boards[board_id]['current_recording_start_time']})',
                            [name, new])
                        self.db_con.commit()
                    self.get_logger().info(
                        f'{time:.2f}: board id {board_id} - {name} -> {new}')
                    self.boards[board_id]['subs'][name]['value'] = new
                    self.boards[board_id]['subs'][name]['time'] = time

            self.boards[board_id]['subs']['subscription'] = self.create_subscription(
                config['type'], f'/task_board_{board_id}/{config['topic']}', listener_callback, 1)

        self.boards[board_id] = {
            'is_recording': False,
            'current_recording_id': 0,
            'last_active': 0,
            'subs': {}
        }
        for i in range(len(self.board_config)):
            self.boards[board_id]['subs'][self.board_config[i]['name']] = {
                'value': self.board_config[i]['initial'],
                'sub': generate_subscription(self.board_config[i]),
                'timeout': self.board_config[i]['timeout'],
                'time': - (self.board_config[i]['timeout'] + 1)
            }

    def init_boards_monitor(self):
        def callback(msg):
            if not msg.data in self.boards:
                self.get_logger().info(f'New board found: -> {msg.data}')
                self.init_board(msg.data)
            self.boards[msg.data]['last_active'] = self.get_clock(
            ).now().nanoseconds * 1e-9

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

        self.boards = {}
        self.heartbeat_sub = self.create_subscription(
            String, '/task_board_heartbeat', callback, 10)

    def init_recordings_db(self):
        DB_FILE = './db/recordings.db'
        if os.path.isfile(DB_FILE):
            self.db_con = sqlite3.connect(DB_FILE)
        else:
            self.db_con = sqlite3.connect(DB_FILE)
            cur = self.db_con.cursor()
            cur.execute(
                'CREATE TABLE recordings(task_board_id VARCHAR(16), \
                                         id            INTEGER, \
                                         protocol      VARCHAR(256), \
                                         start_time    DOUBLE, \
                                         status        VARCHAR(16))')
            cur.execute(
                'CREATE INDEX recordings_task_board_id_idx ON recordings(task_board_id)')
            cur.execute(
                'CREATE INDEX recordings_id_idx ON recordings(id)')
            cur.execute(
                'CREATE INDEX recordings_protocol_idx ON recordings(protocol)')
            cur.execute(
                'CREATE TABLE events(task_board_id VARCHAR(16), \
                                     recording_id  INTEGER, \
                                     name          VARCHAR(32), \
                                     data          TEXT, \
                                     time          DOUBLE)')
            cur.execute(
                'CREATE INDEX events_task_board_id_idx ON events(task_board_id)')
            cur.execute(
                'CREATE INDEX events_recording_id_idx ON events(recording_id)')

    def init_actions(self):
        self._record_action_server = ActionServer(
            self,
            Record,
            '/record',
            self.execute_record_callback)

        self._stop_action_server = ActionServer(
            self,
            Stop,
            '/stop',
            self.execute_stop_callback)

    def init_services(self):
        self._fetch_current_recording_id_srv = self.create_service(
            FetchCurrentRecordingId, 'fetch_current_recording_id', self.fetch_current_recording_id_callback)

        self._fetch_latest_recordings_srv = self.create_service(
            FetchLatestRecordings, 'fetch_latest_recordings', self.fetch_latest_recordings_callback)

        self._fetch_recording_srv = self.create_service(
            FetchRecording, 'fetch_recording', self.fetch_recording_callback)

        self._fetch_recording_events_srv = self.create_service(
            FetchRecordingEvents, 'fetch_recording_events', self.fetch_recording_events_callback)

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
