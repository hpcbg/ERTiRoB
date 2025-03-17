import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState


class BoardJointPub(Node):

    def __init__(self):
        super().__init__('board_joint_pub')

        from rcl_interfaces.msg import ParameterDescriptor
        task_board_id_descriptor = ParameterDescriptor(
            description='The Task Board ID')
        self.declare_parameter('task_board_id', 'test_board_0',
                               task_board_id_descriptor)

        self.task_board_lid_joint_publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            timer_period, self.joint_states_timer_callback)

        task_board_id = self.get_parameter(
            'task_board_id').get_parameter_value().string_value

        self.lid_angle = 0.0
        self.angle_value_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/angleValue', self.angle_value_callback, 10)

        self.slider_position = -0.018
        self.fader_value_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/faderValue', self.fader_value_callback, 10)

        self.red_button = 0
        self.stop_btn_state_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/stopBtnState', self.stop_btn_state_callback, 10)

        self.blue_button = 0
        self.button_push_state_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/buttonPushState', self.button_push_state_callback, 10)

    def joint_states_timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['task_board_lid_joint', 'task_board_slider_joint',
                    'task_board_button_blue_joint', 'task_board_button_red_joint']
        msg.position = [self.lid_angle, self.slider_position,
                        self.blue_button, self.red_button]

        self.task_board_lid_joint_publisher.publish(msg)

    def angle_value_callback(self, msg):
        self.lid_angle = (4076 - msg.data) / (4076 - 2800) * math.pi / 2

    def fader_value_callback(self, msg):
        self.slider_position = (msg.data - 2048) / 2048 * 0.018

    def button_push_state_callback(self, msg):
        self.blue_button = 0 if msg.data > 0 else -0.003

    def stop_btn_state_callback(self, msg):
        self.red_button = 0 if msg.data > 0 else -0.003


def main(args=None):
    rclpy.init(args=args)

    board_joint_pub = BoardJointPub()

    rclpy.spin(board_joint_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    board_joint_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
