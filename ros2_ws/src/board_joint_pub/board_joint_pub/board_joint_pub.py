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
        
        self.angle_value = 0
        self.angle_value_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/angleValue', self.angle_value_callback, 10)

        # self.buttonA_publisher = self.create_publisher(
        #     Int32, 'button/A_status', 10)
        # self.buttonB_publisher = self.create_publisher(
        #     Int32, 'button/B_status', 10)
        # self.voltage_publisher = self.create_publisher(
        #     Int32, 'battery/voltage', 10)

        # self.buttonA = 1
        # self.buttonB = 0
        # self.voltage = 4000
        # self.i = 0

        # timer_period = 2  # seconds
        # self.timer = self.create_timer(
        #     timer_period, self.sensors_timer_callback)

    def joint_states_timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['task_board_lid_joint']
        msg.position = [(4076 - self.angle_value) / (4076 - 2800) * math.pi / 2]
        # self.get_parameter(
            # 'task_board_id').get_parameter_value().string_value
        self.task_board_lid_joint_publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def angle_value_callback(self, msg):
        self.angle_value = msg.data

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
