import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


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

        self.probe_start_state = 1
        self.probe_start_state_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/probeStartState', self.probe_start_state_callback, 10)

        self.probe_goal_state = -1
        self.probe_goal_state_sub = self.create_subscription(
            Int32, f'/task_board_{task_board_id}/probeGoalState', self.probe_goal_state_callback, 10)

        self.probe_in_holder = {
            'x': -0.101844, 'y': -0.007, 'z': 0.019,
            'rx': 0.0, 'ry': 0.0, 'rz': 0.0
        }
        self.probe_in_air = {
            'x': -0.1, 'y': +0.07, 'z': 0.05,
            'rx': 0.2, 'ry': 0.0, 'rz': math.pi/2
        }
        self.probe_in_terminal = {
            'x': -0.05, 'y': 0.0, 'z': 0.03,
            'rx': math.pi/2, 'ry': 0.0, 'rz': math.pi/2
        }
        self.probe = self.probe_in_holder.copy()
        self.cable_plug = {'x': 0.03844299912452698, 'y': 0.04390699937939644,  # 'y': 0.01890699937939644,
                           'z': 0.010, 'rx': 0.0, 'ry': 0.0, 'rz': -1.5707963705062866}
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_states_timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['task_board_lid_joint', 'task_board_slider_joint',
                    'task_board_button_blue_joint', 'task_board_button_red_joint']
        msg.position = [self.lid_angle, self.slider_position,
                        self.blue_button, self.red_button]

        self.task_board_lid_joint_publisher.publish(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'task_board'
        t.child_frame_id = 'task_board_probe'
        t.transform.translation.x = self.probe['x']
        t.transform.translation.y = self.probe['y']
        t.transform.translation.z = self.probe['z']
        q = self.quaternion_from_euler(
            self.probe['rx'], self.probe['ry'], self.probe['rz'])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'task_board'
        t.child_frame_id = 'task_board_cable_plug'
        t.transform.translation.x = self.cable_plug['x']
        t.transform.translation.y = self.cable_plug['y'] + (
            0 if self.probe_goal_state == -1 else -0.025)
        t.transform.translation.z = self.cable_plug['z']
        q = self.quaternion_from_euler(
            self.cable_plug['rx'], self.cable_plug['ry'], self.cable_plug['rz'])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def angle_value_callback(self, msg):
        new_lid_angle = (4076 - msg.data) / (4076 - 2800) * math.pi / 2
        if self.probe_goal_state == 1 and abs(new_lid_angle - self.lid_angle) > 0.01:
            # move probe
            self.probe['x'] = -0.118
            self.probe['y'] = - 0.032 + math.cos(new_lid_angle) * 0.047
            self.probe['z'] = 0.0045 + math.sin(new_lid_angle) * 0.047
            self.probe['rx'] = 0.2
            self.probe['rz'] = math.pi / 2

        self.lid_angle = new_lid_angle - 0.08

    def fader_value_callback(self, msg):
        self.slider_position = (msg.data - 2048) / 2048 * 0.018

    def button_push_state_callback(self, msg):
        self.blue_button = 0 if msg.data > 0 else -0.003

    def stop_btn_state_callback(self, msg):
        self.red_button = 0 if msg.data > 0 else -0.003

    def probe_start_state_callback(self, msg):
        if msg.data != self.probe_start_state:
            if msg.data == 0:
                self.probe = self.probe_in_terminal.copy()
            else:
                self.probe = self.probe_in_air.copy()
        self.probe_start_state = msg.data

    def probe_goal_state_callback(self, msg):
        if msg.data != self.probe_goal_state:
            if msg.data == 0:
                self.probe = self.probe_in_holder.copy()
            else:
                self.probe = self.probe_in_air.copy()
        self.probe_goal_state = msg.data

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


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
