import math
import rclpy
from std_msgs.msg import Int32, String


class TaskBoardDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.lid_joint_sensor = self.__robot.getDevice(
            'task_board_lid_joint_sensor')
        self.lid_joint_sensor.enable(10)
        self.holder_distance_sensor = self.__robot.getDevice(
            'holder_distance_sensor')
        self.holder_distance_sensor.enable(10)

        rclpy.init(args=None)
        self.node = rclpy.create_node('task_board_driver')

        self.angleValue = Int32()
        self.angleValue.data = 0
        self.angle_value_publisher = self.node.create_publisher(
            Int32, '/task_board_simulated/angleValue', 1)

        self.probeStartState = Int32()
        self.probeStartState.data = 1
        self.probe_start_state_publisher = self.node.create_publisher(
            Int32, '/task_board_simulated/probeStartState', 1)

        self.probeGoalState = Int32()
        self.probeGoalState.data = 1
        self.probe_goal_state_publisher = self.node.create_publisher(
            Int32, '/task_board_simulated/probeGoalState', 1)

        self.heartbeat_publisher = self.node.create_publisher(
            String, '/task_board_heartbeat', 10)
        timer_period = 10  # seconds
        self.heartbeat_timer = self.node.create_timer(
            timer_period, self.heartbeat_timer_callback)

        timer_period = 0.1  # seconds
        self.sensors_timer = self.node.create_timer(
            timer_period, self.sensors_timer_callback)

        self.heartbeat_timer_callback()

    def heartbeat_timer_callback(self):
        msg = String()
        msg.data = 'simulated'
        self.heartbeat_publisher.publish(msg)

    def sensors_timer_callback(self):
        try:
            self.angleValue.data = int(
                4030 - self.lid_joint_sensor.getValue() * (4030 - 2770) * 2 / math.pi)
        except:
            pass

        try:
            if self.holder_distance_sensor.getValue() > 20:
                self.probeGoalState.data = 1
            else:
                self.probeGoalState.data = 0
        except:
            pass

        self.angle_value_publisher.publish(self.angleValue)
        self.probe_start_state_publisher.publish(self.probeStartState)
        self.probe_goal_state_publisher.publish(self.probeGoalState)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
