import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String


class TestTaskBoardSimulator(Node):

    def __init__(self):
        super().__init__('test_task_board')

        from rcl_interfaces.msg import ParameterDescriptor
        task_board_id_descriptor = ParameterDescriptor(
            description='The Task Board ID')
        self.declare_parameter('task_board_id', 'test_board_0',
                               task_board_id_descriptor)

        self.heartbeat_publisher = self.create_publisher(
            String, '/task_board_heartbeat', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(
            timer_period, self.heartbeat_timer_callback)

        self.buttonA_publisher = self.create_publisher(
            Int32, 'button/A_status', 10)
        self.buttonB_publisher = self.create_publisher(
            Int32, 'button/B_status', 10)
        self.voltage_publisher = self.create_publisher(
            Int32, 'battery/voltage', 10)

        self.buttonA = 1
        self.buttonB = 0
        self.voltage = 4000
        self.i = 0

        timer_period = 2  # seconds
        self.timer = self.create_timer(
            timer_period, self.sensors_timer_callback)

    def heartbeat_timer_callback(self):
        msg = String()
        msg.data = self.get_parameter(
            'task_board_id').get_parameter_value().string_value
        self.heartbeat_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def sensors_timer_callback(self):
        msg = Int32()
        if self.i % 3 == 0:
            msg.data = self.buttonA
            self.buttonA_publisher.publish(msg)
            self.buttonA = 1 - self.buttonA
        if self.i % 5 == 0:
            msg.data = self.buttonB
            self.buttonB_publisher.publish(msg)
            self.buttonB = 1 - self.buttonB
        msg.data = self.voltage
        self.voltage_publisher.publish(msg)
        self.voltage = self.voltage - 10
        if self.voltage < 3600:
            self.voltage = 4000
        self.i = (self.i + 1) % 1024


def main(args=None):
    rclpy.init(args=args)

    test_task_board_simulator = TestTaskBoardSimulator()

    rclpy.spin(test_task_board_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_task_board_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
