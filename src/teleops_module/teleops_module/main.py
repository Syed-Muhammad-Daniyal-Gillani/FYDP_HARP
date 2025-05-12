import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
from teleops_module.utils import find_arduino_port


class TeleopsNode(Node):
    def __init__(self):
        super().__init__('teleops_node')

        self.subscription = self.create_subscription(
            String,
            'motion_command',
            self.listener_callback,
            10
        )

        try:
            arduino_port = find_arduino_port()
            self.arduino = serial.Serial(arduino_port, baudrate=9600, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to Arduino on {arduino_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        if self.arduino:
            self.arduino.write(command.encode())

    def destroy_node(self):
        if self.arduino:
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()