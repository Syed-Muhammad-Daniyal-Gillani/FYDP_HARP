import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MqttNode(Node):
    def __init__(self):
        super().__init__('mqtt_node')
        self.subscription = self.create_subscription(
            String,
            'motion_command',
            self.listener_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Adjust port and baud rate
        self.get_logger().info("MQTT Node Initialized.")

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        self.send_to_arduino(command)

    def send_to_arduino(self, command):
        try:
            # Ensure the command is in the correct format (e.g., "w 5")
            if len(command.split()) == 2:
                self.serial_port.write((command + '\n').encode())  # Add newline for Arduino parsing
                self.get_logger().info(f"Sent to Arduino: {command}")
            else:
                self.get_logger().error(f"Invalid command format: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send to Arduino: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MqttNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
