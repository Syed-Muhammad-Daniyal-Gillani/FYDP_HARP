import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TeleopsNode(Node):
    def __init__(self):
        super().__init__('teleops_node')

        # Subscribe to the motion_command topic
        self.subscription = self.create_subscription(
            String,
            'motion_command',
            self.listener_callback,
            10
        )
        self.get_logger().info("Teleops Node Initialized.")

    def listener_callback(self, msg):
        """Callback to handle received motion commands."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        # Validate and forward the command
        self.forward_command(command)

    def forward_command(self, command):
        """Forward the received command to the robot or Arduino."""
        # Ensure the command is in the correct format (e.g., "w 5")
        if len(command.split()) == 2:
            direction, duration = command.split()
            try:
                duration = int(duration)  # Ensure duration is an integer
                self.get_logger().info(f"Forwarding command: {direction} for {duration} seconds")
                # Add logic here to forward the command to the robot or Arduino
                # For now, just log the command
            except ValueError:
                self.get_logger().error(f"Invalid duration in command: {command}")
        else:
            self.get_logger().error(f"Invalid command format: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
