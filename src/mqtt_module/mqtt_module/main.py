import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MqttNode(Node):
    def __init__(self):
        super().__init__('mqtt_node')

        # ROS 2 subscription to motion_command topic
        self.subscription = self.create_subscription(
            String,
            'motion_command',
            self.listener_callback,
            10
        )

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the MQTT broker
        broker_address = "localhost"
        broker_port = 1883
        try:
            self.mqtt_client.connect(broker_address, broker_port, 60)
            self.get_logger().info(f"Connected to MQTT broker at {broker_address}:{broker_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

        # Start the MQTT client loop in a separate thread
        self.mqtt_client.loop_start()

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command from ROS 2 topic: {command}")
        self.mqtt_client.publish("robot/motion_command", command)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT client connected successfully.")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        self.get_logger().info(f"Received message from MQTT topic '{msg.topic}': {message}")

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()