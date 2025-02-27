import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import webbrowser
import os

package_path = os.path.expanduser("~/ROS_FYDP/src/face_animations/face_animations")


class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation_node')
        self.subscription = self.create_subscription(
            String,
            '/user_emotions',
            self.emotion_callback,
            10)
        self.current_emotion = "neutral"

        # Open the browser with local HTML file
        webbrowser.open(f"file://{package_path}/index.html")

    def emotion_callback(self, msg):
        emotion = msg.data.lower().strip()
        valid_emotions = ["happy", "sad", "angry", "focused", "surprise", "neutral"]

        if emotion in valid_emotions and emotion != self.current_emotion:
            self.current_emotion = emotion
            self.get_logger().info(f"Received emotion: {emotion}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceAnimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
