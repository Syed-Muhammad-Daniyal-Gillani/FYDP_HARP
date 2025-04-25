import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl  # Import QUrl

package_path = os.path.expanduser("~/fydp_harp/src/face_animations/face_animations")
file_url = f"file://{package_path}/index.html"

class FaceAnimationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl(file_url))  # Convert string to QUrl
        self.setCentralWidget(self.browser)
        self.showFullScreen()  # Open in fullscreen

class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation_node')
        self.subscription = self.create_subscription(
            String,
            '/user_emotions',
            self.emotion_callback,
            10)
        self.current_emotion = "neutral"
        
        # Launch PyQt window
        self.app = QApplication(sys.argv)
        self.window = FaceAnimationWindow()
        self.app.exec_()

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
