import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import os

from ament_index_python.packages import get_package_share_directory

class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation')

        # Get the installed package path
        package_share_path = get_package_share_directory('face_animations')
        emotions_path = os.path.join(package_share_path, 'emotions')

        self.subscription = self.create_subscription(
            String,
            'user_emotions',
            self.emotion_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.emotion_images = {
            "happy": os.path.join(emotions_path, "happy.png"),
            "sad": os.path.join(emotions_path, "sad.png"),
            "angry": os.path.join(emotions_path, "angry.png"),
            "surprised": os.path.join(emotions_path, "surprised.png"),
            "neutral": os.path.join(emotions_path, "neutral.png")
        }

    def emotion_callback(self, msg):
        emotion = msg.data.lower()
        if emotion in self.emotion_images:
            image_path = self.emotion_images[emotion]
            self.display_image(image_path)
        else:
            self.get_logger().warn(f"Emotion '{emotion}' not recognized!")

    def display_image(self, image_path):
        img = cv2.imread(image_path)
        if img is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return
        cv2.imshow("Emotion Display", img)
        cv2.waitKey(1)  # Required to keep the window open

def main(args=None):
    rclpy.init(args=args)
    node = FaceAnimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
