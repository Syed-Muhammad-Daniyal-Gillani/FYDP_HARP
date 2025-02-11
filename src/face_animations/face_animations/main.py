import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import os

class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation')

        self.subscription = self.create_subscription(
            String,
            'user_emotions',
            self.emotion_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Manually set the path to the source directory
        workspace_dir = "/home/darkdev/ROS_Workspaces/FYDP_HARP"

        # Construct the absolute path to the display_emotions folder inside src/
        resource_folder = os.path.join(workspace_dir, "src", "face_animations", "resource", "display_emotions")

        # Define the emotion images dictionary
        self.emotion_images = {
            "happy": os.path.join(resource_folder, "happy.png"),
            "sad": os.path.join(resource_folder, "sad.png"),
            "angry": os.path.join(resource_folder, "angry.png"),
            "surprise": os.path.join(resource_folder, "surprised.png"),
            "neutral": os.path.join(resource_folder, "neutral.png")
        }

        self.get_logger().info(f"Emotion images directory: {resource_folder}")

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
