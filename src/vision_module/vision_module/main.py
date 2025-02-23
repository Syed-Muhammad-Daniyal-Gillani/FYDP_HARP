import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from vision_module.utils import *
import cv2

# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initialize camera
initialize_camera(0)  # Change to 1 for external camera

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'neck_coordinates', 1)
        self.emotion_publisher = self.create_publisher(String, 'user_emotions', 1)
        self.timer = self.create_timer(0.03, self.track_face)  # Runs every 0.03 seconds (~33Hz)
        self.last_emotion = None  # Store last detected emotion

    def track_face(self):
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        img, (normalized_x, normalized_y), face_area, face_roi = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)

        if face_area > 0:
            # Publish neck coordinates
            msg = Float32MultiArray()
            msg.data = [normalized_x, normalized_y]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: X={normalized_x}, Y={normalized_y}")

            # Detect and publish emotion only if it has changed
            if face_roi is not None:
                emotion = detect_emotion(face_roi)
                if emotion != self.last_emotion:
                    self.last_emotion = emotion  # Update last detected emotion
                    emotion_msg = String()
                    emotion_msg.data = emotion
                    self.emotion_publisher.publish(emotion_msg)
                    self.get_logger().info(f"Published Emotion: {emotion}")

        # Display the video frame
        cv2.imshow("Face Tracker", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down vision module...")
            release_camera()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
