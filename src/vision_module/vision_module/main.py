import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from vision_module.utils import *
import cv2

# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initialize camera and neck
initialize_camera(0)  # Change to 1 for external camera
cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)  # Capture video frame

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'neck_coordinates', 10)
        self.timer = self.create_timer(0.1, self.track_face)  # Runs every 0.1 seconds (10Hz)

    def track_face(self):
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        img, (normalized_x, normalized_y), face_area = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)

        if face_area > 0:
            msg = Float32MultiArray()
            msg.data = [normalized_x, normalized_y]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: X={normalized_x}, Y={normalized_y}")

        # Display the video frame with detections
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

# try:
#     while True:

#         # Detect face and retrieve normalized coordinates
#         img, (normalized_x, normalized_y), face_area = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)

#         # Track face if detected
#         if face_area > 0:
#             print(f"Normalized X: {normalized_x}, Normalized Y: {normalized_y}")
#         else:
#             print("No face detected.")

#         # Display the video frame with detections
#         cv2.imshow("StereoCam Input", img)

#         # Kill switch (press 'q' to quit)
#         k = cv2.waitKey(1)
#         if k == ord('q'):
#             break

# finally:
#     release_camera()