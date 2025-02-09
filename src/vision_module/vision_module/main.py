import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from utils import *
import cv2

# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initialize camera and neck
initialize_camera(1)  # Change to 1 for external camera

try:
    while True:
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)  # Capture video frame

        # Detect face and retrieve normalized coordinates
        img, (normalized_x, normalized_y), face_area = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)

        # Track face if detected
        if face_area > 0:
            print(f"Normalized X: {normalized_x}, Normalized Y: {normalized_y}")
        else:
            print("No face detected.")

        # Display the video frame with detections
        cv2.imshow("StereoCam Input", img)

        # Kill switch (press 'q' to quit)
        k = cv2.waitKey(1)
        if k == ord('q'):
            break

finally:
    release_camera()