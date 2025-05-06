import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from vision_module.utils import *
from vision_module.behavior_utils import *
import cv2
import os
from ament_index_python.packages import get_package_share_directory

# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Get paths for model and labels
resource_path = os.path.join(get_package_share_directory('vision_module'), 'resource')
model_path = os.path.join(resource_path, '2.tflite')
label_path = os.path.join(resource_path, 'kinetics600_label_map.txt')

# Initialize camera
initialize_camera()  # Change to 1 for external camera

class HARP_Vision(Node):
    def __init__(self):
        super().__init__('harp_vision')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'neck_coordinates', 1)
        self.behavior_publisher_ = self.create_publisher(String, 'behavior_detection', 1)
        self.timer = self.create_timer(0.03, self.track_face)  # Runs every 0.03 seconds (~33Hz)
        self.srv = self.create_service(Trigger, 'trigger_emotion_request', self.emotion_request_handle)
        
        # Initialize behavior classification
        self.last_behavior_time = time.time()
        self.current_categories = []
        self.time_per_infer = 0.0
        
    def track_face(self):
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        img, (normalized_x, normalized_y), face_area, face_roi = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)
        
        if face_area > 0:
            # Publish neck coordinates
            msg = Float32MultiArray()
            msg.data = [normalized_x, normalized_y]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: X={normalized_x}, Y={normalized_y}")
        
        # Perform behavior classification every ~1/MODEL_FPS seconds
        now = time.time()
        if (now - self.last_behavior_time) * MODEL_FPS >= (1 - MODEL_FPS_ERROR_RANGE):
            self.last_behavior_time = now
            # Classify behavior
            rgb = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            self.current_categories, self.time_per_infer = classify_single_frame(cam_img)
            
            # Reset internal state if no confident prediction
            if all(category.score < 0.5 for category in self.current_categories):
                classifier.clear()
            else:
                # Publish detected behavior
                for cat in self.current_categories:
                    behavior_msg = String()
                    behavior_msg.data = cat.label
                    self.behavior_publisher_.publish(behavior_msg)
                    self.get_logger().info(f"Behavior detected: {cat.label} ({cat.score:.2f})")
        
        # Display behavior info on frame
        cv2.putText(img, f"FPS: {1.0/((now - self.last_behavior_time) + 1e-8):.1f}", 
                   (LEFT_MARGIN, ROW_SIZE), cv2.FONT_HERSHEY_PLAIN, 
                   FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
                   
        cv2.putText(img, f"Infer: {int(self.time_per_infer * 1000)}ms",
                   (LEFT_MARGIN, 2 * ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                   FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
                   
        for idx, cat in enumerate(self.current_categories):
            cv2.putText(img, f"{cat.label} ({cat.score:.2f})",
                       (LEFT_MARGIN, (idx + 3) * ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                       FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
    
        cv2.imshow("HARP Vision", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down vision module...")
            release_camera()
            rclpy.shutdown()
            
    # Emotion request handle to detect emotion when requested
    def emotion_request_handle(self, request, response): 
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        img, (normalized_x, normalized_y), face_area, face_roi = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)
        
        # Detect and publish emotion only if it has changed
        if face_roi is not None:                        # Check if face is being detected
            emotion = detect_emotion(face_roi)          # Call the deepFace function from utils
            emotion_msg = String()                      # Declare string variable
            emotion_msg.data = emotion                  # Store detected emotion to string
            response.success = True
            response.message = emotion_msg.data         # Send detected emotion as response to client
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HARP_Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()