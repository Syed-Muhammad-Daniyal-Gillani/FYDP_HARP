import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from vision_module.utils import *
from vision_module.behavior_utils import *
import cv2
# Frame resolution
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initialize camera
initialize_camera()  # Change to 1 for external camera

class HARP_Vision(Node):
    def __init__(self):
        super().__init__('harp_vision')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'neck_coordinates', 1)
        self.behavior_pub = self.create_publisher(String, 'user_behavior', 1)
        self.timer = self.create_timer(0.03, self.track_face)
        self.behavior_timer = self.create_timer(0.2, self.publish_behavior)
        self.srv = self.create_service(Trigger, 'trigger_emotion_request', self.emotion_request_handle)

    def track_face(self):
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        if cam_img is None:
            return
        img, (normalized_x, normalized_y), face_area, face_roi = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)

        if face_area > 0:
            # Publish neck coordinates
            msg = Float32MultiArray()
            msg.data = [normalized_x, normalized_y]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: X={normalized_x}, Y={normalized_y}")
    
        cv2.imshow("Face Tracker", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down vision module...")
            release_camera()
            rclpy.shutdown()
            
    def publish_behavior(self):
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        if cam_img is None:
            return 
        label, score = run(cam_img)
        self.get_logger().info(f"Behavior detected: {label} with score {score}")
        if score > 0.5:
            msg = String()
            msg.data = label
            self.behavior_pub.publish(msg)
            self.get_logger().info(f"Published behavior: {label} ({score:.2f})")


    #Emotion request handle to detect emotion when requested
    def emotion_request_handle(self, request, response): 
        cam_img = getVideo(FRAME_WIDTH, FRAME_HEIGHT)
        img, (normalized_x, normalized_y), face_area, face_roi = detect_face(cam_img, FRAME_WIDTH, FRAME_HEIGHT)
        # Detect and publish emotion only if it has changed
        if face_roi is not None:                        #Check if face is being detected
            emotion = detect_emotion(face_roi)          #Call the deepFace function from utils
            emotion_msg = String()                      #Declare string variable
            emotion_msg.data = emotion                  #Store detected emotion to string
            response.success = True
            response.message = emotion_msg.data         #Send detected emotion as response to client
        return response

        # Display the video frame

def main(args=None):
    rclpy.init(args=args)
    node = HARP_Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
