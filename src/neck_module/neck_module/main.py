import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from neck_module.utils import RobotNeck, trackFace, pid_yaw, pid_pitch, find_esp32_port, pid_pitch_lookaround, pid_yaw_lookaround
import threading
import time

class NeckController(Node):
    def __init__(self):
        super().__init__('neck_controller')

        # Subscribe to the face tracking topic that publishes normalized face coordinates
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'neck_coordinates',
            self.listener_callback,
            1
        )

        # Publisher to broadcast detected user emotion
        self.emotion_publisher = self.create_publisher(String, 'user_emotions', 1)

        # Variables for state tracking
        self.last_emotion = None  # Track last emotion sent to avoid repetition
        self.face_detected = True  # Tracks if face was recently detected
        self.last_face_time = time.time()  # Timestamp of last face detection

        # Setup thread and event for look-around behavior
        self.lookaround_thread = threading.Thread(target=self.lookaround_loop, daemon=True)
        self.lookaround_active = False
        self.stop_lookaround = threading.Event()
        self.lock = threading.Lock()  # Thread safety

        # Initialize connection to ESP32 controlling the neck
        esp32_port = find_esp32_port()
        self.robot_neck = RobotNeck(serial_port=esp32_port, baud_rate=9600)

        # Track previous error values for PID control
        self.pre_error_x = 0
        self.pre_error_y = 0

        # Start background thread for monitoring face detection gaps
        self.lookaround_thread.start()
        self.get_logger().info("Neck Controller Node Started, listening to /neck_coordinates...")

    def listener_callback(self, msg):
        """Callback function to process received coordinates."""
        if len(msg.data) >= 2:
            # Publish 'happy' emotion if a face is detected
            if self.last_emotion != 'happy':
                self.emotion_publisher.publish(String(data='happy'))
                self.last_emotion = 'happy'

            # Update state with face detection info
            with self.lock:
                self.face_detected = True
                self.last_face_time = time.time()
                self.stop_lookaround.set()  # signal to stop lookaround if running
                self.lookaround_active = False

            # Read normalized face coordinates
            normalized_x, normalized_y = msg.data[0], msg.data[1]
            self.get_logger().info(f"Received Coordinates - X: {normalized_x}, Y: {normalized_y}")

            # Move neck to track face using PID control
            self.pre_error_x, self.pre_error_y = trackFace(
                self.robot_neck, (normalized_x, normalized_y), pid_yaw, pid_pitch, self.pre_error_x, self.pre_error_y
            )
        else:
            # If face not detected, publish 'sad' emotion
            self.get_logger().info("Face not detected")
            if self.last_emotion != 'sad':
                self.emotion_publisher.publish(String(data='sad'))
                self.last_emotion = 'sad'
            with self.lock:
                self.face_detected = False

    def lookaround_loop(self):
        """Background thread that checks if lookaround should be started."""
        while rclpy.ok():
            time.sleep(1)  # Check every second
            with self.lock:
                # If no face for more than 4 seconds, prepare to start lookaround
                time_since_last_face = time.time() - self.last_face_time
                should_lookaround = time_since_last_face > 4 and not self.lookaround_active

            if should_lookaround:
                # Publish 'sad' emotion if lookaround begins
                if self.last_emotion != 'sad':
                    self.emotion_publisher.publish(String(data='sad'))
                    self.last_emotion = 'sad'

                self.get_logger().warn("No face detected for 4+ seconds, starting look around")
                with self.lock:
                    self.lookaround_active = True
                    self.stop_lookaround.clear()
                self.lookaround()

    def yaw_pitch_to_normalized(self, yaw, pitch):
        """Reverse map yaw/pitch to normalized coordinates."""
        norm_x = (55 - yaw) / 35          # yields roughly −1 … +1
        norm_y = (pitch - 55) / 15         # yields roughly −1 … +1
        return norm_x, norm_y

    def lookaround(self):
        """Perform the look-around motion."""
        # Define a sequence of yaw/pitch positions to simulate scanning
        look_pattern = [
            (55, 55),
            (90, 60),
            (90, 45),
            (55, 50),
            (20, 50),
            (20, 50),
            (55, 55)        
        ]
        for yaw, pitch in look_pattern:
            # Exit if lookaround is stopped externally or ROS shutdown begins
            if self.stop_lookaround.is_set() or not rclpy.ok():
                break

            # Convert yaw/pitch to normalized x/y coordinates
            target_x, target_y = self.yaw_pitch_to_normalized(yaw, pitch)

            # Give 1 second to move toward each keyframe
            t_end = time.time() + 1.0
            while time.time() < t_end:
                # Move neck using PID lookaround parameters
                self.pre_error_x, self.pre_error_y = trackFace(
                    self.robot_neck,
                    (target_x, target_y),
                    pid_yaw_lookaround, pid_pitch_lookaround,
                    self.pre_error_x, self.pre_error_y
                )
                if self.stop_lookaround.is_set() or not rclpy.ok():
                    break
                time.sleep(0.05)  # Update every 50ms

        with self.lock:
            self.lookaround_active = False
        self.get_logger().info("Finished look around or interrupted by face detection.")

    def destroy(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down neck controller...")
        self.stop_lookaround.set()
        self.lookaround_thread.join(timeout=1)
        self.robot_neck.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
