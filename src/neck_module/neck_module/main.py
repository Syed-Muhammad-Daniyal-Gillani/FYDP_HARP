import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from neck_module.utils import RobotNeck, trackFace, pid_yaw, pid_pitch, find_esp32_port
import threading
import time

class NeckController(Node):
    def __init__(self):
        super().__init__('neck_controller')

        # Subscribe to the face tracking topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'neck_coordinates',
            self.listener_callback,
            1
        )
        self.emotion_publisher = self.create_publisher(String, 'user_emotions', 1)
        self.last_emotion = None  # Track last emotion sent
        self.face_detected = True
        self.last_face_time = time.time()
        self.lookaround_thread = threading.Thread(target=self.lookaround_loop, daemon=True)
        self.lookaround_active = False
        self.stop_lookaround = threading.Event()
        self.lock = threading.Lock()

        esp32_port = find_esp32_port()
        self.robot_neck = RobotNeck(serial_port=esp32_port, baud_rate=9600)
        self.pre_error_x = 0
        self.pre_error_y = 0

        # Start background thread
        self.lookaround_thread.start()
        self.get_logger().info("Neck Controller Node Started, listening to /neck_coordinates...")

    def listener_callback(self, msg):
        """Callback function to process received coordinates."""
        if len(msg.data) >= 2:
            if self.last_emotion != 'happy':
                self.emotion_publisher.publish(String(data='happy'))
                self.last_emotion = 'happy'

            with self.lock:
                self.face_detected = True
                self.last_face_time = time.time()
                self.stop_lookaround.set()  # signal to stop lookaround if running
                self.lookaround_active = False

            normalized_x, normalized_y = msg.data[0], msg.data[1]
            self.get_logger().info(f"Received Coordinates - X: {normalized_x}, Y: {normalized_y}")

            # Track the face using the received coordinates
            self.pre_error_x, self.pre_error_y = trackFace(
                self.robot_neck, (normalized_x, normalized_y), pid_yaw, pid_pitch, self.pre_error_x, self.pre_error_y
            )
        else:
            self.get_logger().info("Face not detected")
            if self.last_emotion != 'sad':
                self.emotion_publisher.publish(String(data='sad'))
                self.last_emotion = 'sad'
            with self.lock:
                self.face_detected = False

    def lookaround_loop(self):
        """Background thread that checks if lookaround should be started."""
        while rclpy.ok():
            time.sleep(1)
            with self.lock:
                time_since_last_face = time.time() - self.last_face_time
                should_lookaround = time_since_last_face > 4 and not self.lookaround_active

            if should_lookaround:
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
        look_pattern = [
            (55, 55),
            (65, 58)
            # (75, 60),
            # (85, 58),
            # (90, 55),
            # (70, 55),
            # (55, 55),
            # (35, 52),
            # (30, 48),          
            # (25, 45),
            # (35, 45),
            # (45, 50),
            # (55, 50)  # Return to original position            
        ]
        for yaw, pitch in look_pattern:
            if self.stop_lookaround.is_set() or not rclpy.ok():
                break

            # Convert yaw/pitch to normalized x/y
            target_x, target_y = self.yaw_pitch_to_normalized(yaw, pitch)

            t_end = time.time() + 1.0  # 1 s to reach each keyframe
            while time.time() < t_end:
                self.pre_error_x, self.pre_error_y = trackFace(
                    self.robot_neck,
                    (target_x, target_y),
                    pid_yaw, pid_pitch,
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
