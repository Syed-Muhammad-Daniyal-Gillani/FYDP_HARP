import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from neck_module.utils import RobotNeck, trackFace, pid, find_esp32_port, lookAround
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
        self.face_detected = False
        self.lookaround_active = False
        sleep_time = 12
        self.la_timer = self.create_timer(sleep_time, self.lookAround_timer)
        esp32_port = find_esp32_port()
        
        # Initialize neck movement
        self.robot_neck = RobotNeck(serial_port=esp32_port, baud_rate=9600)  # Update port if needed
        self.pre_error_x = 0
        self.pre_error_y = 0
        self.count = 0

        self.get_logger().info("Neck Controller Node Started, listening to /neck_coordinates...")
    def lookAround_timer(self):
        if self.lookaround_active:
            self.get_logger().info("Lookaround is true, returning timer function")
            return
        if self.face_detected:
            self.get_logger().info("Face detected. Resetting count")
            self.count = 0
            self.face_detected = False 
            return
        else:
            self.count+= 1
            self.get_logger().warn(f"Initiating look around in {4 - self.count} seconds")
            if self.count == 4:
                self.get_logger().warn(f"Count reached {self.count}, Initiating look around")
                self.count = 0
                self.get_logger().warn(f"Count reset. Value is: {self.count}")
                self.lookaround_active = True
                return

    def listener_callback(self, msg):
        """Callback function to process received coordinates."""
        if len(msg.data) >= 2:
            self.face_detected = True
            self.lookaround_active = False
            normalized_x, normalized_y = msg.data[0], msg.data[1]
            self.get_logger().info(f"Received Coordinates - X: {normalized_x}, Y: {normalized_y}")
            
            # Track the face using the received coordinates
            self.pre_error_x, self.pre_error_y = trackFace(
                self.robot_neck, (normalized_x, normalized_y), pid, self.pre_error_x, self.pre_error_y
            )
        else:
            self.get_logger().info(f"Face is not detected")
            self.face_detected = False

    def lookaround(self):
        if self.lookaround_active:
            self.lookaround_active = False
            self.get_logger().info("Looking around")
            # Detect and publish emotion only if it has changed
            look_pattern = [
            (65, 55),
            (75, 68),
            (90, 60),
            (75, 55),
            (65, 45),
            (55, 55),
            (40, 55),
            (55,55) #Return to original position
            ]
            for yaw, pitch in look_pattern:
                if not self.lookaround_active or not rclpy.ok():
                    break  # Exit if flag is reset or node is shutting down

                self.robot_neck.move_servo(yaw, pitch)
                # If face was detected during spin_once, break out
                if not self.lookaround_active:
                    self.get_logger().warn("Face detected during look around — breaking out.")
                    break
        else:
            return
    
    def destroy(self):
        """Clean up resources."""
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
