import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from neck_module.utils import RobotNeck, trackFace, pid

class NeckController(Node):
    def __init__(self):
        super().__init__('neck_controller')
        
        # Subscribe to the face tracking topic
        self.neck_subscription = self.create_subscription(
            Float32MultiArray,
            'neck_coordinates',
            self.coordinates_callback,
            1  
        )

        self.search_subscription = self.create_subscription(
            String,
            'look_around',
            self.lookaround_callback,
            1
        )

        self.search_face = False
        
        # Initialize neck movement
        self.robot_neck = RobotNeck(serial_port='/dev/ttyACM0', baud_rate=9600)  # Update port if needed
        self.pre_error_x = 0
        self.pre_error_y = 0

        self.get_logger().info("Neck Controller Node Started, listening to /neck_coordinates...")

    def coordinates_callback(self, msg):
        """Callback function to process received coordinates."""
        if len(msg.data) >= 2:
            normalized_x, normalized_y = msg.data[0], msg.data[1]
            self.get_logger().info(f"Received Coordinates - X: {normalized_x}, Y: {normalized_y}")
    
            if self.search_face:
                self.search_face = False  # Stop search if a face is found
                self.get_logger().info("Face detected! Stopping search and tracking face.")
                
            # Track the face using the received coordinates
            self.pre_error_x, self.pre_error_y = trackFace(
                self.robot_neck, (normalized_x, normalized_y), pid, self.pre_error_x, self.pre_error_y
            )
        else:
            self.get_logger().info("Starting lookaround mode...")

            
    
    def lookaround_callback(self, msg):
        msg = String()
        self.get_logger().info("Recieved lookaround message")
        if msg.data == "lookaround" and not self.search_face:
            self.search_face = True
            self.get_logger().info("Starting lookaround mode...")


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
