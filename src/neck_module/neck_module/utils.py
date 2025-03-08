import cv2
import serial
import time
video_in = None
# PID Controller constants [Kp, Kd, Ki]
pid = [6, 1, 2]

class RobotNeck:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=9600):
        self.servo_yaw = 90  # Default yaw angle (horizontal)
        self.servo_pitch = 40  # Default pitch angle (vertical)
        self.serial_conn = serial.Serial(serial_port, baud_rate)

    def move_servo(self, yaw_angle, pitch_angle):
        """
        Move yaw and pitch servos to the specified angles.
        :param yaw_angle: Desired yaw angle in degrees (0-180).
        :param pitch_angle: Desired pitch angle in degrees (0-180).
        """
        yaw_angle = max(20, min(90, yaw_angle))  # Constrain angle between 20 and 90
        pitch_angle = max(40, min(70, pitch_angle))  # Constrain angle between 40 and 70

        print(f"Moving servos - Yaw: {yaw_angle}, Pitch: {pitch_angle}")
        self.serial_conn.write(f"Y{yaw_angle}P{pitch_angle}\n".encode())  # Send to ESP32
        self.servo_yaw = yaw_angle
        self.servo_pitch = pitch_angle

    def close(self):
        """Closes the serial connection."""
        self.serial_conn.close()

def trackFace(neck, info, pid, pre_error_x, pre_error_y, tolerance_x=0.1, tolerance_y=0.1):
    """
    Tracks the face and moves the servos based on PID correction.
    """
    normalized_x, normalized_y = info
    # correction_x = pid[0] * normalized_x + pid[1] * (normalized_x - pre_error_x)
    # correction_y = pid[0] * normalized_y + pid[1] * (normalized_y - pre_error_y)
    # If the error is within the tolerance range, skip PID calculation for that axis.
    if abs(normalized_x) < tolerance_x:
        correction_x = 0  # No correction needed if within tolerance
    else:
        correction_x = pid[0] * normalized_x + pid[1] * (normalized_x - pre_error_x)
    
    if abs(normalized_y) < tolerance_y:
        correction_y = 0  # No correction needed if within tolerance
    else:
        correction_y = pid[0] * normalized_y + pid[1] * (normalized_y - pre_error_y)


    if normalized_x != 0 or normalized_y != 0:
        # Calculate new angles for both servos
        new_yaw = neck.servo_yaw - correction_x
        new_pitch = neck.servo_pitch + correction_y  # Negative because of inverted axis
        neck.move_servo(new_yaw, new_pitch)
    else:
        print("No face detected. No angle correction.")
        correction_x, correction_y = 0, 0

    return correction_x, correction_y

def searchFace(neck, face_detected_callback, search_positions=None, delay=2):

    if search_positions is None:
        search_positions = [
            (20, 55),  # Center
            (55, 55),  # Left
            (90, 55), # Right
            (90, 40),  # Up
            (55, 70),  # Down
            (55, 50),  # Reset to Center
        ]

    for yaw, pitch in search_positions:
        # Before moving, check if a face is detected
        if face_detected_callback():
            print("Face detected! Stopping search immediately.")
            break  # Exit loop early

        # Move the servos to a new position
        neck.move_servo(yaw, pitch)
        time.sleep(delay)  # Wait before next movement

        # Check again after movement
        if face_detected_callback():
            print("Face detected! Stopping search immediately.")
            break