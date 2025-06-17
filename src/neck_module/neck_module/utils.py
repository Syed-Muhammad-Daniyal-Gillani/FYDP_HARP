import cv2
import serial
import serial.tools.list_ports

video_in = None  # Placeholder for camera input if used elsewhere

# PID Controller constants for regular face tracking
pid_yaw = [1.5, 0.5, 0.1]          # [Kp, Kd, Ki] for yaw (horizontal) control
pid_pitch = [0.7, 0.6, 0.1]        # [Kp, Kd, Ki] for pitch (vertical) control

# PID tuning constants for lookaround behavior
pid_yaw_lookaround = [1.5, .005, 0.05]
pid_pitch_lookaround = [0.7, 0.005, 0.04]


class RobotNeck:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=9600):
        # Initial servo positions
        self.servo_yaw = 55          # Default yaw angle (centered)
        self.servo_pitch = 55        # Default pitch angle (centered)
        
        # Initialize serial connection to ESP32
        self.serial_conn = serial.Serial(serial_port, baud_rate)

    def move_servo(self, yaw_angle, pitch_angle):
        """
        Move yaw and pitch servos to the specified angles.
        :param yaw_angle: Desired yaw angle in degrees (0-180).
        :param pitch_angle: Desired pitch angle in degrees (0-180).
        """
        # Constrain angles within safe bounds
        yaw_angle = max(20, min(90, yaw_angle))        # Clamp yaw between 20 and 90
        pitch_angle = max(40, min(70, pitch_angle))    # Clamp pitch between 40 and 70

        print(f"Moving servos - Yaw: {yaw_angle}, Pitch: {pitch_angle}")
        
        # Send formatted command to ESP32 via serial
        self.serial_conn.write(f"Y{yaw_angle}P{pitch_angle}\n".encode())

        # Update internal state
        self.servo_yaw = yaw_angle
        self.servo_pitch = pitch_angle

    def close(self):
        """Closes the serial connection."""
        self.serial_conn.close()


def trackFace(neck, info, pid_yaw, pid_pitch, pre_error_x, pre_error_y, tolerance_x=0.1, tolerance_y=0.1, pitch_offset=0.2):
    """
    Tracks the face and moves the servos based on PID correction for yaw and pitch.
    Adjusts the target to be slightly above center using a pitch offset.
    
    :param neck: RobotNeck instance
    :param info: Tuple of (normalized_x, normalized_y)
    :param pid_yaw: PID parameters for yaw
    :param pid_pitch: PID parameters for pitch
    :param pre_error_x: Previous error in x (used for derivative and integral)
    :param pre_error_y: Previous error in y
    :param tolerance_x: Error tolerance for yaw
    :param tolerance_y: Error tolerance for pitch
    :param pitch_offset: Upward offset for natural head orientation
    :return: New error values (correction_x, correction_y)
    """
    normalized_x, normalized_y = info
    
    # Apply pitch offset to raise the target's vertical position
    normalized_y += pitch_offset

    # Compute PID correction for yaw (x-axis)
    if abs(normalized_x) < tolerance_x:
        correction_x = 0  # Within tolerance, no correction needed
    else:
        correction_x = (
            pid_yaw[0] * normalized_x +                     # Proportional
            pid_yaw[1] * (normalized_x - pre_error_x) +     # Derivative
            pid_yaw[2] * (normalized_x + pre_error_x)       # Integral
        )

    # Compute PID correction for pitch (y-axis)
    if abs(normalized_y) < tolerance_y:
        correction_y = 0  # Within tolerance, no correction needed
    else:
        correction_y = (
            pid_pitch[0] * normalized_y +                   # Proportional
            pid_pitch[1] * (normalized_y - pre_error_y) +   # Derivative
            pid_pitch[2] * (normalized_y + pre_error_y)     # Integral (note: reuses x error for last term)
        )

    if normalized_x != 0 or normalized_y != 0:
        # Compute new angles based on current position and corrections
        new_yaw = neck.servo_yaw - correction_x
        new_pitch = neck.servo_pitch + correction_y  # Positive correction moves servo upward
        
        # Command the servos to move
        neck.move_servo(new_yaw, new_pitch)
    else:
        print("No face detected. No angle correction.")
        correction_x, correction_y = 0, 0

    return correction_x, correction_y


def find_esp32_port():
    """Scan all serial ports and return the correct ESP32 port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for a known descriptor indicating the ESP32 (you may adjust this string)
        if "USB Single Serial" in port.description:
            return port.device  # Return the port device name
    raise Exception("ESP32 not found!")  # Raise an error if no valid port is found


def lookAround():
    # Placeholder function; can be implemented with patterns if needed
    return
