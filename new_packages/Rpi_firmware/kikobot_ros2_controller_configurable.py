#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
import serial.tools.list_ports
import time
import numpy as np
import threading
import logging
import sys
import argparse

# Import the ST3215 library (assume installed or in PYTHONPATH)
try:
    from st3215 import ST3215
except ImportError:
    sys.path.append('./python-st3215-main')
    from st3215 import ST3215

# Constants
NUM_JOINTS = 6
GRIPPER_ID = 7
GRIPPER_OPEN_ANGLE = -75.0
GRIPPER_CLOSE_ANGLE = -5.0
MIN_ANGLE = -165.0
MAX_ANGLE = 165.0
ST_MIN_POSITION = 0
ST_MAX_POSITION = 4095
ST_SERVO_RESOLUTION = (ST_MAX_POSITION - ST_MIN_POSITION) / (MAX_ANGLE - MIN_ANGLE)
MAX_VELOCITY_ST3215 = 4.71
MAX_VELOCITY_ST3215_HS = 7.0
MAX_ACCELERATION = 1.0
MAX_SPEED_VALUE = 3073
MAX_ACC_VALUE = 150
TEACHING_BUFFER_CAPACITY = 2000

# Home positions for each motor (in degrees)
HOME_POSITIONS = np.array([-90.0, -35.815, 91.663, 49.507, 83.0, 90.0])

# Helper functions
def clamp(value, minv, maxv):
    return max(minv, min(value, maxv))

def angle_to_st_position(angle):
    clamped_angle = clamp(angle, MIN_ANGLE, MAX_ANGLE)
    return round((clamped_angle - MIN_ANGLE) * ST_SERVO_RESOLUTION + ST_MIN_POSITION)

def map_velocity_to_speed(velocity, max_vel):
    abs_vel = abs(velocity)
    normalized_vel = clamp(abs_vel / max_vel, 0.0, 1.0)
    return round(normalized_vel * MAX_SPEED_VALUE)

def map_acceleration_to_acc(acceleration):
    # For now, just use max
    return MAX_ACC_VALUE

def autodetect_usb_port():
    """Detect USB serial port automatically"""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'USB' in port.description:
            return port.device
    return None

def get_available_serial_ports():
    """Get list of all available serial ports"""
    ports = []
    # Common Raspberry Pi serial ports
    raspi_ports = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']
    
    # USB serial ports
    usb_ports = [f'/dev/ttyUSB{i}' for i in range(4)]
    acm_ports = [f'/dev/ttyACM{i}' for i in range(4)]
    
    all_possible_ports = raspi_ports + usb_ports + acm_ports
    
    for port in all_possible_ports:
        try:
            # Try to access the port to see if it exists
            import os
            if os.path.exists(port):
                ports.append(port)
        except:
            pass
    
    return ports

def select_serial_port(custom_port=None, prefer_gpio=False):
    """
    Select serial port based on preference or custom specification
    
    Args:
        custom_port: Specific port to use (e.g., '/dev/serial0')
        prefer_gpio: If True, prefer GPIO UART over USB
    
    Returns:
        Selected port path
    """
    if custom_port:
        return custom_port
    
    if prefer_gpio:
        # Try GPIO UART first (pins 14/15)
        gpio_ports = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']
        for port in gpio_ports:
            try:
                import os
                if os.path.exists(port):
                    return port
            except:
                pass
    
    # Fall back to USB detection
    usb_port = autodetect_usb_port()
    if usb_port:
        return usb_port
    
    # Try to find any available GPIO port as final fallback
    gpio_ports = ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/serial0']
    for port in gpio_ports:
        try:
            import os
            if os.path.exists(port):
                return port
        except:
            pass
    
    # Final fallback
    return '/dev/serial0'

class KikobotControllerConfigurable(Node):
    def __init__(self, serial_port=None, prefer_gpio=False):
        super().__init__('kikobot_controller_configurable')
        self.get_logger().info('Starting KikobotController with configurable serial port...')
        
        # Declare ROS parameter for serial port
        self.declare_parameter('serial_port', '')
        self.declare_parameter('prefer_gpio', False)
        
        # Get serial port from parameter or argument
        ros_serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        ros_prefer_gpio = self.get_parameter('prefer_gpio').get_parameter_value().bool_value
        
        # Priority: command line arg > ROS parameter > default
        final_port = serial_port or ros_serial_port or None
        final_prefer_gpio = prefer_gpio or ros_prefer_gpio
        
        self.selected_port = select_serial_port(final_port, final_prefer_gpio)
        self.servo = self._init_servo()
        
        # Initialize other components
        self.teaching_buffer = np.zeros((TEACHING_BUFFER_CAPACITY, NUM_JOINTS), dtype=float)
        self.teaching_buffer_index = 0
        self.hand_teaching_mode = False
        self.trajectory_mode = True
        self.lock = threading.Lock()
        
        # Create subscriptions
        self.create_subscription(JointTrajectoryPoint, '/traj_point_topic', self.trajectory_callback, 10)
        self.create_subscription(Float32MultiArray, 'hand_teaching_command', self.hand_teaching_callback, 10)
        
        self.move_to_home_positions()
        self.get_logger().info('KikobotController ready.')

    def _init_servo(self):
        self.get_logger().info(f"Using serial port: {self.selected_port}")
        self.get_logger().info(f"Available ports: {get_available_serial_ports()}")
        try:
            return ST3215(self.selected_port)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo on {self.selected_port}: {e}")
            raise

    def move_to_home_positions(self):
        self.get_logger().info('Moving to home positions...')
        for i in range(NUM_JOINTS):
            servo_id = i + 1
            pos = angle_to_st_position(HOME_POSITIONS[i])
            try:
                self.servo.MoveTo(servo_id, pos, speed=500, acc=50)
            except Exception as e:
                self.get_logger().error(f"Error moving servo {servo_id} to home: {e}")
            time.sleep(0.2)

    def move_gripper(self, open_gripper):
        angle = GRIPPER_OPEN_ANGLE if open_gripper else GRIPPER_CLOSE_ANGLE
        pos = angle_to_st_position(angle)
        try:
            self.servo.MoveTo(GRIPPER_ID, pos, speed=MAX_SPEED_VALUE, acc=MAX_ACC_VALUE)
            self.get_logger().info(f"Gripper {'opened' if open_gripper else 'closed'}.")
        except Exception as e:
            self.get_logger().error(f"Error moving gripper: {e}")

    def save_current_teaching_position(self):
        if self.teaching_buffer_index < TEACHING_BUFFER_CAPACITY:
            for i in range(NUM_JOINTS):
                try:
                    pos = self.servo.ReadPosition(i + 1)
                    self.teaching_buffer[self.teaching_buffer_index, i] = pos
                except Exception as e:
                    self.get_logger().error(f"Error reading position for servo {i+1}: {e}")
            self.teaching_buffer_index += 1

    def move_robot_from_teaching_buffer(self):
        self.get_logger().info('Replaying teaching buffer...')
        for i in range(self.teaching_buffer_index):
            for j in range(NUM_JOINTS):
                servo_id = j + 1
                pos = int(self.teaching_buffer[i, j])
                try:
                    self.servo.MoveTo(servo_id, pos, speed=1200, acc=100)
                except Exception as e:
                    self.get_logger().error(f"Error moving servo {servo_id} from buffer: {e}")
            time.sleep(0.02)

    def trajectory_callback(self, msg):
        if not self.trajectory_mode or self.hand_teaching_mode:
            return
        
        # Print incoming trajectory data
        self.get_logger().info("=== Incoming Trajectory Data ===")
        self.get_logger().info(f"Positions (rad): {list(msg.positions)}")
        self.get_logger().info(f"Velocities (rad/s): {list(msg.velocities)}")
        self.get_logger().info(f"Accelerations (rad/s²): {list(msg.accelerations)}")
        
        # Convert and print in degrees for clarity
        positions_deg = [pos * 180.0 / np.pi for pos in msg.positions]
        self.get_logger().info(f"Positions (deg): {[round(pos, 2) for pos in positions_deg]}")
        
        for i in range(NUM_JOINTS):
            servo_id = i + 1
            max_vel = MAX_VELOCITY_ST3215 if servo_id <= 3 else MAX_VELOCITY_ST3215_HS
            try:
                position_rad = msg.positions[i]
                velocity_rad_s = msg.velocities[i] + 7.0  # offset as in firmware
                acceleration_rad_s2 = msg.accelerations[i]
                position_deg = position_rad * 180.0 / np.pi
                # Orientation/sign corrections
                if i == 2 or i == 3:
                    position_deg = -(position_rad * 180.0 / np.pi)
                    if i == 3:
                        position_deg += 15.0
                elif i == 4:
                    position_deg -= 8.0
                servo_position = angle_to_st_position(position_deg)
                servo_speed = map_velocity_to_speed(velocity_rad_s, max_vel)
                servo_acc = map_acceleration_to_acc(acceleration_rad_s2)
                
                # Print servo command details
                self.get_logger().info(f"Servo {servo_id}: pos={servo_position}, speed={servo_speed}, acc={servo_acc}")
                
                self.servo.MoveTo(servo_id, servo_position, speed=servo_speed, acc=servo_acc)
            except Exception as e:
                self.get_logger().error(f"Trajectory error for servo {servo_id}: {e}")
        
        self.get_logger().info("=== End Trajectory Data ===\n")

    def hand_teaching_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn('Hand teaching command must have 3 elements.')
            return
        teach, save, run = msg.data
        open_cmd = teach
        close_cmd = save
        home_cmd = teach
        if teach == 1.0:
            self.hand_teaching_mode = True
            self.teaching_buffer_index = 0
            for i in range(NUM_JOINTS):
                try:
                    self.servo.StopServo(i + 1)
                except Exception as e:
                    self.get_logger().error(f"Error disabling torque for servo {i+1}: {e}")
            self.get_logger().info('Hand teaching mode enabled. Torque disabled.')
        elif save == 1.0:
            self.hand_teaching_mode = False
            self.save_current_teaching_position()
            for i in range(NUM_JOINTS):
                try:
                    self.servo.StartServo(i + 1)
                except Exception as e:
                    self.get_logger().error(f"Error enabling torque for servo {i+1}: {e}")
            self.get_logger().info('Teaching position saved. Torque enabled.')
        elif run == 1.0:
            self.hand_teaching_mode = False
            self.move_robot_from_teaching_buffer()
            self.trajectory_mode = True
            self.get_logger().info('Teaching buffer replayed.')
        elif open_cmd == 2.0:
            self.move_gripper(True)
            self.trajectory_mode = True
            self.hand_teaching_mode = False
        elif close_cmd == 2.0:
            self.move_gripper(False)
            self.trajectory_mode = True
            self.hand_teaching_mode = False
        elif home_cmd == 3.0:
            self.move_to_home_positions()
            self.trajectory_mode = True
            self.hand_teaching_mode = False

    def teaching_sampling_loop(self):
        # Call this in a thread to sample positions every 5ms in hand teaching mode
        while rclpy.ok():
            if self.hand_teaching_mode:
                self.save_current_teaching_position()
                time.sleep(0.005)
            else:
                time.sleep(0.05)

def main(args=None):
    parser = argparse.ArgumentParser(description='Kikobot ROS2 Controller with configurable serial port')
    parser.add_argument('--port', type=str, default=None, 
                       help='Serial port to use (e.g., /dev/serial0, /dev/ttyUSB0)')
    parser.add_argument('--gpio', action='store_true', 
                       help='Prefer GPIO UART (pins 14/15) over USB')
    parser.add_argument('--list-ports', action='store_true',
                       help='List available serial ports and exit')
    
    parsed_args, ros_args = parser.parse_known_args()
    
    if parsed_args.list_ports:
        print("Available serial ports:")
        for port in get_available_serial_ports():
            print(f"  {port}")
        return
    
    rclpy.init(args=ros_args)
    controller = KikobotControllerConfigurable(
        serial_port=parsed_args.port,
        prefer_gpio=parsed_args.gpio
    )
    
    teaching_thread = threading.Thread(target=controller.teaching_sampling_loop, daemon=True)
    teaching_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()