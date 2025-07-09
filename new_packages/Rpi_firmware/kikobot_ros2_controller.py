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
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'USB' in port.description:
            return port.device
    return None

class KikobotController(Node):
    def __init__(self):
        super().__init__('kikobot_controller')
        self.get_logger().info('Starting KikobotController...')
        self.servo = self._init_servo()
        self.teaching_buffer = np.zeros((TEACHING_BUFFER_CAPACITY, NUM_JOINTS), dtype=float)
        self.teaching_buffer_index = 0
        self.hand_teaching_mode = False
        self.trajectory_mode = True
        self.lock = threading.Lock()
        self.create_subscription(JointTrajectoryPoint, '/traj_point_topic', self.trajectory_callback, 10)
        self.create_subscription(Float32MultiArray, 'hand_teaching_command', self.hand_teaching_callback, 10)
        self.move_to_home_positions()
        self.get_logger().info('KikobotController ready.')

    def _init_servo(self):
        usb_port = autodetect_usb_port()
        if usb_port:
            self.get_logger().info(f"Detected USB port: {usb_port}")
            return ST3215(usb_port)
        else:
            self.get_logger().warn("No USB port detected. Falling back to /dev/serial0")
            return ST3215('/dev/serial0')

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
                self.servo.MoveTo(servo_id, servo_position, speed=servo_speed, acc=servo_acc)
            except Exception as e:
                self.get_logger().error(f"Trajectory error for servo {servo_id}: {e}")

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
    rclpy.init(args=args)
    controller = KikobotController()
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