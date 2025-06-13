/************************************************************
 * Arduino Firmware for Mixed ST3215 (IDs 1..3) and
 * ST3215-HS (IDs 4..6) servo motors
 *
 * Changes Implemented:
 *  - Distinguish ST3215 vs. ST3215-HS by servo ID
 *  - Different max velocity for each type
 *  - Removed the previous *25 speed scaling
 *  - Comments explaining new code blocks
 ************************************************************/

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <SCServo.h>  // Waveshare / Feetech SCServo library

// =============================
// Pin definitions and constants
// =============================
#define LED_PIN 13
#define NUM_JOINTS 6
// Change TX/RX if needed for your custom board
#define S_RXD 18  
#define S_TXD 19
#define CONTROL_CYCLE 25
#define TEACHING_BUFFER_CAPACITY 2000

// =============================
// Gripper motor configuration
// =============================
#define GRIPPER_ID 7
const float GRIPPER_OPEN_ANGLE  = -75.0f;
const float GRIPPER_CLOSE_ANGLE = -5.0f;

// =============================
// Servo configuration constants
// =============================
const float MIN_ANGLE = -165.0f;
const float MAX_ANGLE =  165.0f;
const int   ST_MIN_POSITION = 0;
const int   ST_MAX_POSITION = 4095;

// The ST servo range is 0..4095 steps for -165..165 degrees
const float ST_SERVO_RESOLUTION = 
    (ST_MAX_POSITION - ST_MIN_POSITION) / (MAX_ANGLE - MIN_ANGLE);

// =============================
// Distinguish two servo types:
//  - ST3215 for IDs 1..3
//  - ST3215-HS for IDs 4..6
// Each has a different max velocity in rad/s
// (Approx values from Waveshare docs):
//   ST3215    ~ 4.71 rad/s (270 deg/s)
//   ST3215-HS ~ 7.0  rad/s (400+ deg/s)
// =============================
#define MAX_VELOCITY_ST3215    4.71f
#define MAX_VELOCITY_ST3215_HS 7.0f

// Acceleration is still a single constant for both
#define MAX_ACCELERATION 1.0f

// The library expects a speed param up to ~3073
#define MAX_SPEED_VALUE 3073
#define MAX_ACC_VALUE   150

// =============================
// ROS entities
// =============================
rcl_subscription_t trajectory_subscriber;
rcl_subscription_t hand_teaching_command_subscriber;
rclc_executor_t    executor;
rcl_timer_t        timer;

// =============================
// Messages
// =============================
trajectory_msgs__msg__JointTrajectoryPoint traj_msg;
std_msgs__msg__Float32MultiArray           hand_teaching_command_msg;

// =============================
// Servo control object
// =============================
SMS_STS st;

// =============================
// Hand teaching mode variables
// =============================
float teaching_buffer[TEACHING_BUFFER_CAPACITY][NUM_JOINTS];
int   teaching_buffer_index = 0;
bool  hand_teaching_mode    = false;
bool  trajectory_mode       = true;

// Home positions for each motor (in degrees)
float home_positions[] = {
    -90.0f, -35.815f, 91.663f, 49.507f, 83.0f, 90.0f
};

// ---------------------------------------------------------------------
// Utility clamp function
// ---------------------------------------------------------------------
template<typename T>
T clamp(T value, T minv, T maxv) {
  return (value < minv) ? minv : (value > maxv ? maxv : value);
}

// ---------------------------------------------------------------------
// Convert a [deg] angle to internal servo [0..4095] steps
// ---------------------------------------------------------------------
int angleToSTPosition(float angle) {
  float clamped_angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
  return round((clamped_angle - MIN_ANGLE) * ST_SERVO_RESOLUTION 
               + ST_MIN_POSITION);
}

// ---------------------------------------------------------------------
// Read servo position in degrees from [0..4095]
// ---------------------------------------------------------------------
float readSTServoPosition(int id) {
  int pos = st.ReadPos(id);
  if (pos == -1) {
    return -1; // read failure
  }
  float deg = (pos / ST_SERVO_RESOLUTION) - 165.0f;
  return clamp(deg, MIN_ANGLE, MAX_ANGLE);
}

// ---------------------------------------------------------------------
// mapVelocityToSpeed(...)
//   - Distinguishes ST3215 vs. ST3215-HS using 'max_vel' param
//   - Maps an incoming rad/s velocity to [0..MAX_SPEED_VALUE]
// ---------------------------------------------------------------------
int mapVelocityToSpeed(float velocity, float max_vel) {
  float abs_vel = fabs(velocity);
  // normalized_vel = fraction of motor's max possible velocity
  float normalized_vel = clamp(abs_vel / max_vel, 0.0f, 1.0f);

  // Scale into [0..3073] range
  int servo_speed = round(normalized_vel * MAX_SPEED_VALUE);
  return servo_speed;
}

// ---------------------------------------------------------------------
// mapAccelerationToAcc(...)
//   - For simplicity, we always use the max acceleration param
//   - Or you can scale similarly to velocity if you prefer
// ---------------------------------------------------------------------
int mapAccelerationToAcc(float acceleration) {
  // If you want real scaling:
  // float abs_acc = fabs(acceleration);
  // float norm_acc = clamp(abs_acc / MAX_ACCELERATION, 0.0f, 1.0f);
  // return round(norm_acc * MAX_ACC_VALUE);

  // For now, just use max:
  return MAX_ACC_VALUE;
}

// ---------------------------------------------------------------------
// Enable/disable torque on all servo IDs 1..6
// ---------------------------------------------------------------------
void enableTorqueAll(bool enable) {
  for (int i = 1; i <= NUM_JOINTS; i++) {
    st.EnableTorque(i, enable ? 1 : 0);
  }
}

// ---------------------------------------------------------------------
// Simple error blink loop
// ---------------------------------------------------------------------
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// =============================
// Gripper control
// =============================
void moveGripperToOpen() {
  Serial.println("Gripper Opening...");
  st.WritePosEx(GRIPPER_ID, 
                angleToSTPosition(GRIPPER_OPEN_ANGLE), 
                MAX_SPEED_VALUE, 
                MAX_ACC_VALUE);
}

void moveGripperToClose() {
  Serial.println("Gripper Closing...");
  st.WritePosEx(GRIPPER_ID, 
                angleToSTPosition(GRIPPER_CLOSE_ANGLE), 
                MAX_SPEED_VALUE, 
                MAX_ACC_VALUE);
}

// =============================
// Robot Control
// =============================
void moveToHomePositions() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    int servo_id = i + 1;
    int pos = angleToSTPosition(home_positions[i]);
    // example home speed/acc
    int home_speed = 500; 
    int home_acc   = 50;
    st.WritePosEx(servo_id, pos, home_speed, home_acc);
    delay(200);
  }
}

void moveRobotFromTeachingBuffer() {
  for (int i = 0; i < teaching_buffer_index; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      // If you want a uniform speed, pick a single value
      int servo_id = j + 1;
      float speed  = 1200.0f; // e.g., some speed param
      st.WritePosEx(servo_id, 
                    angleToSTPosition(teaching_buffer[i][j]), 
                    (int)speed, 
                    100);
    }
    delay(20);
  }
}

void saveCurrentTeachingPosition() {
  if (teaching_buffer_index < TEACHING_BUFFER_CAPACITY) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      teaching_buffer[teaching_buffer_index][i] 
          = readSTServoPosition(i + 1);
    }
    teaching_buffer_index++;
  }
}

// =============================
// ROS Callback: trajectory points
//   - Distinguishes motor type by servo_id (1..3 => ST3215, 4..6 => HS)
//   - Removes *25 scaling
// =============================
void trajectory_callback(const void *msgin) {
  if (!trajectory_mode || hand_teaching_mode) return;

  const trajectory_msgs__msg__JointTrajectoryPoint *msg =
    (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;

  for (int i = 0; i < NUM_JOINTS; i++) {
    // servo_id in [1..6]
    int servo_id = i + 1;

    // figure out which max velocity to use
    float max_vel = (servo_id <= 3) ? MAX_VELOCITY_ST3215 
                                    : MAX_VELOCITY_ST3215_HS;

    // read fields
    float position_rad        = msg->positions.data[i];
    float velocity_rad_s      = msg->velocities.data[i] + 7.0f; //2.5f added to add an offset speed increase
    float acceleration_rad_s2 = msg->accelerations.data[i];

    // Convert from rad to deg for servo
    float position_deg = position_rad * RAD_TO_DEG;

    // Original sign/orientation offsets
    if (i == 2 || i == 3) {
      position_deg = -(position_rad * RAD_TO_DEG);
      if (i == 3) {
        position_deg += 15.0f;
      }
    } else if (i == 4) {
      position_deg -= 8.0f;
    }

    // Convert angles to servo steps
    int servo_position = angleToSTPosition(position_deg);

    // map velocity/acc with new function
    int servo_speed = mapVelocityToSpeed(velocity_rad_s, max_vel);
    int servo_acc   = mapAccelerationToAcc(acceleration_rad_s2);

    // =========== REMOVE THE OLD *25 MULTIPLIER ===========

    // Command the servo
    st.WritePosEx(servo_id, servo_position, servo_speed, servo_acc);
  }
}

// =============================
// ROS Callback: hand_teaching
// =============================
void hand_teaching_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = 
    (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size != 3) return;

  trajectory_mode = false;
  float teach = msg->data.data[0];
  float save  = msg->data.data[1];
  float run   = msg->data.data[2];
  float open  = msg->data.data[0];
  float close = msg->data.data[1];
  float home  = msg->data.data[0];

  if (teach == 1.0f) {
    hand_teaching_mode = true;
    teaching_buffer_index = 0;
    enableTorqueAll(false);
  } else if (save == 1.0f) {
    hand_teaching_mode = false;
    saveCurrentTeachingPosition();
    enableTorqueAll(true);
  } else if (run == 1.0f) {
    hand_teaching_mode = false;
    enableTorqueAll(true);
    moveRobotFromTeachingBuffer();
    trajectory_mode = true;
  } else if (open == 2.0f) {
    moveGripperToOpen();
    trajectory_mode = true;
    hand_teaching_mode = false;
  } else if (close == 2.0f) {
    moveGripperToClose();
    trajectory_mode = true;
    hand_teaching_mode = false;
  } else if (home == 3.0f) {
    moveToHomePositions();
    trajectory_mode = true;
    hand_teaching_mode = false;
  }
}

// =============================
// setup()
//   - micro-ROS init
//   - create subscriptions
//   - move to home on startup
// =============================
void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  set_microros_transports();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  rcl_node_t node;
  if (rclc_node_init_default(&node, 
                             "micro_ros_servo_controller", 
                             "", 
                             &support) != RCL_RET_OK) {
    error_loop();
  }

  // Initialize trajectory message arrays
  traj_msg.positions.capacity     = NUM_JOINTS;
  traj_msg.positions.size         = NUM_JOINTS;
  traj_msg.positions.data         = 
      (double *)malloc(NUM_JOINTS * sizeof(double));

  traj_msg.velocities.capacity    = NUM_JOINTS;
  traj_msg.velocities.size        = NUM_JOINTS;
  traj_msg.velocities.data        = 
      (double *)malloc(NUM_JOINTS * sizeof(double));

  traj_msg.accelerations.capacity = NUM_JOINTS;
  traj_msg.accelerations.size     = NUM_JOINTS;
  traj_msg.accelerations.data     = 
      (double *)malloc(NUM_JOINTS * sizeof(double));

  // Initialize hand teaching command message
  hand_teaching_command_msg.data.data = 
      (float *)malloc(3 * sizeof(float));
  hand_teaching_command_msg.data.size = 3;
  hand_teaching_command_msg.data.capacity = 3;

  // Create subscribers
  if (rclc_subscription_init_default(
        &trajectory_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/traj_point_topic") != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_subscription_init_default(
        &hand_teaching_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "hand_teaching_command") != RCL_RET_OK) {
    error_loop();
  }

  // Create executor with space for both subscribers
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) 
      != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_executor_add_subscription(
        &executor,
        &trajectory_subscriber,
        &traj_msg,
        &trajectory_callback,
        ON_NEW_DATA) != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_executor_add_subscription(
        &executor,
        &hand_teaching_command_subscriber,
        &hand_teaching_command_msg,
        &hand_teaching_callback,
        ON_NEW_DATA) != RCL_RET_OK) {
    error_loop();
  }

  // Move to home position at startup
  moveToHomePositions();

  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

// =============================
// loop()
//   - executes micro-ROS spin
//   - handles hand-teaching sampling
// =============================
void loop() {
  static unsigned long last_teaching_time = 0;
  unsigned long current_time = millis();

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(CONTROL_CYCLE));

  // If in hand-teaching mode, capture positions every 5 ms
  if (hand_teaching_mode && (current_time - last_teaching_time >= 5)) {
    last_teaching_time = current_time;
    saveCurrentTeachingPosition();
  }
}