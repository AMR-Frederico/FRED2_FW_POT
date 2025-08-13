#include "micro_ros.h"
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

// --- Global Variables for micro-ROS communication ---
rcl_subscription_t cmd_vel_subscriber;                ///< Subscriber for velocity commands
rcl_subscription_t kp_calib_subscriber;
rcl_subscription_t ki_calib_subscriber;
rcl_subscription_t kd_calib_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;                 ///< Message structure for received Twist
std_msgs__msg__Float32 kp_msg;
std_msgs__msg__Float32 ki_msg;
std_msgs__msg__Float32 kd_msg;
rclc_executor_t executor;                             ///< micro-ROS executor to handle callbacks
rcl_allocator_t allocator;                            ///< Allocator for memory management
rclc_support_t support;                               ///< micro-ROS support structure
rcl_node_t node;                                       ///< micro-ROS Node

// Publishers for encoder data
rcl_publisher_t encoder_ticks_right_pub;
std_msgs__msg__Int32 encoder_ticks_right_msg;

rcl_publisher_t encoder_ticks_left_pub;
std_msgs__msg__Int32 encoder_ticks_left_msg;

// Additional publishers are commented for future extension

rcl_publisher_t controlled_pwm_left_pub; 
std_msgs__msg__Float32 controlled_pwm_left_msg; 

rcl_publisher_t controlled_pwm_right_pub; 
std_msgs__msg__Float32 controlled_pwm_right_msg; 

// --- Constants ---
#define LED_PIN 2                        ///< Onboard LED pin for error indication
#define TIMEOUT_INTERVAL_MS 1000          ///< Timeout interval for cmd_vel messages (in ms)

// --- Internal State Variables ---
unsigned long last_cmd_vel_time = 0;      ///< Timestamp of last received cmd_vel
float speed_linear = 0.0;                 ///< Latest linear speed (m/s)
float speed_angular = 0.0;                ///< Latest angular speed (rad/s)
float kp_, ki_, kd_;
// --- Macro Helpers for Error Checking ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// -------------------------------------------------------
// Error handling loop
// -------------------------------------------------------

/**
 * @brief Blinks LED 10 times and restarts ESP if a critical error occurs.
 */
void error_loop() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  ESP.restart();
}

// -------------------------------------------------------
// Subscription callback for /cmd_vel
// -------------------------------------------------------

/**
 * @brief Callback executed when a /cmd_vel message is received.
 * 
 * @param msgin Pointer to received message (geometry_msgs::msg::Twist).
 */
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  speed_linear = msg->linear.x;
  speed_angular = msg->angular.z;
  last_cmd_vel_time = millis();
}

void kp_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*) msgin;
    kp_ = msg->data;
}

void ki_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*) msgin;
    ki_ = msg->data;
}

void kd_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*) msgin;
    kd_ = msg->data;
}
// -------------------------------------------------------
// Getters for latest received commands
// -------------------------------------------------------

/**
 * @brief Get the latest received linear speed command.
 * 
 * @return Linear velocity (m/s).
 */
float getLinear() { 
  return speed_linear; 
}

/**
 * @brief Get the latest received angular speed command.
 * 
 * @return Angular velocity (rad/s).
 */
float getAngular() { 
  return speed_angular; 
}

// -------------------------------------------------------
// Check cmd_vel message timeout
// -------------------------------------------------------

/**
 * @brief Stops the robot if no cmd_vel message is received within TIMEOUT_INTERVAL_MS.
 */
void check_cmd_vel_timeout() {
  unsigned long current_time = millis();
  if (current_time - last_cmd_vel_time > TIMEOUT_INTERVAL_MS) {
    speed_linear = 0;
    speed_angular = 0;
  }
}

// -------------------------------------------------------
// Initialize micro-ROS Node, Subscribers, and Publishers
// -------------------------------------------------------

/**
 * @brief Initializes micro-ROS communication and sets up the node, subscriber, and publishers.
 * 
 * @param node_name Name of the micro-ROS node.
 * @param node_namespace Namespace for the node.
 */
void init_ros(const char* node_name, const char* node_namespace, bool calib) {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, node_namespace, &support));

  // --- Subscriber ---
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/safe"));
  if(calib){
    RCCHECK(rclc_subscription_init_default(
    &kp_calib_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/calib/kp"));

    RCCHECK(rclc_subscription_init_default(
    &ki_calib_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/calib/ki"));

    RCCHECK(rclc_subscription_init_default(
    &kd_calib_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/calib/kd"));
  }

  // --- Publishers ---
  RCCHECK(rclc_publisher_init_default(
    &encoder_ticks_right_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "power/encoder/ticks/right"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_ticks_left_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "power/encoder/ticks/left"));

  RCCHECK(rclc_publisher_init_default(
    &controlled_pwm_left_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "power/controlled/pwm/left"));

  RCCHECK(rclc_publisher_init_default(
    &controlled_pwm_right_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "power/controlled/pwm/right"));

  // --- Executor ---
  int num_nodes_executor = calib? 4 : 1;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_nodes_executor, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));
  
  if(calib){

    RCCHECK(rclc_executor_add_subscription(&executor, &kp_calib_subscriber, &kp_msg, &kp_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &ki_calib_subscriber, &ki_msg, &ki_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &kd_calib_subscriber, &kd_msg, &kd_callback, ON_NEW_DATA));
  }
}

// -------------------------------------------------------
// Publish updated encoder and control data
// -------------------------------------------------------

/**
 * @brief Publishes encoder tick counts and controlled PWM signals to ROS topics.
 * 
 * @param data Struct containing the encoder and control data (RosData type).
 */
void ros_loop(const RosData& data) {
  encoder_ticks_right_msg.data = data.ticks_encoder_right;
  encoder_ticks_left_msg.data = data.ticks_encoder_left;
  controlled_pwm_left_msg.data = data.controlled_pwm_left;
  controlled_pwm_right_msg.data = data.controlled_pwm_right;

  RCSOFTCHECK(rcl_publish(&encoder_ticks_right_pub, &encoder_ticks_right_msg, NULL));
  RCSOFTCHECK(rcl_publish(&encoder_ticks_left_pub, &encoder_ticks_left_msg, NULL));
  RCSOFTCHECK(rcl_publish(&controlled_pwm_left_pub, &controlled_pwm_left_msg, NULL));
  RCSOFTCHECK(rcl_publish(&controlled_pwm_right_pub, &controlled_pwm_right_msg, NULL));

  check_cmd_vel_timeout();
}

// -------------------------------------------------------
// Run the micro-ROS executor
// -------------------------------------------------------

/**
 * @brief Spins the micro-ROS executor to process callbacks.
 */
void ros_spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
