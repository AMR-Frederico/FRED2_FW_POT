#include <Arduino.h>
#include "micro_ros.h"
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>


// --- Global Variables for micro-ROS communication ---
rcl_subscription_t cmd_vel_subscriber;                ///< Subscriber for velocity commands
custom_message__msg__Vel cmd_vel_msg;                 ///< Message structure for received Twist

rclc_executor_t executor;                             ///< micro-ROS executor to handle callbacks
rcl_allocator_t allocator;                            ///< Allocator for memory management
rclc_support_t support;                               ///< micro-ROS support structure
rcl_node_t node;                                       ///< micro-ROS Node

// Publishers for encoder data
rcl_publisher_t encoder_ticks_pub;
custom_message__msg__Encoder encoder_msg; 

extern RosData ros_data; 

// Additional publishers are commented for future extension

rcl_publisher_t controlled_pwm_left_pub; 
std_msgs__msg__Float32 controlled_pwm_left_msg; 

rcl_publisher_t controlled_pwm_right_pub; 
std_msgs__msg__Float32 controlled_pwm_right_msg; 

rcl_publisher_t latency_pub;
std_msgs__msg__Int64 latency_msg; // Int64 é bom para microssegundos

// --- Constants ---
#define LED_PIN 2                        ///< Onboard LED pin for error indication
#define TIMEOUT_INTERVAL_MS 1000          ///< Timeout interval for cmd_vel messages (in ms)

// --- Internal State Variables ---
unsigned long last_cmd_vel_time = 0;      ///< Timestamp of last received cmd_vel
float speed_linear = 0.0;                 ///< Latest linear speed (m/s)
float speed_angular = 0.0;                ///< Latest angular speed (rad/s)

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
  // 1. Captura o tempo de recebimento no ESP32 (em microssegundos)
  long long receive_time_us = (long long)micros();

  // 2. Faz o cast da mensagem para o tipo correto (Vel)
  const custom_message__msg__Vel *msg = (const custom_message__msg__Vel *)msgin;

  // 3. Extrai o timestamp de publicação do Pi e converte para microssegundos
  long long publish_time_sec = msg->stamp.sec;
  long long publish_time_nanosec = msg->stamp.nanosec;
  long long publish_time_us = (publish_time_sec * 1000000LL) + (publish_time_nanosec / 1000LL);

  // 4. Calcula a latência! Este é o dado principal do seu experimento.
  long long latency_us = receive_time_us - publish_time_us;
  
  // 4.1 Salva a latência na struct global
  ros_data.latency_us = latency_us; // <<< ALTERAÇÃO IMPORTANTE

  // 5. Extrai as velocidades como antes, mas da nova estrutura da mensagem
  speed_linear = msg->linear.x;
  speed_angular = msg->angular.z; // Geralmente o robô 2D gira no eixo Z

  // 6. Atualiza o timestamp do último comando recebido para o timeout
  last_cmd_vel_time = millis();
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
void init_ros(const char* node_name, const char* node_namespace) {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Vel),
    "/cmd_vel/safe"));

  // --- Publishers ---
  RCCHECK(rclc_publisher_init_default(
    &encoder_ticks_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Encoder),
    "power/encoder"));

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

  RCCHECK(rclc_publisher_init_default(
    &latency_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "firmware/latency"));

  // --- Executor ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));
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
  encoder_msg.left_ticks = data.ticks_encoder_left;
  encoder_msg.right_ticks = data.ticks_encoder_right;
  encoder_msg.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000ULL);
  encoder_msg.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000ULL);


  controlled_pwm_left_msg.data = data.controlled_pwm_left;
  controlled_pwm_right_msg.data = data.controlled_pwm_right;
  
  
  latency_msg.data = data.latency_us;


  RCSOFTCHECK(rcl_publish(&encoder_ticks_pub, &encoder_msg, NULL));
  RCSOFTCHECK(rcl_publish(&controlled_pwm_left_pub, &controlled_pwm_left_msg, NULL));
  RCSOFTCHECK(rcl_publish(&controlled_pwm_right_pub, &controlled_pwm_right_msg, NULL));
  RCSOFTCHECK(rcl_publish(&latency_pub, &latency_msg, NULL));


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
