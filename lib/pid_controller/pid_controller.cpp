#include "pid_controller.hpp"
#include "Arduino.h"

// -------------------------------------------------------
// Controller Class Constructor
// -------------------------------------------------------

/**
 * @brief Initializes the PID controller with given gains and output limits.
 * 
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param out_lim Output saturation limit.
 */
PIDController::PIDController(const PIDConfig& pid_config)
: Kp(pid_config.kp), Ki(pid_config.ki), Kd(pid_config.kd),
  integral(0.0f),
  last_error(0.0f),
  last_time_ms(millis()),
  output_limit(pid_config.output_limit),
  last_output(0.0f),
  last_derivative(0.0f) // For debugging derivative term
{}

// -------------------------------------------------------
// Compute PID Output
// -------------------------------------------------------

/**
 * @brief Computes the PID control signal based on the setpoint and current measurement.
 * 
 * @param setpoint Desired target value.
 * @param measurement Current measured value.
 * @return Control signal after applying PID calculation and output limiting.
 */
float PIDController::compute_pid_control(float setpoint, float measurement) {
  unsigned long now = millis();
  float dt = (now - last_time_ms) * 1e-3f;  // Convert elapsed time from ms to seconds
  last_time_ms = now;

  // --- Calculate error ---
  float error = setpoint - measurement;

  // --- Proportional term ---
  float P = Kp * error;

  // --- Integral term (with accumulation) ---
  integral += error * dt;
  float I = Ki * integral;

  // --- Derivative term (rate of error change) ---
  float D = 0.0f;
  if (dt > 0.0f) {
    last_derivative = (error - last_error) / dt;
    D = Kd * last_derivative;
  }
  last_error = error;

  // --- Compute total control output ---
  float u = P + I + D;

  // --- Clamp output within set limits ---
  if (output_limit > 0.0f) {
    u = constrain(u, -output_limit, output_limit);
  }

  last_output = u;
  return u;
}

// -------------------------------------------------------
// Debugging Output for PID Terms
// -------------------------------------------------------

/**
 * @brief Prints current PID components and control output to Serial for debugging.
 */
void PIDController::debug() const {
  Serial.print("P: "); Serial.print(Kp * last_error, 3);
  Serial.print(" | I: "); Serial.print(Ki * integral, 3);
  Serial.print(" | D: "); Serial.print(Kd * last_derivative, 3);
  Serial.print(" | OUT: "); Serial.println(last_output, 3);
}
