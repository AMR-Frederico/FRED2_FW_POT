#ifndef POWER_HPP
#define POWER_HPP

#include <motor.h>
#include <tools.h>
#include <config.h>

// fred(linear(m/s),angular(rad/s)) -> |cinematic| -> wheel(angular(rad/s)) -> |angular2rpm| -> wheel(angular(rpm)) -> |rpm2pwm| -> wheel(pwm)

// @vel: PWM signal between 0 and 1023
// Positive or negative represents direction (positive = forward)

motor motor1(M1_IN1, M1_IN2, M1_PWM, 0); // Motor 1 instance
motor motor2(M2_IN1, M2_IN2, M2_PWM, 1); // Motor 2 instance

// Global variables
int pwm_right = 0;
int pwm_left = 0;
int pwm_motor = 0;

// -------------------------------------------------------
// Stop motors function
// -------------------------------------------------------

/**
 * @brief Stop the motor by setting both direction pins HIGH and PWM to 0.
 * 
 * @param motor Motor object to be stopped.
 */
void stop(motor motor) {
  digitalWrite(motor.In_A, HIGH);  
  digitalWrite(motor.In_B, HIGH);
  ledcWrite(motor.Canal, 0);
}

// -------------------------------------------------------
// Write the PWM command to the motor driver
// -------------------------------------------------------

/**
 * @brief Write a PWM signal to the motor driver, setting speed and direction.
 *        Values are saturated within [-SATURATION, +SATURATION].
 * 
 * @param motor Motor object to be controlled.
 * @param vel   PWM value (signed) to set speed and direction.
 */
void write_PWM(motor motor, int vel) {

  // Saturate output
  if (vel >= SATURATION) vel = SATURATION;
  if (vel <= -SATURATION) vel = -SATURATION;

  ledcWrite(motor.Canal, abs(vel));
  digitalWrite(motor.In_B, vel < 0);
  digitalWrite(motor.In_A, vel > 0);  
}

// -------------------------------------------------------
// Converts linear velocity (m/s) to angular velocity (rad/s)
// -------------------------------------------------------


// -------------------------------------------------------
// Converts angular velocity (rad/s) to RPM
// -------------------------------------------------------

/**
 * @brief Convert angular velocity (rad/s) into wheel rotational speed (RPM).
 * 
 * @param angular_vel Angular velocity (radians per second).
 * @return Rotational speed (RPM).
 */
float rad2rpm(float angular_vel) {
  float rpm_cmd_vel = (angular_vel * 60.0f) / (2.0f * PI);
  return rpm_cmd_vel;
}

// -------------------------------------------------------
// Converts RPM to PWM
// -------------------------------------------------------

/**
 * @brief Convert a RPM command into a PWM signal.
 *        Handles deadband, direction, and nonlinear scaling.
 * 
 * @param rpm Rotational speed (signed RPM).
 * @return PWM value (signed).
 */
int rpm2pwm(float rpm) {

  // Handle direction separately
  bool reverse = false;
  if (rpm < 0) {
    reverse = true;
    rpm = -rpm;
  }

  // Deadband: if RPM is too small, return 0
  if (rpm <= DEADBAND_RPM) return 0;

  // Clamp to max RPM
  rpm = fmin(rpm, MAX_RPM);

  // Apply nonlinear scaling
  float scaled = pow(rpm / MAX_RPM, NONLINEARITY_EXPONENT);

  // Map to PWM range
  int pwm = static_cast<int>(round(scaled * MAX_PWM));

  // Ensure PWM is within valid bounds
  pwm = constrain(pwm, 0, MAX_PWM);

  // Return signed PWM based on original direction
  return reverse ? -pwm : pwm;
}

// -------------------------------------------------------
// Send PWM commands to both motors
// -------------------------------------------------------

/**
 * @brief Write PWM commands to both motors simultaneously.
 * 
 * @param pwm_left  PWM value for left motor.
 * @param pwm_right PWM value for right motor.
 */
void write2motors(int pwm_left, int pwm_right) {
  write_PWM(motor1, pwm_right);
  write_PWM(motor2, pwm_left);
}

// -------------------------------------------------------
// Send PWM command to a single motor
// -------------------------------------------------------

/**
 * @brief Write a RPM command to a single motor.
 * 
 * @param rpm   RPM command to be sent.
 * @param motor Motor selector (1 = motor1, 2 = motor2).
 */
void write2motor(int rpm, int motor) {
  pwm_motor = rpm2pwm(rpm);

  switch (motor) {
    case 1:
      write_PWM(motor1, pwm_motor);
      break;
    case 2:
      write_PWM(motor2, pwm_motor);
      break;
    default:
      write_PWM(motor1, pwm_motor); // Default to motor1 if invalid input
  }
}


#endif //POWER_HPP