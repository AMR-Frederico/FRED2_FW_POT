#include "config.h"
#ifndef PID_CONTROLLER_HPP // include guard
#define PID_CONTROLLER_HPP
#include <PIDConfig.hpp>
#include "Arduino.h"


class PIDController {
public:
  /// @param kp Proportional gain
  /// @param ki Integral gain
  /// @param kd Derivative gain
  /// @param output_limit If >0, clamp |output| ≤ this; else no clamp
  PIDController(const PIDConfig& pid_config);
  PIDController(){};
  void update_pid(const PIDConfig& pid_config);



  /// Compute the control output
  /// @param setpoint Desired value
  /// @param measurement Current value
  float compute_pid_control(float setpoint, float measurement);

  /// Print internal state over Serial (call after compute())
  void debug() const;

private:
  // Gains
  float Kp, Ki, Kd;

  // State
  float integral;
  float last_error;
  unsigned long last_time_ms;
  float last_derivative;


  // Limits
  float output_limit;

  // Last output (for debug)
  float last_output;

  // Helpers
  float clamp(float v, float abs_max) const {
    if (abs_max > 0 && v >  abs_max) return  abs_max;
    if (abs_max > 0 && v < -abs_max) return -abs_max;
    return v;
  }
};
    
#endif





