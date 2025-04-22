#include "config.h"
#ifndef CONTROLER // include guard
#define CONTROLER

class Controller {
    public:
      /// @param kp Proportional gain
      /// @param ki Integral gain
      /// @param kd Derivative gain
      /// @param output_limit If >0, clamp |output| ≤ this; else no clamp
      Controller(float kp, float ki, float kd, float output_limit = 0.0f);
    
      /// Compute the control output
      /// @param setpoint Desired value
      /// @param measurement Current value
      float compute(float setpoint, float measurement);
    
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





