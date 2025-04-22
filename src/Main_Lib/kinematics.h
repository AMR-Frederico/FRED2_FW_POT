#include <Main_Lib/config.h>

// Global variables
float cmd_rpm;   // Global command RPM setpoint
bool debug = false; // Debug mode flag

// -------------------------------------------------------
// Kinematics debug control
// -------------------------------------------------------

/**
 * @brief Returns the current debug mode status.
 * 
 * @return true if debug mode is enabled, false otherwise.
 */
bool debugControl() {
  return debug;
}

// -------------------------------------------------------
// Get RPM setpoint
// -------------------------------------------------------

/**
 * @brief Returns the current RPM setpoint.
 * 
 * @return Current command RPM.
 */
float getRPMsetpoint() {
  return cmd_rpm;
}

// -------------------------------------------------------
// Calculate left wheel angular velocity
// -------------------------------------------------------

/**
 * @brief Calculate the left wheel speed based on robot kinematics.
 * 
 * @param linear  Linear velocity (m/s).
 * @param angular Angular velocity (rad/s).
 * @param gain    Gain factor (e.g., for calibration or scaling).
 * @return Left wheel angular velocity (rad/s).
 */
float kinematics_left(float linear, float angular, float gain) {
  return gain * (linear - angular * WHEELS_SEPARATION / 2.0);
}

// -------------------------------------------------------
// Calculate right wheel angular velocity
// -------------------------------------------------------

/**
 * @brief Calculate the right wheel speed based on robot kinematics.
 * 
 * @param linear  Linear velocity (m/s).
 * @param angular Angular velocity (rad/s).
 * @param gain    Gain factor (e.g., for calibration or scaling).
 * @return Right wheel angular velocity (rad/s).
 */
float kinematics_right(float linear, float angular, float gain) {
  return gain * (linear + angular * WHEELS_SEPARATION / 2.0);
}
