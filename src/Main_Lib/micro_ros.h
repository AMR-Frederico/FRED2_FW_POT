#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <custom_message/msg/encoder.h>

// Structure to group all the necessary data
struct RosData {
    
    double ticks_encoder_left;
    double ticks_encoder_right;
    // double angle_encoder_left;
    // double angle_encoder_right;
    // double rpm_encoder_left;
    // double rpm_encoder_right;

    // float  linear_vel_wheel_left; 
    // float  linear_vel_wheel_right; 
    // float  left_rad_cmd;
    // float  right_rad_cmd;  
    // float  left_rpm_cmd;
    // float  right_rpm_cmd;
    // float  controlled_rpm_left;
    // float  controlled_rpm_right;
    float  controlled_pwm_left;
    float  controlled_pwm_right;

};

// Function declarations
void init_ros(const char* node_name, const char* node_namespace);
void ros_loop(const RosData& data);
void ros_spin();

// Utility functions for getting velocities
float getLinear();
float getAngular();
