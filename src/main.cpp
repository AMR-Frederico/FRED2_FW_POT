#include <Arduino.h>
#include <encoder.hpp>
#include <MedianFilter.h>
#include <micro_ros.h>
#include <kinematics.hpp>
#include "power.hpp"
#include <pid_controller.hpp>
#include <PIDConfig.hpp>
#include <Twist.hpp>
#include <RightAndLeftValues.hpp>

RosData ros_data; // Instance of our RosData struct

Encoder encoder(34, 35, 36,39);   // Encoder pins

MedianFilter encoder_right_filter(33, 0); 
MedianFilter encoder_left_filter(33, 0); 


PIDConfig pid_config = PIDConfig(1.0f, 0.0f, 0.0f); 
PIDController left_wheel(pid_config, 350.0f); 
PIDController right_wheel(pid_config, 350.0f); 



Twist get_vel_from_ros(void){
  
  float linear_vel =  getLinear(); 
  float angular_vel =  getAngular(); 
  return Twist(linear_vel, angular_vel);

}

RightAndLeftValues<EncoderData> get_encoders_data(Encoder& encoder){

  EncoderData encoder_right = encoder.get_encoder_data(RIGHT);
  EncoderData encoder_left = encoder.get_encoder_data(LEFT);
  return RightAndLeftValues<EncoderData>{encoder_right, encoder_left};

}

RightAndLeftValues<float> twist_to_wheel_vel_ms(Twist& vel){

  float left_wheel_vel = Kinematics::kinematics_left(vel, 1.0); 
  float right_wheel_vel = Kinematics::kinematics_right(vel, 1.0); 
  return RightAndLeftValues<float>{right_wheel_vel, left_wheel_vel};

}

RightAndLeftValues<float> wheel_vel_ms_to_rad_cmd(RightAndLeftValues<float>& vel){

  float right_rad_cmd = meters2rad(vel.right); 
  float left_rad_cmd = meters2rad(vel.left); 
  return RightAndLeftValues<float>{right_rad_cmd, left_rad_cmd};

}

RightAndLeftValues<float> rad_cmd_to_rpm_cmd(RightAndLeftValues<float>& rad_cmd){

  float left_rpm_cmd = rad2rpm(rad_cmd.left); 
  float right_rpm_cmd = rad2rpm(rad_cmd.right); 
  return RightAndLeftValues<float>{right_rpm_cmd, left_rpm_cmd};

}

RightAndLeftValues<float> get_target_rpm_cmd(RightAndLeftValues<float>& rpm_cmd, RightAndLeftValues<EncoderData>& encoder_data){

  float controlled_RPM_right = right_wheel.compute_pid_control(rpm_cmd.right, encoder_data.right.rpm); 
  float controlled_RPM_left = left_wheel.compute_pid_control(rpm_cmd.left, encoder_data.left.rpm); 
  return RightAndLeftValues<float>{controlled_RPM_left, controlled_RPM_right};

}

RightAndLeftValues<float> get_pwm_control(RightAndLeftValues<float>& cmd_rpm_target){


  float right_pwm_cmd = rpm2pwm(cmd_rpm_target.right); 
  float left_pwm_cmd = rpm2pwm(cmd_rpm_target.left); 

  return RightAndLeftValues<float>{right_pwm_cmd, left_pwm_cmd};

}

void setup() {

  #ifdef FRONT_DRIVE
    init_ros("fred2_fw_motors", "front");
  
  #elif defined(BACK_DRIVE)
    init_ros("fred2_fw_motors", "back");

  #else
    #error "You must define BACK_DRIVE or FRONT_DRIVE"
  #endif
  
  encoder.setup();
}

void loop() {


  Twist robot_vel = get_vel_from_ros();
  RightAndLeftValues<EncoderData> encoders_data = get_encoders_data(encoder);
  RightAndLeftValues<float> wheel_vel = twist_to_wheel_vel_ms(robot_vel);
  RightAndLeftValues<float> rad_cmd = wheel_vel_ms_to_rad_cmd(wheel_vel);
  RightAndLeftValues<float> rpm_cmd = rad_cmd_to_rpm_cmd(rad_cmd);
  RightAndLeftValues<float> rpm_cmd_target = get_target_rpm_cmd(rpm_cmd, encoders_data);
  RightAndLeftValues<float> pwm_cmd = get_pwm_control(rpm_cmd_target);
  write2motors(int(pwm_cmd.left), int(pwm_cmd.right)); 

  // -------------------------------------------------------
  // ROS data
  // -------------------------------------------------------

  ros_data.controlled_pwm_left = pwm_cmd.left;
  ros_data.controlled_pwm_right = pwm_cmd.right;
  ros_data.ticks_encoder_right = encoders_data.right.ticks; 
  ros_data.ticks_encoder_left = encoders_data.left.ticks; 

  ros_loop(ros_data);
  ros_spin();

}
