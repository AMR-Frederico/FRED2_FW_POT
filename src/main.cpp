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

#include <controller.hpp>

RosData ros_data; // Instance of our RosData struct

Encoder encoder(34, 35, 36,39);   // Encoder pins

MedianFilter encoder_right_filter(33, 0); 
MedianFilter encoder_left_filter(33, 0); 


PIDConfig pid_config = PIDConfig(1.0f, 0.0f, 0.0f, 350.0f); 
Controller controller(pid_config);



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
  
  

  // -------------------------------------------------------
  // ROS data
  // -------------------------------------------------------

  RightAndLeftValues<float> pwm_cmd = controller.get_pwm_cmd();
  ros_data.controlled_pwm_left = pwm_cmd.left;
  ros_data.controlled_pwm_right = pwm_cmd.right;
  ros_data.ticks_encoder_right = encoders_data.right.ticks; 
  ros_data.ticks_encoder_left = encoders_data.left.ticks; 

  ros_loop(ros_data);
  ros_spin();

}
