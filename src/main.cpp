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
#include <Preferences.h>
#include <controller.hpp>
#include <memory>

#define CALIB_VAR "calibmode"

RosData ros_data; // Instance of our RosData struct

Encoder encoder(34, 35, 36,39);   // Encoder pins

MedianFilter encoder_right_filter(33, 0); 
MedianFilter encoder_left_filter(33, 0); 

Preferences prefs;
float kp, ki, kd;
std::unique_ptr<Controller> controller;
bool calib_mode = false;


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

  pinMode(0, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);
  

  prefs.begin("storage", false);
  calib_mode = prefs.getBool(CALIB_VAR, false);
  // force to embbebed electronics
  // calib_mode = true;
  kp = prefs.getFloat("kp", 1.0f);
  ki = prefs.getFloat("ki", 0.0f);
  kd = prefs.getFloat("kd", 0.0f);
  prefs.end();
  
  
  PIDConfig pid_config = PIDConfig(kp, ki, kd, 350.0f); 
  controller = std::unique_ptr<Controller>( new Controller(pid_config));
  

  #ifdef FRONT_DRIVE
    init_ros("fred2_fw_motors", "front", calib_mode);
  
  #elif defined(BACK_DRIVE)
    init_ros("fred2_fw_motors", "back", calib_mode);

  #else
    #error "You must define BACK_DRIVE or FRONT_DRIVE"
  #endif

  encoder.setup();
}

void handle_debug_switch(){
  if(digitalRead(0) == LOW){
    calib_mode = !calib_mode;
    prefs.begin("storage", false);
    prefs.putBool(CALIB_VAR, calib_mode);
    prefs.end();
    delay(500); // debounce
  }
}
int i = 0;
int ith = 100;
void loop() {
  i++;
  if(calib_mode)
  {
    
    digitalWrite(2, i > ith/2);
    i = i > ith? 0 : i;
  }
  handle_debug_switch();
  if(debug_mode){
    controller->
    PIDConfig new_pid = get_pid_config();
    controller->right_wheel_pid.update_pid(new_pid);
  }
  
  Twist robot_vel = get_vel_from_ros();
  RightAndLeftValues<EncoderData> encoders_data = get_encoders_data(encoder);
  controller->Control(robot_vel, encoders_data);
  
  

  // -------------------------------------------------------
  // ROS data
  // -------------------------------------------------------

  RightAndLeftValues<float> pwm_cmd = controller->get_pwm_cmd();
  ros_data.controlled_pwm_left = pwm_cmd.left;
  ros_data.controlled_pwm_right = pwm_cmd.right;
  ros_data.ticks_encoder_right = encoders_data.right.ticks; 
  ros_data.ticks_encoder_left = encoders_data.left.ticks; 

  ros_loop(ros_data);
  ros_spin();

}
