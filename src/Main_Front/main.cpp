#include <Arduino.h>
#include <Main_Lib/encoder.h>
#include <Main_Lib/MedianFilter.h>
#include <Main_Lib/micro_ros.h>
#include <Main_Lib/kinematics.hpp>
#include "power.h"
#include <Main_Lib/pid_controller.hpp>
#include <PIDConfig.hpp>
#include <Twist.hpp>

RosData ros_data; // Instance of our RosData struct

Encoder encoder(34, 35, 36,39);   // Encoder pins

MedianFilter encoder_right_filter(33, 0); 
MedianFilter encoder_left_filter(33, 0); 


PIDConfig pid_config = PIDConfig(1.0f, 0.0f, 0.0f); 
Controller left_wheel(pid_config, 350.0f); 
Controller right_wheel(pid_config, 350.0f); 


Twist get_vel_from_ros(void){
  
  float linear_vel =  getLinear(); 
  float angular_vel =  getAngular(); 
  return Twist(linear_vel, angular_vel);

}

void setup() {
  // Serial.begin(115200);

  // -------------------------------------------------------
  // Microros and encoder initialization 
  // -------------------------------------------------------
  init_ros("fred2_fw_motors", "front");

  encoder.setup();
}

void loop() {

  // -------------------------------------------------------
  // Get cmd_vel from ROS
  // -------------------------------------------------------
  // float robot_linear_vel =  getLinear(); 
  // float robot_angular_vel =  getAngular();
  Twist robot_vel = get_vel_from_ros();


  // -------------------------------------------------------
  // Get data from encoder 
  // -------------------------------------------------------
  double angle_encoder_left = encoder.readAngle(LEFT);
  double rpm_encoder_left =  encoder.readRPM(LEFT);
  double ticks_encoder_left = encoder.readPulses(LEFT);

  double angle_encoder_right = encoder.readAngle(RIGHT); 
  double rpm_encoder_right = encoder.readRPM(RIGHT);
  double ticks_encoder_right = encoder.readPulses(RIGHT);


  // -------------------------------------------------------
  // Median filter to throw outliers 
  // -------------------------------------------------------
  //TODO: test the behavior of the median filter
  // encoder_left_filter.in(rpm_encoder_left); 
  // rpm_encoder_left = encoder_left_filter.out(); 
  
  // encoder_right_filter.in(rpm_encoder_right); 
  // rpm_encoder_right = encoder_right_filter.out(); 



  // -------------------------------------------------------
  // Robot kinematics  - in m/s
  // -------------------------------------------------------  
  float left_wheel_vel = Kinematics::kinematics_left(robot_vel, 1.0); 
  float right_wheel_vel = Kinematics::kinematics_right(robot_vel, 1.0); 

  
  
  // -------------------------------------------------------
  // Robot kinematics  - conversion from m/s to rad/s 
  // -------------------------------------------------------
  float left_rad_cmd = meters2rad(left_wheel_vel); 
  float right_rad_cmd = meters2rad(right_wheel_vel); 

  
  


  // -------------------------------------------------------
  // Robot kinematics  - conversion from rad/s to rpm 
  // -------------------------------------------------------
  float left_rpm_cmd = rad2rpm(left_rad_cmd); 
  float right_rpm_cmd = rad2rpm(right_rad_cmd); 

  
  

  // -------------------------------------------------------
  // PID control for the wheels velocities
  // -------------------------------------------------------
  float controlled_RPM_left = left_wheel.compute(left_rpm_cmd, rpm_encoder_left); 
  float controlled_RPM_right = right_wheel.compute(right_rpm_cmd, rpm_encoder_right); 

  
  
  
  
  // -------------------------------------------------------
  // Robot knimatics - conversion rpm to pwm 
  // -------------------------------------------------------
  float left_pwm_cmd = rpm2pwm(controlled_RPM_left); 
  float right_pwm_cmd = rpm2pwm(controlled_RPM_right); 



  // -------------------------------------------------------
  // Send the comands to the motors
  // -------------------------------------------------------
  write2motors(int(left_pwm_cmd), int(right_pwm_cmd)); 


  

  // Serial.println("======= Kinematics Debug =======");

  // Serial.print("Lin vel (m/s) L: ");
  // Serial.print(left_wheel_vel, 4);
  // Serial.print(" | R: ");
  // Serial.print(right_wheel_vel, 4);
  
  // Serial.print(" || Ang vel (rad/s) L: ");
  // Serial.print(left_rad_cmd, 4);
  // Serial.print(" | R: ");
  // Serial.print(right_rad_cmd, 4);
  
  // Serial.print(" || Setpoint RPM L: ");
  // Serial.print(left_rpm_cmd, 2);
  // Serial.print(" | R: ");
  // Serial.print(right_rpm_cmd, 2);
  
  // Serial.print(" || Controlled RPM L: ");
  // Serial.print(controlled_RPM_left, 2);
  // Serial.print(" | R: ");
  // Serial.print(controlled_RPM_right, 2);
  
  // Serial.print(" || PWM L: ");
  // Serial.print(left_pwm_cmd, 0);
  // Serial.print(" | R: ");
  // Serial.println(right_pwm_cmd, 0);
  
  // Serial.println("================================");
  // Serial.println();
  

  // delay(1000);
  



  // -------------------------------------------------------
  // ROS data
  // -------------------------------------------------------

  // left wheel 
  // ros_data.ticks_encoder_left = ticks_encoder_left;
  // ros_data.angle_encoder_left = angle_encoder_left;
  // ros_data.rpm_encoder_left = rpm_encoder_left;
  // ros_data.linear_vel_wheel_left = left_wheel_vel;
  // ros_data.left_rad_cmd = left_rad_cmd;
  // ros_data.left_rpm_cmd = left_rpm_cmd; 
  // ros_data.controlled_rpm_left = controlled_RPM_left;
  ros_data.controlled_pwm_left = left_pwm_cmd;

  // right wheel 
  ros_data.ticks_encoder_right = ticks_encoder_right; 
  // ros_data.angle_encoder_right = angle_encoder_right; 
  // ros_data.rpm_encoder_right = rpm_encoder_right; 
  // ros_data.linear_vel_wheel_right = right_wheel_vel;
  // ros_data.right_rad_cmd = right_rad_cmd; 
  // ros_data.right_rpm_cmd = right_rpm_cmd;
  // ros_data.controlled_rpm_right = controlled_RPM_right; 
  ros_data.controlled_pwm_right = right_pwm_cmd;

  // Publish the updated data to ROS topics
  ros_loop(ros_data);

  // Run the micro-ROS executor to process incoming messages
  ros_spin();
}
