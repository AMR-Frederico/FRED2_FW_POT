#include <Arduino.h>
#include <Main_Lib/config.h>
#include <Main_Back/power.h>

// motor motor1(BM1_IN1, BM1_IN2, BM1_PWM, 0); // Motor 1 instance
// motor motor2(BM2_IN1, BM2_IN2, BM2_PWM, 1); // Motor 2 instance

void setup(){

  write_PWM(motor1, 300);
  write_PWM(motor2, 300); 

}

void loop(){

}