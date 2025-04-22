
#include <Main_Lib/motor.h>
#include <Main_Lib/tools.h>
#include <Main_Lib/config.h>

// fred(linear(m/s),angular(rad/s)) -> |cinematic| -> wheel(angular(rad/s)) -> |angular2rpm| -> wheel(angular(rpm)) -> |rpm2pwm| -> wheel(pwm)

//@vel PWM signal between 0 and 1023 
// postive our negative representes de direction, positive been forward

motor motor1(FM1_IN1, FM1_IN2, FM1_PWM, 0);
motor motor2(FM2_IN1, FM2_IN2, FM2_PWM, 1);


//Global 

int pwm_right = 0 ;
int pwm_left = 0  ; 
int pwm_motor = 0;


// -------------------------------------------------------
// Stop motors function
// -------------------------------------------------------

void stop(motor motor){
  digitalWrite(motor.In_A, HIGH);  
  digitalWrite(motor.In_B, HIGH);
  ledcWrite(motor.Canal ,0);
}



// -------------------------------------------------------
// Write the PWM command at the motor driver
// -------------------------------------------------------

void write_PWM(motor motor, int vel){

  //satured output

  if(vel>=SATURATION){
    vel = SATURATION;
  }
  if(vel<= -SATURATION){
    vel = -SATURATION;
  }

  ledcWrite(motor.Canal ,abs(vel));
  digitalWrite(motor.In_B, vel < 0);
  digitalWrite(motor.In_A, vel > 0);  
}


// -------------------------------------------------------
// Converts m/s to rad/s
// -------------------------------------------------------

float meters2rad(float linear_vel){

  float cmd_rad = linear_vel/WHEEL_RADIUS; 

  return cmd_rad; 
}


// -------------------------------------------------------
// Converts rad/s to rpm
// -------------------------------------------------------

float rad2rpm(float angular_vel){
  //convert from angular w to rpm 
  
  float rpm_cmd_vel = (angular_vel*60.0f)/(2.0f*PI)  ;

  return rpm_cmd_vel;
}


// -------------------------------------------------------
// Converts RPM to PWM
// -------------------------------------------------------

int rpm2pwm(float rpm){

  // Handle direction separately
  bool reverse = false;
  if (rpm < 0) {
      reverse = true;
      rpm = -rpm;
  }

  // Deadband: if rpm is too small, just return 0
  if (rpm <= DEADBAND_RPM) return 0;

  // Clamp to max RPM
  rpm = fmin(rpm, MAX_RPM);

  // Apply nonlinear scaling
  float scaled = pow(rpm / MAX_RPM, NONLINEARITY_EXPONENT);

  // Map to PWM range
  int pwm = static_cast<int>(round(scaled * MAX_PWM));

  // Ensure within valid PWM bounds
  pwm = constrain(pwm, 0, MAX_PWM);

  // Return signed PWM (positive/negative indicates direction)
  return reverse ? -pwm : pwm;
  
}









// -------------------------------------------------------
// Receives the RPM command to sent PWM for both motors 
// -------------------------------------------------------

void write2motors(int pwm_left, int pwm_right) {

  write_PWM(motor1, pwm_right); 
  write_PWM(motor2, pwm_left); 
}




// -------------------------------------------------------
// Receives the RPM command to sent PWM for a single motor 
// -------------------------------------------------------

void write2motor(int rpm,int motor){
  pwm_motor = rpm2pwm(rpm);

  switch (motor)  {
    case 1:
       write_PWM(motor1,pwm_motor);
        break;

    case 2:
       write_PWM(motor2,pwm_motor);
        break;
    default:
          write_PWM(motor1,pwm_motor);
  }

}




