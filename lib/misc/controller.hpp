#if !defined(CONTROLLER_HPP)
#define CONTROLLER_HPP

#include <Twist.hpp>
#include <RightAndLeftValues.hpp>
#include <kinematics.hpp>
#include "encoder.hpp"
#include "power.hpp"
#include <PIDConfig.hpp>
#include <pid_controller.hpp>

class Controller{

private:
    RightAndLeftValues<float> pwm_cmd;

public:
    PIDController left_wheel_pid;
    PIDController right_wheel_pid;
    
    Controller(const PIDConfig& pid_config){
        left_wheel_pid = PIDController(pid_config); 
        right_wheel_pid = PIDController(pid_config); 
    }

    

    float meters2rad(float linear_vel) {
        float cmd_rad = linear_vel / WHEEL_RADIUS;
        return cmd_rad;
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

        float controlled_RPM_right = right_wheel_pid.compute_pid_control(rpm_cmd.right, encoder_data.right.rpm); 
        float controlled_RPM_left = left_wheel_pid.compute_pid_control(rpm_cmd.left, encoder_data.left.rpm); 
        return RightAndLeftValues<float>{controlled_RPM_left, controlled_RPM_right};

    }

    RightAndLeftValues<float> get_pwm_control(RightAndLeftValues<float>& cmd_rpm_target){


        float right_pwm_cmd = rpm2pwm(cmd_rpm_target.right); 
        float left_pwm_cmd = rpm2pwm(cmd_rpm_target.left); 

        return RightAndLeftValues<float>{right_pwm_cmd, left_pwm_cmd};

    }
    
    void Control(Twist& robot_vel, RightAndLeftValues<EncoderData>& encoders_data){

        RightAndLeftValues<float> wheel_vel = twist_to_wheel_vel_ms(robot_vel);
        RightAndLeftValues<float> rad_cmd = wheel_vel_ms_to_rad_cmd(wheel_vel);
        RightAndLeftValues<float> rpm_cmd = rad_cmd_to_rpm_cmd(rad_cmd);
        RightAndLeftValues<float> rpm_cmd_target = get_target_rpm_cmd(rpm_cmd, encoders_data);
        pwm_cmd = get_pwm_control(rpm_cmd_target);
        write2motors(int(pwm_cmd.left), int(pwm_cmd.right)); 

    }

    RightAndLeftValues<float> get_pwm_cmd(void){ return pwm_cmd;}

};

#endif // CONTROLLER_HPP
