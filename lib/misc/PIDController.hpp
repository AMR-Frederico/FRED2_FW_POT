#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController{
    public:
        const float kp, ki, kd;
        PIDController(const float& kp, const float& ki, const float& kd): kp(kp), ki(ki), kd(kd) 
        {

        }
};


#endif