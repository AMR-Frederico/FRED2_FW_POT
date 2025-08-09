#ifndef PID_CONFIG_HPP
#define PID_CONFIG_HPP

class PIDConfig{
    public:
        const float kp, ki, kd;
        PIDConfig(const float& kp, const float& ki, const float& kd): kp(kp), ki(ki), kd(kd) 
        {

        }
};


#endif