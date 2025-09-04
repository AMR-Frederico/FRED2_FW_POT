#ifndef PID_CONFIG_HPP
#define PID_CONFIG_HPP

class PIDConfig{
    public:
        const float kp, ki, kd;
        float output_limit = 0.0f;
        PIDConfig(const float& kp, const float& ki, const float& kd, const float& output_limit = 350.0f): kp(kp), ki(ki), kd(kd), output_limit(output_limit) 
        {

        }
};

#endif