#if !defined(TWIST_HPP)
#define TWIST_HPP

class Twist{

public:
    float linear, angular;    
    Twist(){};
    Twist(float linear_vel, float angular_vel): linear(linear_vel), angular(angular_vel){};

};


#endif // TWIST_HPP
