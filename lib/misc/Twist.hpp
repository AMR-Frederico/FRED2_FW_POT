#if !defined(TWIST_HPP)
#define TWIST_HPP

class Twist{

public:
    float linear_vel, angular_vel;    
    Twist(){};
    Twist(float linear_vel, float angular_vel): linear_vel(linear_vel), angular_vel(angular_vel){};

};


#endif // TWIST_HPP
