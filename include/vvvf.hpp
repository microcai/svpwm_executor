
#pragma once


#include "pwmdriver.hpp"
#include "current_sense.hpp"

class VVVF
{
public:
    VVVF(motorlib::pwmdriver* driver, current_sense*);

    void set_v_and_f(float V, float F);

    // set pwm [-1, 1], let system to do slip control to determine the freq
    void set_duty(float duty);

protected:
    void pwm_callback(int, int);
    void set_foc(float shaft_angle, float Uout);

    motorlib::pwmdriver* m_driver;
    current_sense* m_cs;
    float duty_with_current_limit = 1;
public:
    float currentLimit = 5;
    float InputCurrentLimit = 3;

    float cur_angle = 0;
    float output_freq = 0;
    float output_duty = 0;
};
