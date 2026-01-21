
#pragma once


#include "pwmdriver.hpp"

class VVVF
{
public:
    VVVF(motorlib::pwmdriver* driver);

    void set_v_and_f(float V, float F);

protected:
    void pwm_callback(int, int);

    motorlib::pwmdriver* m_driver;

public:
    float cur_angle = 0;
    float output_freq = 0;
    float output_duty = 0;
};
