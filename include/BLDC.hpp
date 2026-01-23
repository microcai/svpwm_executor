
#pragma once


#include "pwmdriver.hpp"
#include <cstdint>
#include "hall_sensor.hpp"

class BLDC
{
public:
    BLDC(motorlib::pwmdriver* driver, hall_sensor * _hall);

    void pwm_callback(int, int);

    void set_duty(float U_a, float U_b, float U_c);

    void set_foc(float angle, float duty);

    void set_duty(float);

    motorlib::pwmdriver* m_driver;
    hall_sensor* m_hall;

    float cur_angle = 0;
    float output_duty = 0;

    bool direct_control_mode = true;
};
