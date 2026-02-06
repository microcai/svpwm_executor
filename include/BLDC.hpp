
#pragma once


#include "current_sense.hpp"
#include "pwmdriver.hpp"
#include <cstdint>
#include "hall_sensor.hpp"
#include "pll.hpp"

class BLDC
{
public:
    BLDC(motorlib::pwmdriver* driver, hall_sensor * _hall, PLL* angle_pll, current_sense*);

    void pwm_callback(int, int);

    void set_duty(float U_a, float U_b, float U_c);

    void set_foc(float angle, float duty);
    void set_6step(float angle, float duty);

    void set_duty(float);

    motorlib::pwmdriver* m_driver;
    hall_sensor* m_hall;
    PLL* m_angle_pll;
    current_sense* m_cs;

    float InputCurrentLimit = 1;
    float currentLimit = 15;
    float cur_angle = 0;
    float output_duty = 0;
    float hw_duty = 0;

    float duty_with_current_limit = 0;

    bool direct_control_mode = true;
};
