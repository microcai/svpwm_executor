
#include "BLDC.hpp"
#include <math.h>

extern float sin_of_degree(float degree);

BLDC::BLDC(motorlib::pwmdriver* driver, hall_sensor * _hall)
    : m_driver(driver)
    , m_hall(_hall)
{
    m_driver->link_timer([](int pwm_freq, int perids, void* user_data){
        reinterpret_cast<BLDC*>(user_data)->pwm_callback(pwm_freq, perids);
    }, this);
}

void BLDC::set_duty(float U_a, float U_b, float U_c)
{
    if (!direct_control_mode)
    {
        direct_control_mode = true;
        output_duty = 0.0;
    }
    m_driver->set_duty(U_a, U_b, U_c);
}

void BLDC::set_duty(float _output_duty)
{
    if (direct_control_mode)
    {
        m_driver->set_duty(-1, -1, -1);
        direct_control_mode = false;
    }
    output_duty = _output_duty;
}

void BLDC::set_foc(float electron_angle_, float Uout)
{

    using float_number = float;

    // 使用 SVPWM 的方式产生 输出
    // find the sector we are in currently
    int sector = static_cast<int>(electron_angle_ / 120);
    static const float_number sector_angle[]
        = { float_number(0), float_number(120), float_number(240), float_number(360) };

    auto angle_in_sector = electron_angle_ - sector_angle[sector];

    float_number T1 = Uout * sin_of_degree(120 - angle_in_sector);
    float_number T2 = Uout * sin_of_degree(angle_in_sector);

    float_number Tmax;

    if (angle_in_sector > 60)
        Tmax = T2;
    else
        Tmax = T1;

    float_number center = (float_number{ 1 } - Tmax) / 2;

    float_number U_a, U_b, U_c;
    switch (sector)
    {
        case 0:
            U_a = T2;
            U_b = 0;
            U_c = T1;
            break;
        case 1:
            U_a = T1;
            U_b = T2;
            U_c = 0;
            break;
        case 2:
            U_a = 0;
            U_b = T1;
            U_c = T2;
            break;
        default:
            // possible error state
            U_a = 0.0f;
            U_b = 0.0f;
            U_c = 0.0f;
    }

    U_a += center;
    U_b += center;
    U_c += center;

    m_driver->set_duty(U_c, U_b, U_a);
}

void BLDC::pwm_callback(int pwm_freq, int perids)
{
    if (direct_control_mode)
        return;
    // update modulation based on hall state and duty
    auto step = m_hall->get_sector();
    int electron_angle_;

    auto Uout = std::abs(output_duty);

    if (Uout < 0.01)
    {
        m_driver->set_duty(-1.0f, -1.0f, -1.0f);
        return;
    }
    else if (output_duty > 0)
    {
        electron_angle_ = step*60 + 90;
        if (electron_angle_ >= 360)
            electron_angle_ -= 360;
    }
    else
    {
        electron_angle_ = step*60 - 90;
        if (electron_angle_ < 0)
            electron_angle_ += 360;
    }

    set_foc(electron_angle_, Uout);
}
