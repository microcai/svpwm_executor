
#include "BLDC.hpp"
#include <math.h>

extern float sin_of_degree(float degree);

BLDC::BLDC(motorlib::pwmdriver* driver, hall_sensor * _hall, PLL* angle_pll, current_sense* cs)
    : m_driver(driver)
    , m_hall(_hall)
    , m_angle_pll(angle_pll)
    , m_cs(cs)
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

void BLDC::set_6step(float electron_angle_, float Uout)
{
    cur_angle = electron_angle_;
    using float_number = float;

    // 使用 SVPWM 的方式产生 输出
    // find the sector we are in currently
    int sector = static_cast<int>(electron_angle_ / 60);

    m_driver->set_6step(sector, Uout);

}


void BLDC::set_foc(float electron_angle_, float Uout)
{
    cur_angle = electron_angle_;
    using float_number = float;

    // 使用 SVPWM 的方式产生 输出
    // find the sector we are in currently
    int sector = static_cast<int>(electron_angle_ / 60);
    static const float_number sector_angle[]
        = { float_number(0), float_number(60), float_number(120), float_number(180), float_number(240), float_number(300), float_number(360) };

    auto angle_in_sector = electron_angle_ - sector_angle[sector];

    float_number T1 = Uout * sin_of_degree(60 - angle_in_sector);
    float_number T2 = Uout * sin_of_degree(angle_in_sector);
    float_number T0 = 1 - T1 - T2;

    float_number U_a, U_b, U_c;
    switch (sector)
    {
        case 0:
            U_a = T0/2;
            U_b = T1 + T2 + T0/2;
            U_c = T1 + T0/2;
            break;
        case 1:
            U_a = T2 + T0/2;
            U_b = T1 + T2 + T0/2;
            U_c = T0/2;
            break;
        case 2:
            U_a = T1 + T2 + T0/2;
            U_b = T1 + T0/2;
            U_c = T0/2;
            break;
        case 3:
            U_a = T1 + T2 + T0/2;
            U_b = T0/2;
            U_c = T2 + T0/2;
        break;
        case 4:
            U_a = T1 + T0/2;
            U_b = T0/2;
            U_c = T1 + T2 + T0/2;
            break;
        case 5:
            U_a = T0/2;
            U_b = T2 + T0/2;
            U_c = T1 + T2 + T0/2;
            break;
        default:
            // possible error state
            U_a = 0.0f;
            U_b = 0.0f;
            U_c = 0.0f;
    }

    m_driver->set_duty(U_a, U_b, U_c);
}

void BLDC::pwm_callback(int pwm_freq, int perids)
{
    if (direct_control_mode)
    {
        set_6step(m_hall->get_sector(), 0);
        return;
    }

    float passed_time = (float) perids / (float) pwm_freq;

    auto tracked_angle = m_angle_pll->get_predict_phase(passed_time);

    if (!m_angle_pll->is_phase_locked())
        tracked_angle = m_hall->get_sector() * 60;

    // update modulation based on hall state and duty
    // auto step = m_hall->get_sector();
    int electron_angle_;

    // 获取电流采样，以便限制峰值电流
    auto current = m_cs->get_current(cur_angle, hw_duty);
    bool limit_reached = false;

    if (current.BusCurrent > InputCurrentLimit*1.2 )
    {
        duty_with_current_limit *= 0.5;
        limit_reached = true;
    }
    else if (current.BusCurrent > InputCurrentLimit )
    {
        duty_with_current_limit *= 0.95;
        limit_reached = true;
    }
    if (current.Current > currentLimit*1.2)
    {
        // 大过流立即大幅减小占空比
        limit_reached = true;
        duty_with_current_limit *= 0.6;
    }
    if (current.BusCurrent > InputCurrentLimit )
    {
        limit_reached = true;
        duty_with_current_limit *= 0.95;
    }
    else if (current.Current > currentLimit)
    {
        // 小过流稍稍减少占空比
        limit_reached = true;
        duty_with_current_limit *= 0.98;
    }
    
    if (!limit_reached)
    {
        // 占空比慢速跟踪用户的设定值.
        duty_with_current_limit = duty_with_current_limit * 0.95 + output_duty * 0.05;
    }

    hw_duty = std::abs(duty_with_current_limit);

    if (hw_duty < 0.01)
    { 
        m_driver->set_duty(-1.0f, -1.0f, -1.0f);
        return;
    }
    else if (output_duty > 0)
    {
        electron_angle_ = tracked_angle + 90;
        if (electron_angle_ >= 360)
            electron_angle_ -= 360;
    }
    else
    {
        electron_angle_ = tracked_angle - 90;
        if (electron_angle_ < 0)
            electron_angle_ += 360;
    }

    // set_foc(electron_angle_,hw_duty);
    set_6step(electron_angle_, hw_duty);
}
