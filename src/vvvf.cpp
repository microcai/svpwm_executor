
#include <arm_math.h>
#include "vvvf.hpp"

VVVF::VVVF(motorlib::pwmdriver* driver, current_sense* cs)
    : m_driver(driver)
    , m_cs(cs)
{
    m_driver->link_timer([](int pwm_freq, int perids, void* user_data){
        reinterpret_cast<VVVF*>(user_data)->pwm_callback(pwm_freq, perids);
    }, this);
}

constexpr float Deg2Rad = 3.14159265358979323846/180;
constexpr float TWO_PI = 3.14159265358979323846 * 2;

extern float sin_of_degree(float degree);

void VVVF::set_foc(float shaft_angle, float Uout)
{
    using float_number = float;
    int sector = (int)shaft_angle / 60;

    static const float_number sector_angle[]
        = { float_number(0), float_number(60), float_number(120), float_number(180), float_number(240), float_number(300), float_number(360) };

    auto angle_in_sector = shaft_angle - sector_angle[sector];

    float_number T1 = Uout * sin_of_degree(60 - angle_in_sector);
    float_number T2 = Uout * sin_of_degree(angle_in_sector);
    float_number T0 = 1.0f - T1 - T2;

    float_number Ta, Tb, Tc;
    switch(sector)
    {
    case 0:
        Ta = T1 + T2 + T0/2;
        Tb = T2 + T0/2;
        Tc = T0/2;
        break;
    case 1:
        Ta = T1 +  T0/2;
        Tb = T1 + T2 + T0/2;
        Tc = T0/2;
        break;
    case 2:
        Ta = T0/2;
        Tb = T1 + T2 + T0/2;
        Tc = T2 + T0/2;
        break;
    case 3:
        Ta = T0/2;
        Tb = T1+ T0/2;
        Tc = T1 + T2 + T0/2;
        break;
    case 4:
        Ta = T2 + T0/2;
        Tb = T0/2;
        Tc = T1 + T2 + T0/2;
        break;
    case 5:
        Ta = T1 + T2 + T0/2;
        Tb = T0/2;
        Tc = T1 + T0/2;
        break;
    default:
        // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    output_A = Ta;
    output_B = Tb;
    output_C = Tc;

    m_driver->set_duty(Ta, Tb, Tc);

    // char buf[64];
    // int len = snprintf(buf, 64, "duty = %d, %d, %d\r\n", (int)(Ua*100), (int) (Ub * 100),(int)( Uc * 100));

    // 	SEGGER_RTT_Write(0, buf, len);
}

void VVVF::loop()
{
    if (m_driver->break_status == 1)
    {
        if (break_ttl == 0)
        {
            m_driver->stop();
            break_ttl = 1000;
        }
        // wait for some minit to re-start();
        m_driver->break_status = 0;
    }

    if (break_ttl)
    {
        if ( --break_ttl ==0)
        {
            m_driver->start();
        }
    }

    // estimate slip rate
}

void VVVF::pwm_callback(int pwm_freq, int perids)
{
    // 在中断里，根据 freq 和 perids 计算经过的时间
    // 然后根据时间和频率，决定新的相位
    float passed_time = (float) perids / (float) pwm_freq;

    auto current = m_cs->get_current(cur_angle, duty_with_current_limit);

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
        if (output_duty >= 0)
            duty_with_current_limit = std::min(duty_with_current_limit, output_duty);
        else
            duty_with_current_limit = std::max(duty_with_current_limit, output_duty);
    }


    cur_angle += passed_time * output_freq * 360;

    // normalize angle
    while (cur_angle >= 360)
        cur_angle -= 360;
    while (cur_angle < 0)
        cur_angle += 360;

    set_foc(cur_angle, duty_with_current_limit);
}

void VVVF::set_v_and_f(float V, float F)
{
    this->output_freq = F;
    this->output_duty = std::min(V, 1.0f);
}
