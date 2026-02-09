
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
    Uout *= 0.57735;

    using float_number = float;

    float_number _park_sin, _park_cos;

    static constexpr float_number negtive_half{-0.5};
    static constexpr float_number _half{0.5};
    static constexpr float_number _SQRT3_2{0.86602540378443864676372317075294};

    arm_sin_cos_f32(shaft_angle, &_park_sin, &_park_cos);

    float_number Ualpha, Ubeta;

    arm_inv_park_f32(0, Uout, &Ualpha, &Ubeta, _park_sin, _park_cos);

    float_number Ua, Ub, Uc;

    // Inverse Clarke transform
    Ua = Ualpha;
    Uc = negtive_half * Ualpha - _SQRT3_2 * Ubeta;
    Ub = negtive_half * Ualpha + _SQRT3_2 * Ubeta;

    float_number center = _half;

    float_number Umin = std::min(Ua, std::min(Ub, Uc));
    float_number Umax = std::max(Ua, std::max(Ub, Uc));

    center -= (Umax+Umin) / 2;
    Ua += center;
    Ub += center;
    Uc += center;

    m_driver->set_duty(Ua, Ub, Uc);

		// char buf[64];
		// int len = snprintf(buf, 64, "duty = %d, %d, %d\r\n", (int)(Ua*100), (int) (Ub * 100),(int)( Uc * 100));

	  	// 	SEGGER_RTT_Write(0, buf, len);

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
    while (cur_angle > 360)
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
