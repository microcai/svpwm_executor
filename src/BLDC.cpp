
#include "BLDC.hpp"
#include "SEGGER_RTT.h"
#include "cyccounter.hpp"
#include "dsp/controller_functions.h"
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
    using float_number = float;

    // 使用 SVPWM 的方式产生 输出
    // find the sector we are in currently
    int sector = static_cast<int>(electron_angle_ / 60);

    m_driver->set_6step(sector, Uout);

}


void BLDC::set_foc(float shaft_angle, float Uout)
{
    Uout *= 0.57735;
    cur_angle = shaft_angle;

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

void BLDC::pwm_callback(int pwm_freq, int perids)
{
    if (direct_control_mode)
    {
        return;
    }

    float passed_time = (float) perids / (float) pwm_freq;
    auto tracked_angle = m_angle_pll->get_predict_phase(passed_time);

    if (!m_angle_pll->is_phase_locked())
        tracked_angle = m_hall->get_sector() * 60;

    // update modulation based on hall state and duty
    // auto step = m_hall->get_sector();
    cyc_counter<int, 0, 360> electron_angle_;

    // 获取电流采样，以便限制峰值电流
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

    hw_duty = std::abs(duty_with_current_limit);

    // set_foc(tracked_angle,duty_with_current_limit);
    // return;

    electron_angle_ = tracked_angle;

    if (output_duty > 0)
    {
        electron_angle_ += 90;
    }
    else
    {
        electron_angle_ -= 90;
    }
    cur_angle = tracked_angle;

    set_6step(electron_angle_.get_under_value(), hw_duty);
}
