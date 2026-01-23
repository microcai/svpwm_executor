
#include "BLDC.hpp"

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

void BLDC::set_foc(float angle, float duty)
{

}

void BLDC::pwm_callback(int pwm_freq, int perids)
{
    if (direct_control_mode)
        return;
    // update modulation based on hall state and duty
    auto step = m_hall->get_sector();

    // 小于 0 处理成反转.
    auto _duty = std::abs(output_duty);

    if (output_duty >= 0)
    {
        switch (step)
        {
            case 0:
                m_driver->set_duty(_duty, 0, -1);
                break;
            case 1:
                m_driver->set_duty(-1, 0, _duty);
                break;
            case 2:
                m_driver->set_duty(0, -1, _duty);
                break;
            case 3:
                m_driver->set_duty(0, _duty, -1);
                break;
            case 4:
                m_driver->set_duty(-1, _duty, 0);
                break;
            default: 
                m_driver->set_duty(_duty, -1, 0);
                break;
        }
    }
    else
    {
        switch (step)
        {
            case 3:
                m_driver->set_duty(_duty, 0, -1);
                break;
            case 4:
                m_driver->set_duty(-1, 0, _duty);
                break;
            case 5:
                m_driver->set_duty(0, -1, _duty);
                break;
            case 0:
                m_driver->set_duty(0, _duty, -1);
                break;
            case 1:
                m_driver->set_duty(-1, _duty, 0);
                break;
            default:
                m_driver->set_duty(_duty, -1, 0);
                break;
        }
    }
}
