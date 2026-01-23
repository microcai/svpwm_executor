
#include <arm_math.h>
#include "vvvf.hpp"

VVVF::VVVF(motorlib::pwmdriver* driver)
    : m_driver(driver)
{
    m_driver->link_timer([](int pwm_freq, int perids, void* user_data){
        reinterpret_cast<VVVF*>(user_data)->pwm_callback(pwm_freq, perids);
    }, this);
}

constexpr float Deg2Rad = 3.14159265358979323846/180;
constexpr float TWO_PI = 3.14159265358979323846 * 2;

extern float sin_of_degree(float degree);

void VVVF::pwm_callback(int pwm_freq, int perids)
{
    // 在中断里，根据 freq 和 perids 计算经过的时间
    // 然后根据时间和频率，决定新的相位
    float passed_time = (float) perids / (float) pwm_freq;

    cur_angle += passed_time * output_freq * 360;

    // normalize angle
    while (cur_angle > 360)
        cur_angle -= 360;
    while (cur_angle < 0)
        cur_angle += 360;

    // 好了，根据  angle + output_duty 计算 三相的作用矢量.
    // find the sector we are in currently
    int sector = static_cast<int>(cur_angle / 120);
    static const float sector_angle[] = { 0, 120, 240, 360 };

    auto angle_in_sector = cur_angle - sector_angle[sector];

    float T1 = output_duty * sin_of_degree(120 - angle_in_sector);
    float T2 = output_duty * sin_of_degree(angle_in_sector);

    float Tmax;

    if (angle_in_sector > 60)
        Tmax = T2;
    else
        Tmax = T1;

    float center = (1 - Tmax) / 2;

    float U_a, U_b, U_c;
    switch (sector)
    {
        case 0:
            U_a = T1;
            U_b = T2;
            U_c = 0;
            break;
        case 1:
            U_a = 0;
            U_b = T1;
            U_c = T2;
            break;
        case 2:
            U_a = T2;
            U_b = 0;
            U_c = T1;
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

    m_driver->set_duty(U_a, U_b, U_c);
}

void VVVF::set_v_and_f(float V, float F)
{
    this->output_freq = F;
    this->output_duty = std::min(V, 1.0f);
}
