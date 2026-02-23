
#pragma once

#include "dsp/filtering_functions.h"
struct CompontCurrent
{
    float BusCurrent = 0; // 母线电流
    float Current = 0; // 视在电流
    float RealCurrent = 0; // 有功电流
    float ReactiveCurrent = 0; // 无功电流
    float PowerFactor = 1; // 功率因数
};

class current_lpf
{
    // arm_fir_instance_f32 m_fir;
    // float32_t state[16];
    float last_sample = 0;
public:
	current_lpf();

	float operator()(float new_sample);
};

class current_sense
{
public:
    CompontCurrent get_current(float VoltageAngle, float Duty);
    virtual void get_hw_current() = 0;

    float A_current = 0;
    float B_current = 0;
    float C_current = 0;

    current_lpf A_lpf, B_lpf, C_lpf;

    CompontCurrent caculated_current;
};
