
#pragma once

struct CompontCurrent
{
    float BusCurrent = 0; // 母线电流
    float Current = 0; // 视在电流
    float RealCurrent = 0; // 有功电流
    float ReactiveCurrent = 0; // 无功电流
    float PowerFactor = 1; // 功率因数
};

class current_sense
{
public:
    CompontCurrent get_current(float VoltageAngle, float Duty);
    virtual void get_hw_current() = 0;

    float A_current = 0;
    float B_current = 0;
    float C_current = 0;

    CompontCurrent caculated_current;
};
