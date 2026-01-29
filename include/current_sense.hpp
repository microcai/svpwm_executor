
#pragma once

struct CompontCurrent
{
    float BusCurrent; // 母线电流
    float Current; // 视在电流
    float RealCurrent; // 有功电流
    float ReactiveCurrent; // 无功电流
};

class current_sense
{
public:
    CompontCurrent get_current(float VoltageAngle, float Duty);
    virtual void get_hw_current() = 0;

    float A_current;
    float B_current;
    float C_current;

    CompontCurrent caculated_current;
};
