
#pragma once

#include <functional>
#include <vector>

class VVVF;

namespace motorlib
{

class pwmdriver
{
    pwmdriver(const pwmdriver&) = delete;
    pwmdriver(pwmdriver&&) = delete;
public:
    pwmdriver();

    virtual ~pwmdriver();

    typedef void (VVVF::*pwm_callback_memptr)(int, int);

public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual int get_frequency() = 0;
    virtual void set_frequency(int) = 0;
    virtual void set_duty(float U_a, float U_b, float U_c) = 0;

    void link_timer(pwm_callback_memptr, VVVF* parent);

    int break_status = 0; // error status report

    int perid_exec_time = 0;

    pwm_callback_memptr callback_ptr;
    VVVF* parent;

protected:
    void invoke_callbacks(int pwm, int perids);
};

}
