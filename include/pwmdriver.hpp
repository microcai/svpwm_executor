
#pragma once

#include <functional>
#include <vector>

#include "fast_math.hpp"

namespace motorlib
{

class svpwm;
class pwmdriver
{
    pwmdriver(const pwmdriver&) = delete;
    pwmdriver(pwmdriver&&) = delete;
public:
    pwmdriver();

    virtual ~pwmdriver();

    typedef void (svpwm::*pwm_callback_memptr)(int, int);

public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual int get_frequency() = 0;
    virtual void set_frequency(int) = 0;
    virtual void set_duty(float_number U_a, float_number U_b, float_number U_c) = 0;

    void link_timer(pwm_callback_memptr, svpwm* parent);

    int break_status = 0; // error status report

    int perid_exec_time = 0;

    pwm_callback_memptr callback_ptr;
    svpwm* parent;

protected:
    void invoke_callbacks(int pwm, int perids);
};

}
