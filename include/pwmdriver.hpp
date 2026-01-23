
#pragma once

#include <functional>
#include <vector>

namespace motorlib
{

typedef void (*pwm_callback_ptr)(int, int, void * user_data);

class pwmdriver
{
    pwmdriver(const pwmdriver&) = delete;
    pwmdriver(pwmdriver&&) = delete;
public:
    pwmdriver();

    virtual ~pwmdriver();


public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual int get_frequency() = 0;
    virtual void set_frequency(int) = 0;
    virtual void set_duty(float U_a, float U_b, float U_c) = 0;

    void link_timer(pwm_callback_ptr, void* user_data);
    void unlink_timer();

    int break_status = 0; // error status report

    int perid_exec_time = 0;

    pwm_callback_ptr callback_ptr;
    void* callback_ptr_user_data;

protected:
    void invoke_callbacks(int pwm, int perids);
};

}
