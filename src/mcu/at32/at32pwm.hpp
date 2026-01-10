
#pragma once

#include <memory>
#include "pwmdriver.hpp"

namespace motorlib
{
struct at32pwmdriver_impl;
class at32pwmdriver : public pwmdriver
{
public:
    at32pwmdriver();
    ~at32pwmdriver();

public:
    virtual void start() override;
    virtual void stop() override;
    virtual int get_frequency() override;
    virtual void set_frequency(int) override;
    virtual void set_duty(float_number U_a, float_number U_b, float_number U_c) override;
private:
    void tmr1_interrupt(int);
    at32pwmdriver_impl * impl;
    friend class at32pwmdriver_impl;
    char impl_static_storage[80];
};

}