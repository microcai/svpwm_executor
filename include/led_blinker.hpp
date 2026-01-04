
#pragma once

#include "awaitable.hpp"

template<int ON_STATE, int OFF_STATE>
class led_blinker
{
public:
    led_blinker(int PIN_DEF)
        : led_pin(PIN_DEF)
    {
        blink_thread().detach();
    }

public:
    void off()
    {
        on_time = 0;
        off_time =50;
    }

    void on()
    {
        on_time = 50;
        off_time = 0;
    }

    void blink(const int on_time_off_time[])
    {
        this->on_time = on_time_off_time[0];
        this->off_time = on_time_off_time[1];
    }

    void blink(int on_time, int off_time)
    {
        this->on_time = on_time;
        this->off_time = off_time;
    }

private:
    int led_pin;

    int on_time;
    int off_time;

    mcucoro::awaitable<void> blink_thread()
    {
        for (;;)
        {
            digitalWrite(led_pin, on_time > 0 ? ON_STATE : OFF_STATE);
            co_await coro_delay_ms(on_time);
            digitalWrite(led_pin, off_time > 0 ? OFF_STATE : ON_STATE);
            co_await coro_delay_ms(off_time);
        }
    }
};