
#pragma once

#include "mcu_coro.hpp"

mcucoro::awaitable<void> async_led_blink(int LED_pin, int on_time, int off_time, int blink_times = 1)
{
	for (; blink_times > 0; blink_times--)
	{
		digitalWrite(LED_pin, LOW);
		co_await coro_delay_ms(on_time);
		digitalWrite(LED_pin, HIGH);
		co_await coro_delay_ms(off_time);
	}
}