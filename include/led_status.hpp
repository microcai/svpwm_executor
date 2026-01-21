
#pragma once

#include "Arduino.h"
#include "dros/mcu_coro.hpp"
#include "async_led_blink.hpp"


mcucoro::awaitable<void> led_status_1(int LED_pin)
{
	for (;;)
	{
		co_await async_led_blink(LED_pin, 350, 350);
	}
}

