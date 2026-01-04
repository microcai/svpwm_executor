
#include "app.hpp"
#include "mcu_coro.hpp"
#include "coroutine.hpp"
#include "Arduino.h"
#include "awaitable.hpp"

mcucoro::awaitable<void> modbusReadLoop(APP* app)
{
    HardwareSerial* uart_port;

    for (;;)
    {
        while(!uart_port->available())
                co_await coro_delay_ms(20);

        uart_port->readBytes();    
    }

    co_return;
}
