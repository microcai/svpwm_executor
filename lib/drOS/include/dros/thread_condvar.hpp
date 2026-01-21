#pragma once

#include "callable.hpp"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include SYSTEM_HEADER
#endif

namespace corothread
{
    class condition_variable
    {
    public:
        condition_variable()
        {
        }

        // wait for condition to set
        void wait()
        {
            current_context_as_callback([this](callable resumer){
#ifdef ESP_IDF_VERSION
                vPortEnterCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __disable_irq();
#endif                
                m_waiter = std::move(resumer);
#ifdef ESP_IDF_VERSION
                vPortExitCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __enable_irq();
#endif
                in_wait = true;
            });
        }

        // notify the waiter
        void notify()
        {
            if (m_waiter && in_wait)
            {
                in_wait = false;
                mcucoro::executor::system_executor().post_from_isr(std::move(m_waiter));
            }
        }

    private:
        callable m_waiter;
#ifdef ESP_IDF_VERSION
        portMUX_TYPE coro_task_mux;
#endif
        bool in_wait = false;
    };


};