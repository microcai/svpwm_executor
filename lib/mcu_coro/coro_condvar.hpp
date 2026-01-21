#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include SYSTEM_HEADER
#endif

#include <coroutine>
#include "awaitable.hpp"

namespace mcucoro
{
    class condition_variable
    {
        struct condition_variable_waiter
        {
            condition_variable* cond;

            condition_variable_waiter(condition_variable* c) : cond(c)
            {}

            constexpr bool await_ready() noexcept { return false; }

            void await_suspend(std::coroutine_handle<> handle)
            {
#ifdef ESP_IDF_VERSION
                vPortEnterCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __disable_irq();
#endif
                cond->m_cond = false;
                cond->m_waiter = handle;
#ifdef ESP_IDF_VERSION
                vPortExitCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __enable_irq();
#endif
            }

            bool await_resume() noexcept
            {
#ifdef ESP_IDF_VERSION
                vPortEnterCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __disable_irq();
#endif
                cond->m_waiter = std::noop_coroutine();
#ifdef ESP_IDF_VERSION
                vPortExitCriticalSafe(&cond->coro_task_mux);
#elif defined(__arm__)
                __enable_irq();
#endif
                return cond->m_cond;
            }
        };

    public:
        condition_variable()
            : m_waiter(std::noop_coroutine())
        {
        }

        // wait for condition to set
        auto wait()
        {
            return condition_variable_waiter{this};
        }

        // notify the waiter
        void notify()
        {
            if (!m_cond)
            {
                m_cond = true;
                m_waiter.resume();
            }
        }

    private:
        std::coroutine_handle<> m_waiter;
#ifdef ESP_IDF_VERSION
        portMUX_TYPE coro_task_mux;
#endif
        bool m_cond = false;
    };


};