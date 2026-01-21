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
    template<typename T>
    class isr_var
    {
        struct isr_var_waiter
        {
            isr_var* _parent;

            isr_var_waiter(isr_var* v) : _parent(v)
            {}

            constexpr bool await_ready() noexcept { return false; }

            void await_suspend(std::coroutine_handle<> handle)
            {
                __disable_irq();
                _parent->m_waiter = handle;
                __enable_irq();
            }

            T& await_resume() noexcept
            {
                __disable_irq();
                _parent->m_waiter = std::noop_coroutine();
                __enable_irq();

                return _parent->var;
            }
        };

    public:
        isr_var()
            : m_waiter(std::noop_coroutine())
        {
        }

        // wait for condition to set
        auto wait()
        {
            return isr_var_waiter{this};
        }

        // notify the waiter
        void notify(T&& _var)
        {
            this->var = _var;
            m_waiter.resume();
        }

        void notify(const T& _var)
        {
            this->var = _var;
            m_waiter.resume();
        }

    private:
        std::coroutine_handle<> m_waiter;
        T var;
    };

};