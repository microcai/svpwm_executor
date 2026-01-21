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
    class event_var
    {
        struct event_var_waiter
        {
            event_var* _parent;

            event_var_waiter(event_var* v) : _parent(v)
            {}

            constexpr bool await_ready() noexcept { return false; }

            void await_suspend(std::coroutine_handle<> handle)
            {
                _parent->m_waiter = handle;
            }

            T& await_resume() noexcept
            {
                _parent->m_waiter = std::noop_coroutine();

                return _parent->var;
            }
        };

    public:
        event_var()
            : m_waiter(std::noop_coroutine())
        {
        }

        // wait for condition to set
        auto wait()
        {
            return event_var_waiter{this};
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