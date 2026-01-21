
#pragma once

#include "mcu_coro.hpp"

#include "awaitable.hpp"

#include "corothread.hpp"

struct delay_awaiter
{
	int ms;
public:
    // using CallbackFunction = std::function<void(std::function<void()>)>;
    delay_awaiter(int ms)
        : ms(ms){}

    constexpr bool await_ready() noexcept { return false; }

    void await_suspend(std::coroutine_handle<> handle)
	{
        mcucoro::delay_ms(ms, handle);
	}
    constexpr void await_resume() noexcept { }
};

template<typename INT>
auto coro_delay_ms(INT ms)
{
    return delay_awaiter(ms);
}

inline auto leave_isr()
{
	struct leave_isr_awaiter
	{
	    constexpr bool await_ready() noexcept { return false; }
	    constexpr void await_resume() noexcept { }
		void await_suspend(std::coroutine_handle<> handle)
		{
			mcucoro::post_from_isr(handle);
		}
	};
    return leave_isr_awaiter{};
}
