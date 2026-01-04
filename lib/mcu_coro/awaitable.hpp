
#pragma once

#include <coroutine>
#include "mcu_coro.hpp"

namespace mcucoro
{
	template<typename T>
	struct await_transformer;

	template<typename T>
	struct awaitable;

	template<typename T>
	struct awaitable_promise;

	template<typename T, typename CallbackFunction>
	struct CallbackAwaiter;

	template<typename T>
	struct local_storage_t
	{
	};

	inline constexpr local_storage_t<void> local_storage;

	//////////////////////////////////////////////////////////////////////////
	struct debug_coro_promise
	{
#if defined(DEBUG_CORO_PROMISE_LEAK)

		void* operator new(std::size_t size)
		{
			void* ptr = std::malloc(size);
			if (!ptr)
			{
				throw std::bad_alloc{};
			}
			debug_coro_leak.insert(ptr);
			return ptr;
		}

		void operator delete(void* ptr, [[maybe_unused]] std::size_t size)
		{
			debug_coro_leak.erase(ptr);
			std::free(ptr);
		}

#endif // DEBUG_CORO_PROMISE_LEAK
	};

	//////////////////////////////////////////////////////////////////////////
	// 存储协程 promise 的返回值
	template<typename T>
	struct awaitable_promise_value
	{
		template<typename V>
		void return_value(V&& val) noexcept
		{
			value_ = val;
		}

		void unhandled_exception() noexcept
		{
		}

		T get_value() const
		{
            return value_;
		}

		T value_;
	};

	//////////////////////////////////////////////////////////////////////////
	// 存储协程 promise 的返回值 void 的特化实现
	template<>
	struct awaitable_promise_value<void>
	{
		constexpr void return_void() noexcept
		{
		}

		void unhandled_exception() noexcept
		{
		}

		void get_value() const
		{
		}
	};

	//////////////////////////////////////////////////////////////////////////

	template<typename T>
	struct final_awaitable
	{
		awaitable_promise<T> * parent;

		constexpr void await_resume() noexcept
		{
			// 并且，如果协程处于 .detach() 而没有被 co_await
			// 则异常一直存储在 promise 里，并没有代码会去调用他的 await_resume() 重抛异常
			// 所以这里重新抛出来，避免有被静默吞并的异常
			parent->get_value();
		}

		bool await_ready() noexcept
		{
			// continuation_ 不为空，则 说明 .detach() 被 co_await, 则
			// 返回 continuation_，以便让协程框架调用 continuation_.resume()
			// 这样就把等它的协程唤醒了.
			return !parent->continuation_;
			// 如果 continuation_ 为空，则说明此乃调用链上的最后一个 promise
			// 返回 true 让协程框架 自动调用 coroutine_handle::destory()
		}

		std::coroutine_handle<> await_suspend(std::coroutine_handle<awaitable_promise<T>> h) noexcept
		{
			return h.promise().continuation_;
		}
	};

	//////////////////////////////////////////////////////////////////////////
	// 返回 T 的协程 awaitable_promise 实现.

	// Promise 类型实现...
	template<typename T>
	struct awaitable_promise : public awaitable_promise_value<T>, public debug_coro_promise
	{
		awaitable<T> get_return_object();

		auto final_suspend() noexcept
		{
			return final_awaitable<T>{this};
		}

		auto initial_suspend()
		{
			return std::suspend_always{};
		}

		std::coroutine_handle<> continuation_;
	};

	//////////////////////////////////////////////////////////////////////////

	// awaitable 协程包装...
	template<typename T>
	struct awaitable
	{
		using promise_type = awaitable_promise<T>;
		std::coroutine_handle<promise_type> current_coro_handle_;

		explicit awaitable(std::coroutine_handle<promise_type> h)
			: current_coro_handle_(h)
		{
		}

		~awaitable()
		{
			if (current_coro_handle_)
			{
				if (current_coro_handle_.done())
				{
					current_coro_handle_.destroy();
				}
				else
				{
					current_coro_handle_.resume();
				}
			}
		}

		awaitable(awaitable&& t) noexcept
			: current_coro_handle_(t.current_coro_handle_)
		{
			t.current_coro_handle_ = nullptr;
		}

		awaitable& operator=(awaitable&& t) noexcept
		{
			if (&t != this)
			{
				if (current_coro_handle_)
				{
					current_coro_handle_.destroy();
				}
				current_coro_handle_ = t.current_coro_handle_;
				t.current_coro_handle_ = nullptr;
			}
			return *this;
		}

		awaitable(const awaitable&) = delete;
		awaitable(awaitable&) = delete;
		awaitable& operator=(const awaitable&) = delete;
		awaitable& operator=(awaitable&) = delete;

		constexpr bool await_ready() const noexcept
		{
			return false;
		}

		T await_resume()
		{
			return current_coro_handle_.promise().get_value();
		}

		template<typename PromiseType>
		auto await_suspend(std::coroutine_handle<PromiseType> continuation)
		{
			current_coro_handle_.promise().continuation_ = continuation;
			return current_coro_handle_;
		}

		auto detach()
		{
			auto launched_coro = [](awaitable<T> lazy) mutable -> awaitable<T>
			{
				co_return co_await std::move(lazy);
			}(std::move(*this));


			return launched_coro;
		}
    };

	//////////////////////////////////////////////////////////////////////////

	template<typename T>
	awaitable<T> awaitable_promise<T>::get_return_object()
	{
		auto result = awaitable<T>{std::coroutine_handle<awaitable_promise<T>>::from_promise(*this)};
		return result;
	}
}

template<typename T, typename CallbackFunction>
struct CallbackAwaiter
{
public:
    // using CallbackFunction = std::function<void(std::function<void(T)>)>;

    CallbackAwaiter(CallbackFunction callback_function)
        : callback_function_(std::move(callback_function)) {}

    bool await_ready() noexcept { return false; }

    void await_suspend(std::coroutine_handle<> handle)
    {
        callback_function_([handle = std::move(handle), this](T t) mutable
        {
            result_ = std::move(t);
            handle.resume();
        });
    }

    T await_resume() noexcept { return std::move(result_); }

private:
    CallbackFunction callback_function_;
    T result_;
};

template<typename CallbackFunction>
struct CallbackAwaiter<void, CallbackFunction>
{
public:
    // using CallbackFunction = std::function<void(std::function<void()>)>;
    CallbackAwaiter(CallbackFunction callback_function)
        : callback_function_(std::move(callback_function)) {}

    bool await_ready() noexcept { return false; }

    void await_suspend(std::coroutine_handle<> handle) {
        callback_function_(handle);
    }
    void await_resume() noexcept { }

private:
    CallbackFunction callback_function_;
};

template<typename T, typename callback>
CallbackAwaiter<T, callback>
awaitable_to_callback(callback cb)
{
    return CallbackAwaiter<T, callback>{cb};
}

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
