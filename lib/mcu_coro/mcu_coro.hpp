
#pragma once

#include <map>
#include <list>
#include <vector>
#include <map>
#include <cstdint>
#include <coroutine>

#include "myfunc.hpp"

namespace mcucoro
{
    class executor
    {
    public:
        // run for ever
        void run();
        // returns if no more active task.
        void poll();
        void poll_one();

        void post(callable fn);
        void post_from_isr(callable fn);

        static executor& system_executor();

        executor();

        void add_timed_sleeper(uint32_t ms, callable fn);

    protected:
        std::list<callable> clean_sleepers(std::list<callable>);

    private:
        std::vector<callable> active_tasks_from_isr;
        std::list<callable> active_tasks;

        std::multimap<uint32_t, callable> sleepers;
    };

    // delay ms and execute fn
    static inline void delay_ms(int ms, callable fn)
    {
        if (ms)
            executor::system_executor().add_timed_sleeper(ms, std::move(fn));
        else
            executor::system_executor().post(std::move(fn));
    }

    static inline void yield(callable fn)
    {
        executor::system_executor().post(std::move(fn));
    }

    static inline void post_from_isr(void (*function)())
    {
        executor::system_executor().post_from_isr(callable{function});
    }

    static inline void post_from_isr(void (*function)(const void*), const void* data)
    {
        executor::system_executor().post_from_isr(callable{function, data});
    }

    static inline void post_from_isr(std::coroutine_handle<void> resume_function)
    {
        executor::system_executor().post_from_isr(callable{resume_function});
    }

    static inline void post(callable fn)
    {
        executor::system_executor().post(std::move(fn));
    }

    template <typename T>
    static inline void post(T&& fn)
    {
        executor::system_executor().post(callable(fn));
    }

} // namespace mcu
