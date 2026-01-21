

#include "dros/mcu_coro.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#else
extern "C" uint32_t millis(void);

#include SYSTEM_HEADER

#endif


#include <utility>

#ifdef ESP_PLATFORM
#define __disable_irq taskDISABLE_INTERRUPTS
#define __enable_irq taskENABLE_INTERRUPTS
#endif

namespace mcucoro{
    executor& executor::system_executor()
    {
        static executor instance;
        return instance;
    }

    executor::executor()
    {
        active_tasks_from_isr.reserve(16);
    }

    void executor::post_from_isr(callable fn)
    {
        active_tasks_from_isr.push_back(std::move(fn));
    }

    void executor::post(callable fn)
    {
        active_tasks.push_back(std::move(fn));
    }

    void executor::poll_one()
    {
        decltype(active_tasks) to_be_run;

        __disable_irq();

        if (!active_tasks_from_isr.empty())
        {
            for(auto& T : active_tasks_from_isr)
                to_be_run.push_back(std::move(T));
            active_tasks_from_isr.clear();
        }
        else
        {
            to_be_run = clean_sleepers(std::move(active_tasks));
        }

        __enable_irq();

        for( auto & T : to_be_run)
        {
            T();
        }
    }

    void executor::poll()
    {
        poll_one();
    }

    void executor::add_timed_sleeper(uint32_t ms, callable fn)
    {
        auto point = millis() + ms;
        __disable_irq();
        sleepers.emplace( point, std::move(fn));
        __enable_irq();
    }

    std::list<callable> executor::clean_sleepers(std::list<callable> to_be_run_tasks)
    {
        auto execute_and_delete_and_advance_next = [this, &to_be_run_tasks](auto it)
        {
            auto old_it = it;
            it ++;
            to_be_run_tasks.push_back(std::move(old_it->second));
            sleepers.erase(old_it);
            return it;
        };

        auto now = millis();
        for (auto it = sleepers.begin(); it != sleepers.end();)
        {
            if (now >= it->first)
            {
                if ((now - it->first) < UINT16_MAX)
                {
                    it = execute_and_delete_and_advance_next(it);
                    continue;
                }
            }
            else if (now < it->first)
            {
                if ((now - it->first) < UINT16_MAX)
                {
                    it = execute_and_delete_and_advance_next(it);
                    continue;
                }
            }
            it ++;
        }
        return to_be_run_tasks;
    }

}