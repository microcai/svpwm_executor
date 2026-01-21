
#pragma once

// this is a cooperative-multi-thread library for arm32 based MCU
#include <functional>
#include "callable.hpp"

#include "../fcontext/fcontext.hpp"

namespace corothread {

extern fcontext_t ___current_yield_fcontext;

union thread_context {
    unsigned int stacks[384];
};

// 用静态内存创建协程
void create_static_context(thread_context* ctx, callable thread_entry);

// 用动态内存创建协程，协程退出的时候自动清除自己.
void create_dynamic_context(callable thread_entry);

static inline void resume_context(const void * arg)
{
    jump_fcontext(reinterpret_cast<fcontext_t>(const_cast<void*>(arg)), 0);
}

// 回调机制转协程，用这个实现.
template<typename resumerHandler>
void current_context_as_callback(resumerHandler resumer)
{


    auto on_suspend_fcontext = [](transfer_t caller) -> transfer_t
    {
        resumerHandler* resumer = reinterpret_cast<resumerHandler*>(caller.data);
        (*resumer)( callable( &resume_context, caller.fctx ) );
        return caller;    
    };

	___current_yield_fcontext = ontop_fcontext(___current_yield_fcontext, static_cast<void*>(&resumer), on_suspend_fcontext).fctx;
}

void context_yield();
void thread_delay(int ms);
void leave_isr();

class coro_mutex
{
protected:
    int locked;
public:
    coro_mutex();
    void lock();
    void unlock();
};

}