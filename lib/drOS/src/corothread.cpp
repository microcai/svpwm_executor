


#include <stdlib.h>
#include "Arduino.h"
#include "dros/delay.hpp"
#include "dros/corothread.hpp"
#include "../fcontext/fcontext.hpp"

#include "dros/awaitable.hpp"

namespace corothread {

fcontext_t ___current_yield_fcontext = nullptr;

static void _provided_entry_no_auto_free(transfer_t arg)
{
    ___current_yield_fcontext = arg.fctx;

    callable* user_function = reinterpret_cast<callable*>( arg.data );

    // invoke callable
    (*user_function)();

    user_function->~callable();

	jump_fcontext(___current_yield_fcontext, 0);
}

// 用静态内存创建协程
void create_static_context(thread_context* ctx, callable thread_entry)
{
    // 其实主要在于 swap_context 的实现方式
    new (reinterpret_cast<void*>(ctx)) callable (std::move(thread_entry));

    // swap_context
    auto newly_created_ctx = make_fcontext(ctx + 1, sizeof (thread_context), _provided_entry_no_auto_free);

    jump_fcontext(newly_created_ctx, ctx);
}

static void _provided_entry_with_auto_free(transfer_t arg)
{
    ___current_yield_fcontext = arg.fctx;

    thread_context* ctx = reinterpret_cast<thread_context*>(arg.data);
    callable* user_function = reinterpret_cast<callable*>( arg.data );

    // invoke callable
    (*user_function)();
    user_function->~callable();

	auto at_exit_fcontext = [](transfer_t caller)
	{
        free(caller.data);
		return caller;
	};

	ontop_fcontext(___current_yield_fcontext, ctx, at_exit_fcontext);
}

// 用动态内存创建协程，协程退出的时候自动清除自己.
void create_dynamic_context(callable thread_entry)
{
    thread_context* ctx = (thread_context*) malloc(sizeof(thread_context));
    new (reinterpret_cast<void*>(ctx)) callable (std::move(thread_entry));

    // swap_context
    auto newly_created_ctx = make_fcontext(ctx + 1, sizeof (thread_context), _provided_entry_with_auto_free);
    jump_fcontext(newly_created_ctx, ctx);
}

void context_yield()
{
	current_context_as_callback([](callable resume) { mcucoro::delay_ms(0, std::move(resume)); });
}

void thread_delay(int ms)
{
    if (___current_yield_fcontext)
    	current_context_as_callback([ms](callable resume) { mcucoro::delay_ms(ms, std::move(resume)); });
    else        
        delay(ms);
}

static transfer_t on_suspend_fcontext_isr(transfer_t caller)
{
    mcucoro::post_from_isr( &resume_context, caller.fctx );
	return caller;
}

void leave_isr()
{
	___current_yield_fcontext = ontop_fcontext(___current_yield_fcontext, 0, on_suspend_fcontext_isr).fctx;
}

coro_mutex::coro_mutex()
    : locked(0)
{}

void coro_mutex::lock()
{
    if ( locked == 0)
    {
        locked = 1;
        return;
    }

    while (locked!=0)
    {
        context_yield();
    }
    locked = 1;
}

void coro_mutex::unlock()
{
    locked = 0;
}

}