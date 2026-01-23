
#pragma once

#include <functional>
#include <type_traits>
#include <utility>
#include <coroutine>

template<typename T>
concept FunctionObject = requires (T t)
{
    t();
};

/**
    std::function should be copy-constructable which is not suitable for awaitable object which is move-only,
    so let me write this wrapper that is std::function alike.
*/
struct callable
{
    void operator()() const
    {
        m_function(m_data);
    }

    callable(const callable&) = delete;
    callable(callable&) = delete;
    callable(callable&& o)
        : m_function(o.m_function)
        , m_data(o.m_data)
    {
        o.m_function = nullptr;
        o.m_data = nullptr;
    }

    operator bool() const
    {
        return m_function != nullptr;
    }

    callable& operator = (callable&& o)
    {
        this->~callable();
        m_function = o.m_function;
        m_data = o.m_data;
        o.m_data = nullptr;
        o.m_function = nullptr;
        return *this;
    }

    callable()
        : m_function(nullptr)
        , m_data(nullptr)
    {}

    callable(void (*raw_function)())
        : m_function( reinterpret_cast< decltype(m_function)>(raw_function))
        , m_data(nullptr)
    {
    }

    callable(void (*raw_function)(const void* data), const void* data)
        : m_function(reinterpret_cast<decltype(m_function)>(raw_function))
        , m_data(data)
    {
    }
    
    template<typename UserArgType, typename UserArgType2> requires std::convertible_to<UserArgType2*, UserArgType*>
    callable(void (*raw_function)(UserArgType* data), UserArgType2* data)
        : m_function(reinterpret_cast<decltype(m_function)>(raw_function))
        , m_data(data)
    {
    }


    callable(std::coroutine_handle<> coro_handle)
    {
        m_data = coro_handle.address();
        m_function = [](const void* data){
            std::coroutine_handle<>::from_address(const_cast<void*>(data)).resume();
        };
    }

    template<typename T>
    static void invoke_functor_wrapper(const void* data)
    {
        (* reinterpret_cast<T*>(const_cast<void*>(data)))();
    }

    template<typename T> requires (FunctionObject<T>)
    callable(T* functionobject)
    {
        m_data = functionobject;
        m_function = invoke_functor_wrapper<T>;
    }

    // 将  [](){} 这种类型的 lambda 纳入 函数指针范畴.
    template<typename T> requires ( FunctionObject<T> && sizeof(T) == 1)
    callable(const T& functionobject)
        : m_data(nullptr)
    {
        typedef void (*dummy_function)();
        // m_data = functionobject;
        m_function = (decltype(m_function)) (dummy_function) functionobject;
    }

    typedef std::function<void()> functor_t;

    template<typename T> requires ( sizeof(T) >= 8)
    callable(T&& function)
    {
        static_assert(sizeof(function) >= 8, "should be real object");
        m_data = new functor_t{std::move(function)};
        m_function = invoke_std_function_wrapper;
    }

    static void invoke_std_function_wrapper(const void* data)
    {
        (* reinterpret_cast<functor_t*>(const_cast<void*>(data)))();
    }


    ~callable()
    {
        if (m_data && (m_function == &invoke_std_function_wrapper))
        {
            delete reinterpret_cast<functor_t*>(const_cast<void*>(m_data));
        }
        m_data = nullptr;
        m_function = nullptr;
    }

    void (*m_function)(const void* data);
    const void * m_data;
};
