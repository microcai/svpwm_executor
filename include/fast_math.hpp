
#pragma once

#include "smartfloat.hpp"

#include <cmath>

namespace fast_math
{
    float_number sin_of_degree(float_number angle_degree);

#if !defined (USE_SLOW_MATH)
    inline bool is_close_enough(const float_number& a, const float_number& b)
    {
        return a.close_to(b);
    }
#endif

    template<class Number>
    inline bool is_close_enough(Number a, Number b)
    {
        return abs(a - b) <= 1e-8;
    }


    template<class Number>
    Number sqrt(Number a)
    {
        Number x0 = Number{1}; // 初始猜测值
        Number x1 = (x0 + a/x0)/2; // 第一次计算下一个猜测值
        while (!is_close_enough(x1,x0)) { // 如果差值大于精度，继续迭代
            x0 = x1; // 更新猜测值
            x1 = (x0 + a/x0)/2; // 计算下一个猜测值
        }
        return x1; // 返回最终的猜测值
    }
}

namespace slow_math
{
	// TODO, write fast sin code.
	static inline float sin_of_degree(float degree_angle) { return std::sin(degree_angle / 180.0 * 3.1415926535897932384626); }
}


// #define  USE_SLOW_MATH  1
#if defined (USE_SLOW_MATH)
using namespace slow_math;
#else
using namespace fast_math;
#endif
