
#pragma once

#include <cmath>
#include <cstdint>
#include "ctsqrt.hpp"
#include <type_traits>

#ifdef CPU_IM90
#include "hau_drv.h"
#define HAVE_HAU 1
#else
#define HAVE_HAU 0
#endif

#ifdef abs
#undef abs
#endif

namespace fast_math
{
    // 用 cpp 的运算符重载功能，实现自定义的定点浮点数功能.
    template <int SCALE, typename number_holder = long>
    struct float_number_t
    {
        static constexpr int scale_sqr = ct::sqrt(SCALE);
        number_holder scaled_number;

        constexpr float_number_t() : scaled_number(0) {}

        explicit constexpr float_number_t(unsigned short v): scaled_number(static_cast<number_holder>(v * SCALE)){}
        explicit constexpr float_number_t(int v): scaled_number(static_cast<number_holder>(v * SCALE)){}
        explicit constexpr float_number_t(long v): scaled_number(static_cast<number_holder>(v * SCALE)){}
        explicit constexpr float_number_t(number_holder v, int scale): scaled_number(v)
        {
            if (scale != SCALE)
            {
                scaled_number = scaled_number * SCALE / scale;
            }
        }

        template <typename Q = number_holder>
        constexpr float_number_t(number_holder v, typename std::enable_if<!std::is_same<Q, int>::value>::type*) : scaled_number(static_cast<number_holder>(v * SCALE)){}

        explicit constexpr float_number_t(float v): scaled_number(static_cast<number_holder>(v * SCALE)){}
        explicit constexpr float_number_t(double v): scaled_number(static_cast<number_holder>(v * SCALE)){}
        constexpr float_number_t(const float_number_t& o) : scaled_number(o.scaled_number){ }
        template<int other_scale, typename other_type>
        explicit float_number_t(const float_number_t<other_scale, other_type>& o)
        {
            if constexpr(other_scale > SCALE)
            {
                scaled_number = o.scaled_number / (other_scale/ SCALE);
            }
            else if constexpr (other_scale < SCALE)
            {
                scaled_number = o.scaled_number * (SCALE / other_scale);
            }
            else if constexpr(SCALE == other_scale)
            {
                scaled_number = o.scaled_number;
            }
        }

        float_number_t& operator = (float_number_t&& o) { scaled_number = o.scaled_number; return *this; }
        // float_number_t& operator = (const float_number_t& o) { scaled_number = o.scaled_number; return *this; }
        float_number_t& operator = (const float_number_t<SCALE, number_holder>& o) { scaled_number = o.scaled_number; return *this; }

        float_number_t& operator = (float v) { scaled_number = static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator = (double v) { scaled_number = static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator = (int v) { scaled_number = static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator = (long v) { scaled_number = static_cast<number_holder>(v * SCALE); return *this; }


        template<int other_scale, typename other_type>
        float_number_t& operator = (const float_number_t& o)
        {
            if constexpr(other_scale > SCALE)
            {
                scaled_number = o.scaled_number / (other_scale/ SCALE);
            }
            else if constexpr (other_scale < SCALE)
            {
                scaled_number = o.scaled_number * (SCALE / other_scale);
            }
            else if (SCALE == other_scale)
            {
                scaled_number = o.scaled_number;
            }
        }

        float_number_t floor() const
        {
            float_number_t r(*this);
            r.scaled_number = r.scaled_number / SCALE * SCALE;
            return r;
        }

        float_number_t abs() const
        {
            float_number_t r(*this);
            if (r.scaled_number < 0)
                r.scaled_number = 0 - scaled_number;
            return r;
        }

        float_number_t& flip_sign()
        {
            scaled_number = 0 - scaled_number;
            return *this;
        }

        explicit operator float () const { return static_cast<float>(scaled_number) / static_cast<float>(SCALE) ; }
        explicit operator double () const { return static_cast<double>(scaled_number) / static_cast<double>(SCALE) ; }
        explicit operator int () const { return scaled_number / SCALE;}
        explicit operator unsigned int () const { return scaled_number / SCALE;}
        explicit operator unsigned long () const { return scaled_number / SCALE;}

        template<class Q = number_holder>
        typename std::enable_if<!std::is_same<Q, int>::value, float_number_t&>::type
        operator += ( int v) { scaled_number += static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator += ( number_holder v ) { scaled_number +=static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator += ( float v ) { scaled_number += static_cast<number_holder>(v * SCALE); return *this; }
        float_number_t& operator += ( const float_number_t& v ) { scaled_number += v.scaled_number; return *this; }

        float_number_t operator -() { return float_number_t{-scaled_number, SCALE}; }
        float_number_t& operator --() { --scaled_number; return *this; }
        float_number_t& operator ++() { ++scaled_number; return *this; }

        template<class Q = number_holder>
        typename std::enable_if<!std::is_same<Q, int>::value, float_number_t&>::type
        operator -= ( int v ) { scaled_number -= static_cast<number_holder>(v) * static_cast<number_holder>(SCALE); return *this; }
        float_number_t& operator -= ( number_holder v ) { scaled_number -= static_cast<number_holder>(v) * static_cast<number_holder>(SCALE); return *this; }
        float_number_t& operator -= ( float v ) { scaled_number -= v * SCALE; return *this; }
        float_number_t& operator -= ( const float_number_t& v ) { scaled_number -= v.scaled_number; return *this; }

        float_number_t& operator *= ( const float_number_t& v )
        {
            scaled_number /= scale_sqr;
            scaled_number *= v.scaled_number / scale_sqr;
            return *this;
        }
        float_number_t& operator *= ( int v ) { scaled_number *= static_cast<number_holder>(v); return *this; }
        float_number_t& operator *= ( unsigned int v ) { scaled_number *= static_cast<number_holder>(v); return *this; }
        float_number_t& operator *= ( long v ) { scaled_number *= static_cast<number_holder>(v); return *this; }
        float_number_t& operator *= ( unsigned long v ) { scaled_number *= static_cast<number_holder>(v); return *this; }
        float_number_t& operator *= ( float v ) { scaled_number = v * scaled_number; return *this; }
        float_number_t& operator *= ( double v ) { scaled_number = v * scaled_number; return *this; }

        inline float_number_t& operator /= ( const float_number_t& v )
        {
            scaled_number *= scale_sqr;
            scaled_number /= v.scaled_number;
            scaled_number *= scale_sqr;
            return *this;
        }
        template<int SCALE_v, typename T>
        float_number_t& operator /= ( const float_number_t<SCALE_v, T>& v )
        {
            scaled_number *= float_number_t<SCALE_v, T>::scale_sqr;
            scaled_number /= v.scaled_number;
            scaled_number *= float_number_t<SCALE_v, T>::scale_sqr;
            return *this;
        }

        inline float_number_t& operator /= ( int v )
        {
            scaled_number /= static_cast<number_holder>(v);
            return *this;
        }

        float_number_t& operator /= ( float v ) { scaled_number /= static_cast<number_holder>(v); return *this; }

        float_number_t Root()
        {
            if constexpr ( HAVE_HAU && std::is_same_v<number_holder, long>)
            {
                number_holder new_number = HAU_CalcRoot(scaled_number);
                return float_number_t{new_number, SCALE};
            }
            else
            {
                number_holder new_number = sqrt(scaled_number);
                return float_number_t{new_number, SCALE};
            }
        }

        void sub_a(int a)
        {
            // this.sub_a(a) = a / this;
            scaled_number = SCALE * a / scaled_number;
            scaled_number *= SCALE;
        }

        bool operator == (int v) const
        {
            return scaled_number == v *SCALE;
        }

        bool operator == (const float_number_t& v) const
        {
            return scaled_number == v.scaled_number;
        }

        bool operator != (int v) const
        {
            return scaled_number != v *SCALE;
        }

        bool operator != (const float_number_t& v) const
        {
            return scaled_number != v.scaled_number;
        }

        bool operator < (const float_number_t& b) const
        {
            return this->scaled_number < b.scaled_number;
        }

        bool operator < (int v) const
        {
            return this->scaled_number < static_cast<number_holder>(v) * SCALE;
        }

        bool operator < (float v) const
        {
            return this->scaled_number < static_cast<number_holder>(v * SCALE);
        }

        bool operator < (double v) const
        {
            return this->scaled_number < static_cast<number_holder>(v * SCALE);
        }

        bool operator <= (int v) const
        {
            return this->scaled_number <= static_cast<number_holder>(v) * SCALE;
        }

        bool operator <= (float v) const
        {
            return this->scaled_number <= v * SCALE;
        }

        bool operator <= (const float_number_t& v) const
        {
            return this->scaled_number <= v.scaled_number;
        }

        bool operator > (int v) const
        {
            return this->scaled_number > static_cast<number_holder>(v) * SCALE;
        }

        bool operator > (const float_number_t& v) const
        {
            return this->scaled_number > v.scaled_number;
        }

        bool operator >= (int v) const
        {
            return this->scaled_number >= static_cast<number_holder>(v) * SCALE;
        }

        bool operator >= (long v) const
        {
            return this->scaled_number >= v * SCALE;
        }

        bool operator >= (const float_number_t& v) const
        {
            return this->scaled_number >= v.scaled_number;
        }

        bool operator >= (float v) const
        {
            return this->scaled_number >= v * SCALE;
        }

        bool operator > (float v) const
        {
            return this->scaled_number > v * SCALE;
        }

        bool operator >= (double v) const
        {
            return this->scaled_number >= v * SCALE;
        }

        bool close_to(const float_number_t& b) const
        {
            return (scaled_number - b.scaled_number) < scale_sqr
            && (b.scaled_number - scaled_number) < scale_sqr ;
        }

    };

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator + (const float_number_t<SCALE, H>& a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(a);
        r += b;
        return r;
    }

    template <int SCALE, typename H, typename  T>
    float_number_t<SCALE, H> operator + (const float_number_t<SCALE, H>& a, const T& b)
    {
        float_number_t<SCALE, H> r(a);
        r += b;
        return r;
    }

    template <int SCALE, typename H, typename  T>
    float_number_t<SCALE, H> operator - (const float_number_t<SCALE, H>& a, const T& b)
    {
        float_number_t<SCALE, H> r(a);
        r -= b;
        return r;
    }

    template <int SCALE, typename H, typename  T>
    float_number_t<SCALE, H> operator - (T a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(b);
        r.flip_sign();
        r += a;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator - (const float_number_t<SCALE, H>& a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(a);
        r -= b;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator - (const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(b);
        r.flip_sign();
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator * (const float_number_t<SCALE, H>& a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(a);
        r *= b;
        return r;
    }

    template <int SCALE, typename H, typename  T>
    float_number_t<SCALE, H> operator * (const float_number_t<SCALE, H>& a, T&& b)
    {
        float_number_t<SCALE, H> r(a);
        r *= b;
        return r;
    }

    template <int SCALE, typename H, typename  T>
    float_number_t<SCALE, H> operator * (T b, const float_number_t<SCALE, H>& a)
    {
        float_number_t<SCALE, H> r(a);
        r *= b;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator / (const float_number_t<SCALE, H>& a, int b)
    {
        float_number_t<SCALE, H> r(a);
        r /= b;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator / (const float_number_t<SCALE, H>& a, float b)
    {
        float_number_t<SCALE, H> r(a);
        r /= b;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator / (const float_number_t<SCALE, H>& a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(a);
        r /= b;
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> operator / (int a, const float_number_t<SCALE, H>& b)
    {
        float_number_t<SCALE, H> r(b);
        r.sub_a(a);
        return r;
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> floor (const float_number_t<SCALE, H>& input)
    {
        return input.floor();
    }

    template <int SCALE, typename H>
    float_number_t<SCALE, H> abs (const float_number_t<SCALE, H>& input)
    {
        return input.abs();
    }

    #if !defined (ESP_PLATFORM)
    static inline float abs(float v)
    {
        return std::abs(v);
    }
    #endif

    // using int128_t = Wider<int64_t>;

    // 默认的 SCALE 为 10000 倍，也就是 4 位小数点精度.
    using float_number = float_number_t<10000>;
    using bigger_float = float_number_t<10000, int64_t>;
    using double_number = float_number_t<90000, int64_t>;
}

namespace slow_math
{
    using float_number = float;
    using double_number = double;
    using bigger_float = float;
}
