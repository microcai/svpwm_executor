
#pragma once

#include "./vec.hpp"
#include "./mat.hpp"

namespace matrix
{

    // 3x2 * 3 = vec2
    template <typename T>
    vec<T, 2> operator* ( const mat<T, 2, 3>& m, const vec<T, 3>& v)
    {
        vec<T, 2> r;
        r[0] = v[0] * m[0][0] + v[1] * m[0][1] + v[2] * m[0][2];
        r[1] = v[0] * m[1][0] + v[1] * m[1][1] + v[2] * m[1][2];
        return r;
    }

    // 2x2 * 2 = vec2
    template <typename T>
    vec<T, 2> operator* ( const mat<T, 2, 2>& m, const vec<T, 2>& v)
    {
        vec<T, 2> r;
        r[0] = v[0] * m[0][0] + v[1] * m[0][1];// + v[2] * m[0][2];
        r[1] = v[0] * m[1][0] + v[1] * m[1][1];// + v[2] * m[1][2];
        return r;
    }

    // 2x3 * 2 = vec3
    template <typename T>
    vec<T, 3> operator* ( const mat<T, 3, 2>& m, const vec<T, 2>& v)
    {
        vec<T, 3> r;
        static_assert( std::is_same<const vec<T, 2>&, decltype(m[0])>::value );
        r[0] = v[0] * m[0][0] + v[1] * m[0][1];// + v[2] * m[0][2];
        r[1] = v[0] * m[1][0] + v[1] * m[1][1];// + v[2] * m[1][2];
        r[2] = v[0] * m[2][0] + v[1] * m[2][1];// + v[2] * m[1][2];
        return r;
    }

} // namespace matrix
