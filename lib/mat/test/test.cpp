
#include "../include/matrix_operator.hpp"

#include <cmath>
#include <cstdio>

using current_t = float;
using namespace matrix;

static mat<current_t, 2, 3>  ClarkTransform_Matrix =
{
    vec<current_t, 3> {1.f/ 1.5f, -0.5f/ 1.5f, -0.5f/ 1.5f},
    vec<current_t, 3> {0.f/ 1.5f, 0.86602540378f/ 1.5f, -0.866025403784f/ 1.5f},
};

static mat<current_t, 3, 2>  InverseClarkTransform_Matrix =
{
    vec<current_t, 2> {1.f, 0.0f },
    vec<current_t, 2> {-0.5f, 0.86602540378f},
    vec<current_t, 2> {-0.5f, -0.866025403784f},
};

static const float pi = 3.1415926535897932384626f;
static const float _2pi = pi*2;
static const float dec2rad = pi / 180.0;
static const float Bphaserad = _2pi/3.0;
static const float Cphaserad = pi*4/3.0;

int main()
{
    for (int angle = 0; angle <= 360; angle ++)
    {
        float angle_ = angle;
        vec<current_t, 3> sample = { cos(angle*pi / 180) * 0.2, cos(angle*dec2rad - Bphaserad)* 0.2, cos(angle*dec2rad + Bphaserad) * 0.2};


        vec<current_t, 2> result = ClarkTransform_Matrix * sample;

        current_t AC_CURRENT = sqrt(result[0] * result[0] + result[1] * result[1]);

        std::printf("angle %f : sample %f, %f, %f\n", angle_, sample[0], sample[1], sample[2]);
        std::printf("angle %f : transformed %f, %f\n", angle_, result[0], result[1]);
        std::printf("angle %f : DC = %f\n", angle_, AC_CURRENT);


        float theta = angle + 40;
        mat<current_t, 2, 2>  ParkTransform_Matrix =
        {
            vec<current_t, 2> {cos(theta* pi / 180), sin(theta* pi / 180)},
            vec<current_t, 2> { -sin(theta* pi / 180), cos(theta* pi / 180)},
        };

        vec<current_t, 2> dq = ParkTransform_Matrix * result;

        std::printf("angle %f : park transformed %f, %f\n", angle_, dq[0], dq[1]);
        std::printf("angle %f : park DC = %f, power factor = %f\n", angle_,dq[0], dq[0]/AC_CURRENT);


        mat<current_t, 2, 2>  InverseParkTransform_Matrix =
        {
            vec<current_t, 2> {cos(theta*pi / 180), -sin(theta*pi / 180)},
            vec<current_t, 2> {sin(theta*pi / 180), cos(theta*pi / 180)},
        };

        vec<current_t, 2> result2 = InverseParkTransform_Matrix * dq;

        std::printf("angle %f : inverse park transformed %f, %f\n", angle_, result2[0], result2[1]);

        vec<current_t, 3> orig =  InverseClarkTransform_Matrix * result2;

        std::printf("angle %f : inverse clarke transformed %f, %f, %f\n", angle_, orig[0], orig[1], orig[2]);

        std::printf("----------------------------------------\n");

    }
}
