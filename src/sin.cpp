
#include <arm_math.h>

float sin_of_degree(float degree)
{
    float sinval, cosval;
    arm_sin_cos_f32(degree, &sinval, &cosval);
    return sinval;
}
