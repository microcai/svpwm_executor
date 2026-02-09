
#include "arm_math.h"
#include "current_sense.hpp"
#include "SEGGER_RTT.h"

#include "debug.hpp"

CompontCurrent current_sense::get_current(float VoltageAngle, float hw_duty)
{
    // debug_print("get_hw_current enter this = %p\r\n", this);

    this->get_hw_current();

    auto pingfang =  2.0* (A_current*A_current + B_current * B_current + C_current*C_current) / 3;

    arm_sqrt_f32(pingfang, &caculated_current.Current);

    float Ialpha, Ibeta, sinval, cosval;

    arm_sin_cos_f32(VoltageAngle, &sinval, &cosval);
    arm_clarke_f32(A_current, B_current, &Ialpha,&Ibeta);

    arm_park_f32(Ialpha, Ibeta, &caculated_current.ReactiveCurrent, &caculated_current.RealCurrent, sinval, cosval);

    auto RealCurrentSqure = caculated_current.RealCurrent*caculated_current.RealCurrent;
    auto ReactiveCurrentSqure = caculated_current.ReactiveCurrent*caculated_current.ReactiveCurrent;

    arm_sqrt_f32( RealCurrentSqure / pingfang, &caculated_current.PowerFactor);
    // 母线电流只取决于有功电流.
    caculated_current.BusCurrent = (caculated_current.RealCurrent * hw_duty);
    return caculated_current;
}
