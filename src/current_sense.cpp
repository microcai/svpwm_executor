
#include "arm_math.h"
#include "current_sense.hpp"
#include "SEGGER_RTT.h"

#include "debug.hpp"
#include "dsp/fast_math_functions.h"
#include "dsp/filtering_functions.h"

#include "mat.hpp"

#define sqrt3 1.732050807568877f

constexpr auto clark_matrix = matrix<3, 3>{
        {2.0f, -1.0f , -1.0f},
        { 0.0f, sqrt3, -sqrt3},
        { 1.0f, 1.0f , 1.0f},
};

CompontCurrent current_sense::get_current(float VoltageAngle, float hw_duty)
{
    matrix<3, 1> Iabc = {
        { A_current },
        { B_current },
        { C_current },
    };


    // debug_print("get_hw_current enter this = %p\r\n", this);

    // this->get_hw_current();

    // auto pingfang =  2.0* (A_current*A_current + B_current * B_current + C_current*C_current) / 3;

    // arm_sqrt_f32(pingfang, &caculated_current.Current);

    float Ialpha, Ibeta, sinval, cosval;

    matrix<3, 1> Iab = clark_matrix * Iabc;

    Ialpha = Iab.values[0][0] / 3.0f;
    Ibeta = Iab.values[1][0] /3.0f;

    // arm_clarke_f32(A_current, B_current, &Ialpha, &Ibeta);

    arm_sin_cos_f32(VoltageAngle, &sinval, &cosval);

    arm_park_f32(Ialpha, Ibeta, &caculated_current.RealCurrent, &caculated_current.ReactiveCurrent, sinval, cosval);

    float current_lag;
    arm_atan2_f32(caculated_current.ReactiveCurrent, caculated_current.RealCurrent, &current_lag);

    caculated_current.PowerFactor = arm_cos_f32(current_lag);

    // auto RealCurrentSqure = caculated_current.RealCurrent*caculated_current.RealCurrent;
    // auto ReactiveCurrentSqure = caculated_current.ReactiveCurrent*caculated_current.ReactiveCurrent;

    // auto total_current = caculated_current.RealCurrent + caculated_current.ReactiveCurrent;

    // caculated_current.PowerFactor = 1;
    // if (total_current> 0)
    //     caculated_current.PowerFactor = caculated_current.RealCurrent / total_current;

    // arm_sqrt_f32( RealCurrentSqure / pingfang, &caculated_current.PowerFactor);
    // 母线电流只取决于有功电流.
    caculated_current.BusCurrent = (caculated_current.RealCurrent * hw_duty);
    return caculated_current;
}

const float Coeffs[] = {
-0.013160246982733511331153941625871084398,
0.054322645634425605065676023741616518237,
-0.124378805415713805659905233369499910623,
0.195316119533468834923439771955600008368,
0.774358143658792186236894394824048504233,
0.195316119533468834923439771955600008368,
-0.124378805415713805659905233369499910623,
0.054322645634425605065676023741616518237,
-0.013160246982733511331153941625871084398,
};

current_lpf::current_lpf()
{ 
    // arm_fir_init_f32(&m_fir, 9, Coeffs, state, 1);
}

float current_lpf::operator()(float new_sample)
{
    float ret = last_sample * 0.95 + new_sample * 0.05;
    // arm_fir_f32(&m_fir, &new_sample, &ret, 1);
    last_sample = ret;
    return ret;
}
