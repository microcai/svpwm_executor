
#include <cstdlib>
#include "pll.hpp"

#include "stdio.h"
#include "SEGGER_RTT.h"

// 速度补偿的 增益.
static float gain = 0.1;

void PLL::reset()
{
    tracked_speed_roate_per_second = 0.0f;
}

// 通过调节内部的 RPS (Rotated Per Second) 数字使得内部跟踪的相位和外部的相位保持一个较低的误差
void PLL::new_phase_arrive(int phase)
{
    // 找到相位误差。
    auto cur_raw_phase = decltype(locked_phase)(phase);
    auto raw_phase_diff = cur_raw_phase - last_raw_phase;
    auto phase_diff = cur_raw_phase - locked_phase;
    auto phase_diff_abs = std::abs(phase_diff);
    auto speed_between_phase_arrive = raw_phase_diff / time_since_last_arrive;
    auto locked_phase_ = locked_phase;

    if (raw_phase_diff * tracked_speed_roate_per_second < 0)
    {
        tracked_speed_roate_per_second = speed_between_phase_arrive;
    }

    if (phase_diff_abs >= 60)
    {
        phase_success_lock_count = 0;
    }

    // 如果相位误差较大，则丢失相位跟踪
    if (phase_diff_abs < 15)
    {
        phase_success_lock_count ++;
    }
    if (phase_success_lock_count == 0)
    {
        //锁定次数清零，这将导致 is_phase_loced 返回 false.
        locked_phase = cur_raw_phase;
    }

    tracked_speed_roate_per_second = tracked_speed_roate_per_second * 0.85 + speed_between_phase_arrive * 0.15;
    tracked_speed_roate_per_second += phase_diff;
    last_raw_phase = cur_raw_phase;

    // static	char buf[128];
	// int len = snprintf(buf, 128, "hall angle = %d, hall_advance = %d, tracked_angle=%d, angle diff = %d, speed = %d, locked_count = %d\r\n",
    //     phase,(int)raw_phase_diff, (int) locked_phase_.holded, (int)phase_diff, (int) tracked_speed_roate_per_second/6, phase_success_lock_count);

	// SEGGER_RTT_Write(0, buf, len);
    time_since_last_arrive = 0;
}

float PLL::get_predict_phase(float delta)
{
    tracked_speed_roate_per_second *= 0.99995;
    time_since_last_arrive += delta;
    // 根据 delta * tracked_speed_roate_per_second 更新
    cyc_counter<float, 0.0f, 360.0f> new_pos = locked_phase;
    new_pos += tracked_speed_roate_per_second * delta;

    auto diff = new_pos - last_raw_phase;
    locked_phase = new_pos;

    if (std::abs(diff) > 80)
    {
        static	char buf[128];
        if (phase_success_lock_count > 0)
        {
            // int len = snprintf(buf, 128, "--- pll out out sync --, predict = %d, last_raw = %d\r\n",
            //     (int) new_pos.holded, (int)last_raw_phase.holded);
        	// SEGGER_RTT_Write(0, buf, len);
        }
        phase_success_lock_count = 0;
    }

    return new_pos.holded;
}

float PLL::get_speed()
{
    return tracked_speed_roate_per_second / 6;
}

bool PLL::is_phase_locked()
{
    return false;
    // return phase_success_lock_count >= 3;
}
