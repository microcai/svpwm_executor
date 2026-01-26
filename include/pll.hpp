
#pragma once

#include "cyccounter.hpp"

class PLL
{
public:
    void reset();
    void new_phase_arrive(int phase);

    // TODO, 预留 api, 为 一圈一个脉冲的 PLL 留好.
    void new_full_cycle_arrive();

    bool is_phase_locked();

    float get_predict_phase(float delta_time);
    float get_speed();

    // 相位在指定区间 [0, 360)
    cyc_counter<float, 0.0f, 360.0f> locked_phase;

    cyc_counter<float, 0.0f, 360.0f> last_raw_phase;

    // 成功执行相位跟踪的次数。如果 < 3 则视为相位跟踪失败.
    int phase_success_lock_count;

    float tracked_speed_roate_per_second = 0.0f;
    float time_since_last_arrive;
};

