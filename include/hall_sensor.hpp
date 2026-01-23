
#pragma once

#include <Arduino.h>
#include <assert.h>

template<int min, int max>
struct cyc_counter
{
    static int constexpr numbercount = (max + 1 - min);
    static int constexpr halfcount = (numbercount)/2;

    cyc_counter(){};
    cyc_counter(int init) : holded(init){}
    void operator ++(){
        holded++;
        if (holded > max)
            holded = min;
    }

    void operator --(){
        holded --;
        if (holded < min)
            holded = max;
    }

    operator int() {
        return holded;
    }

    bool operator > ( cyc_counter other)
    {
        auto diff = holded - other.holded;
        if (diff > halfcount)
        {
            return false;
        }
        else if (diff > 0)
        {
            return true;
        }
        else if (diff > -halfcount)
        {
            return false;
        }
        return true;
    }

    int holded;
};



class hall_sensor
{
public:
    int get_sector() { return m_hall_to_sector_map[hall_state]; }

    void update_sector_hall_map(uint8_t hall_state, int sector)
    {
        assert(hall_state < 7);
        m_hall_to_sector_map[hall_state] = sector;
    }

    // seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
    int m_hall_to_sector_map[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };

    int hall_state = -1;
    float erpm = 0;

    cyc_counter<0,5> pre_sector = 0;

    void hal_irq_handle(uint32_t clipsed_tmr_clock) {
        hall_state = digitalRead(PC6) + (digitalRead(PC7) << 1) + (digitalRead(PC8) << 2);

        if (erpm == 0 && clipsed_tmr_clock < 108000000)
        {
            erpm = 200;
        }
        else
        {
            erpm = system_core_clock * 10 / (float) clipsed_tmr_clock;
        }

        cyc_counter<0,5> cur_sector = m_hall_to_sector_map[hall_state];
        if (pre_sector > cur_sector)
            erpm = -erpm;
        pre_sector = cur_sector;
    }
};
