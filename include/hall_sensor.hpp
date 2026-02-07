
#pragma once

#include <Arduino.h>
#include <assert.h>

#include "cyccounter.hpp"


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
    int m_hall_to_sector_map[8] = { -1,  2,  0,  1,  4,  3,  5 , -1 };

    int hall_state = -1;

    void hal_irq_handle() {
        hall_state = digitalRead(PC6) + (digitalRead(PC7) << 1) + (digitalRead(PC8) << 2);
    }
};
