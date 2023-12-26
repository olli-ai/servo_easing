#include "SE_ticks.h"

static uint32_t l_current_ticks = 0;

uint32_t SE_tick_get_current_tick()
{
    return l_current_ticks;
}

void SE_tick_update(uint32_t elapse_ticks)
{
    l_current_ticks += elapse_ticks;
}