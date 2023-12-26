#ifndef SE_TICKS_H
#define SE_TICKS_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
uint32_t SE_tick_get_current_tick();
void SE_tick_update(uint32_t elapse_ticks);

#ifdef __cplusplus
}
#endif

#endif /*SE_TICKS_H*/