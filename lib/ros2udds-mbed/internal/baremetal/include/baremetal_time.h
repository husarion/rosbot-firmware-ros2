#ifndef __BAREMETAL_TIME_H__
#define __BAREMETAL_TIME_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t baremetal_millis();
uint32_t baremetal_micros();
int64_t baremetal_nanos();
void baremetal_init_timer();

#ifdef __cplusplus
}
#endif 
#endif /* __BAREMETAL_TIME_H__ */