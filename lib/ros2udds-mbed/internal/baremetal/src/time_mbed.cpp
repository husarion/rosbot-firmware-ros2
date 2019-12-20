#include <mbed.h>
#include <../include/time.h>

static Timer udds_timer;

void baremetal_init_timer()
{
    udds_timer.start();
}

uint32_t baremetal_millis()
{
    return udds_timer.read_ms();
}

uint32_t baremetal_micros()
{
    return udds_timer.read_us();
}

int64_t baremetal_nanos()
{
    static uint32_t pre_usec = 0, now_usec;
    static int64_t nsec = 0;

    now_usec = baremetal_micros();
    nsec += (int64_t)(now_usec - pre_usec) * (int64_t)1000;
    pre_usec = now_usec;

    return nsec;
}