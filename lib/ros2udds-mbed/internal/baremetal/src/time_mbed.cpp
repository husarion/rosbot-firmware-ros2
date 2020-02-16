#include <mbed.h>
#include <../include/baremetal_time.h>

static Timer udds_timer;
//TODO use decicated hardware timer

void baremetal_init_timer()
{
    udds_timer.start();
}

int64_t baremetal_millis()
{
    return baremetal_nanos()/1000000;
}

int64_t baremetal_micros()
{
    return baremetal_nanos()/1000;
}

int64_t baremetal_nanos()
{
    static uint32_t pre_usec = 0, now_usec;
    static int64_t nsec = 0;

    now_usec = udds_timer.read_us();
    nsec += (int64_t)(now_usec - pre_usec) * (int64_t)1000;
    pre_usec = now_usec;

    return nsec;
}