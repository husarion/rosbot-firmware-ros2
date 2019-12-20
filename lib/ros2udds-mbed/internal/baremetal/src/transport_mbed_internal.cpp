#include "../include/transport_mbed_internal.h"
#include <BufferedSerial.h>

static BufferedSerial serial_instance(ROS2UDDS_SERIAL_TX_PIN, ROS2UDDS_SERIAL_RX_PIN);

size_t uxr_writeSerialDataMbed(uint8_t* buf, size_t len)
{
    return serial_instance.write(buf,len);
}

size_t uxr_readSerialDataMbed(uint8_t* buf, size_t len, int timeout)
{
    size_t rv = 0;
    uint32_t pre_time = Kernel::get_ms_count() + timeout;
    
    // change to non blocking
#if 0
    while (rv < len && (ros2mbed_timer.read_ms() - pre_time) < (uint32_t) timeout)
    {
        if(serial_instance.readable())
            buf[rv++] = serial_instance.getc();
    }
    return rv;
#else
    while(pre_time < Kernel::get_ms_count())
    {
        // ThisThread::sleep_for(1);
        ThisThread::yield();
    }
    for(int i=0;i<len;i++)
    {
        if(!serial_instance.readable()) break;
        buf[rv++] = serial_instance.getc();
    }
#endif
    return rv;

}