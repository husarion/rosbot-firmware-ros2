#include "../include/transport_mbed_internal.h"
#include <BufferedSerial.h>

static BufferedSerial * serial_instance_ptr = nullptr;

bool uxr_initSerialMbed(void * serial_instance, int baudrate)
{
    serial_instance_ptr = (BufferedSerial *)serial_instance;
    serial_instance_ptr->baud(baudrate);
    return true;
}

size_t uxr_writeSerialDataMbed(uint8_t* buf, size_t len)
{
    return serial_instance_ptr->write(buf,len);
}

size_t uxr_readSerialDataMbed(uint8_t* buf, size_t len, int timeout)
{
    size_t rv = 0;
    
    // change to non blocking
#if 0
    uint32_t pre_time = Kernel::get_ms_count() + timeout;
    while (rv < len && (ros2mbed_timer.read_ms() - pre_time) < (uint32_t) timeout)
    {
        if(serial_instance_ptr->readable())
            buf[rv++] = serial_instance_ptr->getc();
    }
    return rv;
#else
    // while(pre_time < Kernel::get_ms_count())
    // {
    //     // ThisThread::sleep_for(1);
    //     ThisThread::yield();
    // }
    ThisThread::sleep_for(timeout);
    
    for(int i=0;i<len;i++)
    {
        if(!serial_instance_ptr->readable()) break;
        buf[rv++] = serial_instance_ptr->getc();
    }
#endif
    return rv;

}