#include <uxr/client/profile/transport/serial/serial_transport_baremetal.h>
#include <src/c/profile/transport/serial/serial_protocol_internal.h>

#include "../include/transport_mbed_internal.h"

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd,
    uint8_t remote_addr, uint8_t local_addr)
{
  (void) (fd);
  (void) (remote_addr);
  (void) (local_addr);
  
  /* Open device */
  if(platform == NULL)
  {
      return false;
  }

  return uxr_initSerialMbed(platform->baudrate);
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{
  (void)(platform);

  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf,
    size_t len, uint8_t* errcode)
{
  (void)(platform);

  size_t rv = 0;

  rv = uxr_writeSerialDataMbed(buf, len);
  if (0 != rv)
  {
    *errcode = 0;
  }
  else
  {
    *errcode = 1;
  }
  return rv;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf,
    size_t len, int timeout, uint8_t* errcode)
{
  (void)(platform);

  size_t rv = 0;

  rv = uxr_readSerialDataMbed(buf, len, timeout);

  if (0 < rv)
  {
    *errcode = 0;
  }
  else
  {
    *errcode = 1;
  }

  return rv;
}

