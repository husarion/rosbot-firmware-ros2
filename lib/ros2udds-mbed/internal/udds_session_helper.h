#ifndef __UDDS_HELPER_H__
#define __UDDS_HELPER_H__

#include <stdint.h>

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct udds_session
    {
        bool active;
        uint32_t session_key;
        uxrSession session;
        uxrStreamId output_stream_id;
        uxrStreamId input_stream_id;
#ifdef PROFILE_SERIAL_TRANSPORT
        uxrSerialTransport transport_serial;
        uxrSerialPlatform platform_serial;
#endif
#ifdef PROFILE_UDP_TRANSPORT
        uxrUDPTransport transport_udp;
        uxrUDPPlatform platform_udp;
#endif
#ifdef PROFILE_TCP_TRANSPORT
        uxrTCPTransport transport_tcp;
        uxrTCPPlatform platform_tcp;
#endif
    } udds_session_t;

#ifdef __cplusplus
}
#endif

#endif /* __UDDS_HELPER_H__ */