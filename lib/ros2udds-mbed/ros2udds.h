#ifndef __ROS2_UDDS_H__
#define __ROS2_UDDS_H__

#include <string.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

namespace ros2udds{

struct EntityUdds
{
    uxrObjectId id;
    const char *name;
    const char *topic_type;
    bool active;
};

struct SessionUdds
{
    uint32_t session_key;
    uxrSession session;
    uxrStreamId output_stream_id;
    uxrStreamId input_stream_id;
    uxrSerialTransport transport_serial;
    uxrSerialPlatform platform_serial;
    bool active;
};

bool initTransport(SessionUdds * session, void * serial_instance, int baudrate);
bool initSession(SessionUdds * session, uint32_t session_key);
bool deleteSession(SessionUdds * session);

enum MessagePrefix
{
    TOPICS_PUBLISH = 0,
    TOPICS_SUBSCRIBE,
    SERVICE_REQUEST,
    SERVICE_RESPONSE,
    SERVICE,
    PARAMETER,
    ACTION
};

const char* getPrefixString(MessagePrefix prefix);

} //ros2udds


#endif /* __ROS2_UDDS_H__ */