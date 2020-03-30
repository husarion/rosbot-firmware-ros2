#ifndef __ROS2_UDDS_H__
#define __ROS2_UDDS_H__

#include <stdlib.h>
#include <string.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#define UDDS_MAX_DATA_READERS 10
#define UDDS_MAX_DATA_WRITERS 10
#define UDDS_MAX_TOPICS (UDDS_MAX_DATA_READERS)+(UDDS_MAX_DATA_WRITERS)
#define UDDS_ARRAY_SIZE 3 + UDDS_MAX_TOPICS + UDDS_MAX_DATA_WRITERS + UDDS_MAX_DATA_READERS

namespace ros2udds{

struct Entity
{
    uxrObjectId id;
    const char *name;
    const char *topic_type;
    bool active;
};

struct SessionHandle
{
    uint32_t session_key;
    uxrSession session;
    uxrStreamId output_stream_id;
    uxrStreamId input_stream_id;
    uxrSerialTransport transport_serial;
    uxrSerialPlatform platform_serial;
    Entity * udds_entities[UDDS_ARRAY_SIZE];
    bool active;
};

enum EntityNamePrefix : int
{
    TOPICS_PUBLISH = 0,
    TOPICS_SUBSCRIBE,
    SERVICE_REQUEST,
    SERVICE_RESPONSE,
    SERVICE,
    PARAMETER,
    ACTION
};

// void initSessionUdds();

bool initTransport(SessionHandle * session, void * serial_instance, int baudrate);

bool initSession(SessionHandle * session, uint32_t session_key);

void registerSessionCallbacks(SessionHandle *session, 
                              uxrOnTopicFunc on_topic_func, 
                              uxrOnTimeFunc on_time_func, 
                              void *args);

bool deleteSession(SessionHandle * session);

const char* getPrefixString(EntityNamePrefix prefix);

int addEntity(SessionHandle * session, Entity * entity);

int registerEntities(SessionHandle * session);

} //ros2udds


#endif /* __ROS2_UDDS_H__ */