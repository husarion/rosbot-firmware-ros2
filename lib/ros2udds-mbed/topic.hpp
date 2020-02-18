#ifndef __TOPIC_HPP__
#define __TOPIC_HPP__

#include "ros2udds.h"
#include "topic_id_number.h"

namespace ros2udds
{

template <class MsgT>
class Topic
{
public:
    Topic(const char *type, const char *name, uint16_t id)
        :topic_udds{{id,UXR_TOPIC_ID},name,type,false}
    {}

    virtual ~Topic(){}
    virtual bool serialize(ucdrBuffer *msg_buf, const MsgT *topic) = 0;
    virtual bool deserialize(ucdrBuffer *msg_buf, MsgT *topic) = 0;
    virtual uint32_t size_of_topic(const MsgT *topic, uint32_t size)=0;

    EntityUdds topic_udds;
};

} // namespace ros2udds

#endif /* __TOPIC_HPP__ */
