#ifndef __SUBSCRIBER_H__
#define __SUBSCRIBER_H__

#include <mbed.h>
#include <ros2udds.h>

namespace ros2udds
{

class SubscriberHandle
{
public:
    virtual ~SubscriberHandle(){};
    virtual void call(ucdrBuffer *data) = 0;
    virtual bool subscribe(SessionUdds *session, uint16_t *request_id) = 0;
public:
    ros2udds::EntityUdds data_reader_udds;
};

template <typename MsgT, typename ObjT = void>
class Subscriber : public SubscriberHandle
{
public:
    typedef void (ObjT::*CallbackT)(const MsgT &);

    Subscriber(const char *topic_name, CallbackT cb, ObjT *obj, uint16_t id)
        : _topic_name(""), _cb(cb), _obj(obj)
    {
        if (topic_name != nullptr)
            sprintf(_topic_name, "%s/%s", getPrefixString(TOPICS_SUBSCRIBE), topic_name);
        data_reader_udds.id.id = id;
        data_reader_udds.id.type = UXR_DATAREADER_ID;
        data_reader_udds.name = (const char *)_topic_name;
        data_reader_udds.topic_type = topic.topic_udds.topic_type;
    }

    void call(ucdrBuffer *data)
    {
        topic.deserialize(data, &topic);
        (_obj->*_cb)(topic);
    }

    bool subscribe(SessionUdds *session, uint16_t *request_id)
    {
        if(session->active && data_reader_udds.active && topic.topic_udds.active)
        {
            uxrDeliveryControl delivery_control;

            delivery_control.max_bytes_per_second = UXR_MAX_BYTES_PER_SECOND_UNLIMITED;
            delivery_control.max_elapsed_time = UXR_MAX_ELAPSED_TIME_UNLIMITED;
            delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
            delivery_control.min_pace_period = 0;

            *request_id = uxr_buffer_request_data(&session->session, session->output_stream_id, data_reader_udds.id, session->input_stream_id, &delivery_control);
            return true;
        }
        return false;
    }

public:
    MsgT topic;
    
private:
    CallbackT _cb;
    ObjT *_obj;
    char _topic_name[32];
};

} // namespace ros2udds
#endif