#ifndef __SUBSCRIBER_H__
#define __SUBSCRIBER_H__

#include <mbed.h>
#include <ros2udds.h>

namespace ros2udds
{

class SubscriberHandle
{
public:
    SubscriberHandle(SessionHandle *session)
        : _session(session), _topic_name("")
    {
        _id_cnt++;
    }
    virtual ~SubscriberHandle()
    {
        _id_cnt--;
    };
    virtual void call(ucdrBuffer *data) = 0;
    virtual bool subscribe(uint16_t *request_id) = 0;
    virtual int addToRegister() = 0;
    uint16_t getId() { return _data_reader_udds.id.id; }

protected:
    static uint16_t _id_cnt;
    SessionHandle *_session;
    char _topic_name[32];
    Entity _data_reader_udds;
    Entity _topic_udds;
};

template <typename MsgT, typename ObjT = void>
class Subscriber : public SubscriberHandle
{
public:
    typedef void (ObjT::*CallbackT)(const MsgT &);

    Subscriber(SessionHandle *session, const char *topic_name, CallbackT cb, ObjT *obj, EntityNamePrefix endpoint = TOPICS_SUBSCRIBE)
        : SubscriberHandle(session)
        , _cb(cb)
        , _obj(obj)
    {
        if (topic_name != nullptr) sprintf(_topic_name, "%s/%s", getPrefixString(endpoint), topic_name);
        
        // topic
        _topic_udds.id = uxr_object_id(topic.id_,UXR_TOPIC_ID);
        _topic_udds.name = topic.name_;
        _topic_udds.topic_type = topic.type_;

        // data reader
        _data_reader_udds.id = uxr_object_id(_id_cnt, UXR_DATAREADER_ID);
        _data_reader_udds.name = (const char *)_topic_name;
        _data_reader_udds.topic_type = _topic_udds.topic_type;
    }

    int addToRegister()
    {
        int res = 0;
        res += addEntity(_session, &_topic_udds);
        res += addEntity(_session, &_data_reader_udds);
        return res;
    }

    ~Subscriber()
    {
        // removeEntities(_session, &topic.topic_udds)
        // removeEntities(_session, &_data_reader_udds)
    }

    void call(ucdrBuffer *data)
    {
        topic.deserialize(data, &topic);
        (_obj->*_cb)(topic);
    }

    bool subscribe(uint16_t *request_id)
    {
        if(_session->active && _data_reader_udds.active && _topic_udds.active)
        {
            uxrDeliveryControl delivery_control;

            delivery_control.max_bytes_per_second = UXR_MAX_BYTES_PER_SECOND_UNLIMITED;
            delivery_control.max_elapsed_time = UXR_MAX_ELAPSED_TIME_UNLIMITED;
            delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
            delivery_control.min_pace_period = 0;

            *request_id = uxr_buffer_request_data(&_session->session, _session->output_stream_id, _data_reader_udds.id, _session->input_stream_id, &delivery_control);
            return true;
        }
        return false;
    }

public:
    MsgT topic;
    
private:
    CallbackT _cb;
    ObjT *_obj;
};

} // namespace ros2udds
#endif