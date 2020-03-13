#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

#include <mbed.h>
#include <ros2udds.h>

namespace ros2udds{

class PublisherHandle
{
public:
    PublisherHandle(SessionHandle *session)
        : _session(session), _topic_name("")
    {
        _id_cnt++;
    }
    virtual ~PublisherHandle()
    {
        _id_cnt--;
    }
    virtual bool publish(bool flush) = 0;
    virtual int addToRegister() = 0;
    uint16_t getId() { return _data_writer_udds.id.id; }

protected:
    static uint16_t _id_cnt;
    SessionHandle *_session;
    char _topic_name[32];
    Entity _data_writer_udds;
    Entity _topic_udds;
};

template <class MsgT>
class Publisher : public PublisherHandle
{
public:
    Publisher(SessionHandle *session, const char *topic_name, EntityNamePrefix endpoint = TOPICS_PUBLISH)
        : PublisherHandle(session)
    {
        if (topic_name != nullptr)
            sprintf(_topic_name, "%s/%s", getPrefixString(endpoint), topic_name);

        // topic
        _topic_udds.id = uxr_object_id(topic.id_, UXR_TOPIC_ID);
        _topic_udds.name = topic.name_;
        _topic_udds.topic_type = topic.type_;

        // data writer
        _data_writer_udds.id = uxr_object_id(_id_cnt, UXR_DATAWRITER_ID);
        _data_writer_udds.name = (const char *)_topic_name;
        _data_writer_udds.topic_type = _topic_udds.topic_type;
    }

    int addToRegister()
    {
        int res = 0;
        res += addEntity(_session, &_topic_udds);
        res += addEntity(_session, &_data_writer_udds);
        return res;
    }

    ~Publisher()
    {
        // removeEntities(_session, &topic.topic_udds);
        // removeEntities(_session, &_data_writer_udds);
    }

    bool publish(bool flush=false)
    {
        ucdrBuffer ub;
        // ucdr_init_buffer(&ub,nullptr,0); 
        if (_session->active && _topic_udds.active && _data_writer_udds.active)
        {
            uint32_t size = topic.size_of_topic(&topic,0);
            if(uxr_prepare_output_stream(&_session->session,_session->output_stream_id,
            _data_writer_udds.id, &ub, size))
            {
                bool res = topic.serialize(&ub, &topic);
                if (res && flush) uxr_flash_output_streams(&_session->session);
                return res;
            }
        }
        return false;
    }

public:
    MsgT topic;
};

} // namespace ros2udds
#endif /* __PUBLISHER_H__ */