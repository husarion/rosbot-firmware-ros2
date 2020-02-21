#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

#include <mbed.h>
#include <ros2udds.h>

namespace ros2udds{

class PublisherHandle
{
    public:
        PublisherHandle(){};
        virtual ~PublisherHandle(){};
        virtual bool publish(SessionUdds * session, bool flush)=0;
};

template <class MsgT>
class Publisher : public PublisherHandle
{
    public:
        Publisher(const char * topic_name, int16_t id)
        :_topic_name("")
        {
            if(topic_name != nullptr) sprintf(_topic_name, "%s/%s", getPrefixString(TOPICS_PUBLISH), topic_name);
            data_writer_udds.id.id = id;
            data_writer_udds.id.type = UXR_DATAWRITER_ID;
            data_writer_udds.name = (const char *)_topic_name;
            data_writer_udds.topic_type = topic.topic_udds.topic_type;    
        }

        bool publish(SessionUdds * session, bool flush=false)
        {
            ucdrBuffer ub;
            if(session->active && topic.topic_udds.active && data_writer_udds.active)
            {
                uxr_prepare_output_stream(&session->session,session->output_stream_id,
                data_writer_udds.id, &ub, topic.size_of_topic(&topic,0));
                topic.serialize(&ub,&topic);
                if(flush) uxr_flash_output_streams (&session->session);
                return true;
            }
            return false;
        }

    public:
        MsgT topic;
        ros2udds::EntityUdds data_writer_udds;
    private:
        char _topic_name[32];

};
} // ros2udds
#endif /* __PUBLISHER_H__ */