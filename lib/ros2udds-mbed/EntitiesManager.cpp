#include "EntitiesManager.h"
#include <mbed.h>
#include <debug_logs.h>

#if (UXR_CREATE_ENTITIES_USING_REF == 0)
    const char UDDS_PARTICIPANT_XML[]= "<dds><participant><rtps><name>%s</name></rtps></participant></dds>";
    const char UDDS_WRITER_XML[] = "<dds><data_writer><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></data_writer></dds>";
    const char UDDS_READER_XML[] = "<dds><data_reader><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></data_reader></dds>";
    const char UDDS_TOPIC_XML[] = "<dds><topic><name>%s</name><dataType>%s</dataType></topic></dds>";
#endif

namespace ros2udds
{

EntitiesManager::EntitiesManager()
:_entity_cnt(0)
{
    for(int i=0;i<UDDS_ARRAY_SIZE;i++) _udds_entities[i] =nullptr;
}

int EntitiesManager::addEntities(EntityUdds * entities, size_t length)
{
    int index, num = 0;
    if(entities == nullptr) return num;
    
    for(int i=0;i<length;i++)
    {
        index = -1;
        switch ((entities + i)->id.type)
        {
        case UXR_PARTICIPANT_ID:
            index = UDDS_PARTICIPANT_INDEX;
            break;

        case UXR_PUBLISHER_ID:
            index = UDDS_PUBLISHER_INDEX;
            break;

        case UXR_SUBSCRIBER_ID:
            index = UDDS_SUBSCRIBER_INDEX;
            break;

        case UXR_DATAREADER_ID:
            index = findEmpty(_udds_entities, UDDS_DATA_READERS_START_INDEX, UDDS_DATA_READERS_START_INDEX + UDDS_MAX_DATA_READERS);
            break;

        case UXR_DATAWRITER_ID:
            index = findEmpty(_udds_entities, UDDS_DATA_WRITERS_START_INDEX, UDDS_DATA_WRITERS_START_INDEX + UDDS_MAX_DATA_WRITERS);
            break;

        case UXR_TOPIC_ID:
            index = findEmpty(_udds_entities, UDDS_TOPICS_START_INDEX, UDDS_TOPICS_START_INDEX + UDDS_MAX_TOPICS);
            break;

        default:
            break;
        }

        if (index != -1)
        {
            _udds_entities[index] = (entities+i);
            _udds_entities[index]->active = false;
            num++;
        }
    }
    _entity_cnt +=num;
    return num;
}

bool EntitiesManager::registerEntities(SessionUdds * session)
{
    if (session == nullptr || session->active == false || _udds_entities[UDDS_PARTICIPANT_INDEX] == nullptr)
    {
        LOG("EntitiesManager: registration start failure!\r\n");
        return false;
    }

    char entity_profile[256];
    uint16_t request;
    uint8_t status;
    int successful_init = 0;
    
    for(int i=0; i<UDDS_ARRAY_SIZE; i++)
    {
        if(_udds_entities[i]==nullptr)
            continue;

        _udds_entities[i]->active = false;

        switch(_udds_entities[i]->id.type)
        {
            case UXR_PARTICIPANT_ID:
                sprintf(entity_profile, UDDS_PARTICIPANT_XML, _udds_entities[i]->name);
                request = uxr_buffer_create_participant_xml(&session->session, session->output_stream_id, 
                _udds_entities[i]->id, 0, entity_profile, UXR_REPLACE);
                break;
            case UXR_PUBLISHER_ID:
                request = uxr_buffer_create_publisher_xml(&session->session, session->output_stream_id,
                 _udds_entities[i]->id, _udds_entities[UDDS_PARTICIPANT_INDEX]->id,"", UXR_REPLACE);
                break;
            case UXR_SUBSCRIBER_ID:
                request = uxr_buffer_create_subscriber_xml(&session->session, session->output_stream_id,
                 _udds_entities[i]->id, _udds_entities[UDDS_PARTICIPANT_INDEX]->id,"", UXR_REPLACE);
                break;
            case UXR_TOPIC_ID:
                sprintf(entity_profile, UDDS_TOPIC_XML, _udds_entities[i]->name, _udds_entities[i]->topic_type);
                request = uxr_buffer_create_topic_xml(&session->session, session->output_stream_id,
                 _udds_entities[i]->id, _udds_entities[UDDS_PARTICIPANT_INDEX]->id, entity_profile, UXR_REUSE);
                 break;
            case UXR_DATAWRITER_ID:
                sprintf(entity_profile, UDDS_WRITER_XML, _udds_entities[i]->name, _udds_entities[i]->topic_type);
                request = uxr_buffer_create_datawriter_xml(&session->session, session->output_stream_id,
                 _udds_entities[i]->id, _udds_entities[UDDS_PUBLISHER_INDEX]->id, entity_profile, UXR_REPLACE);
                 break;
            case UXR_DATAREADER_ID:
                sprintf(entity_profile, UDDS_READER_XML, _udds_entities[i]->name, _udds_entities[i]->topic_type);
                request = uxr_buffer_create_datareader_xml(&session->session, session->output_stream_id,
                _udds_entities[i]->id, _udds_entities[UDDS_SUBSCRIBER_INDEX]->id, entity_profile, UXR_REPLACE);
                 break;
            default:
                break;
        }

         if(!uxr_run_session_until_all_status(&session->session, 50, &request, &status, 1))
         {
             // TODO add more info
             if(status == UXR_STATUS_ERR_ALREADY_EXISTS)
             {
                LOG("EntityUdds already exists.\r\n");
                _udds_entities[i]->active = true;
                successful_init++;
             }
             else
             {
                LOG("EntityUdds initialisation failure.\r\n",i);
                 _udds_entities[i]->active = false;
             }
         }
         else
         {
             LOG("EntityUdds successfully registered.\r\n",i);
            _udds_entities[i]->active = true;
            successful_init++;
         }
        LOG("Type: %d\r\n"
            "ID: %d\r\n"
            "Status: %d\r\n\r\n",
            _udds_entities[i]->id.type,
            _udds_entities[i]->id.id,
            status
            );
         
    }
    return successful_init == _entity_cnt;
}



} // namespace ros2udds