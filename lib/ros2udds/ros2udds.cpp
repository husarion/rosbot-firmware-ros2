#include <mbed.h>
#include "ros2udds.h"
#include <debug_logs.h>
#include <baremetal_time.h>

//TODO change to dynamic memory
#ifdef UXR_CREATE_BUFFER_BEST_EFFORT
#define BUFFER_SIZE    4096
static uint8_t output_best_effort_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS];
#else
#define STREAM_HISTORY 4
#define BUFFER_SIZE    2048 * STREAM_HISTORY
static uint8_t output_reliable_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS];
static uint8_t input_reliable_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS];
#endif

#if (UXR_CREATE_ENTITIES_USING_REF == 0)
    const char UDDS_PARTICIPANT_XML[]= "<dds><participant><rtps><name>%s</name></rtps></participant></dds>";
    const char UDDS_WRITER_XML[] = "<dds><data_writer><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></data_writer></dds>";
    const char UDDS_READER_XML[] = "<dds><data_reader><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></data_reader></dds>";
    const char UDDS_TOPIC_XML[] = "<dds><topic><name>%s</name><dataType>%s</dataType></topic></dds>";
#endif

#define UDDS_PARTICIPANT_INDEX 0
#define UDDS_PUBLISHER_INDEX 1
#define UDDS_SUBSCRIBER_INDEX 2
#define UDDS_TOPICS_START_INDEX 3
#define UDDS_DATA_READERS_START_INDEX (UDDS_TOPICS_START_INDEX + UDDS_MAX_TOPICS)
#define UDDS_DATA_WRITERS_START_INDEX (UDDS_DATA_READERS_START_INDEX + UDDS_MAX_DATA_READERS)  

static int findEmpty(ros2udds::Entity **array, int start_index, int stop_index)
{
    for (int i = start_index; i <= stop_index; i++) if (array[i] == nullptr) return i;
    return -1;
}

static inline bool isEmpty(const ros2udds::Entity** array, int index){return *(array+index) == nullptr; }

bool ros2udds::initTransport(SessionHandle *session, void *serial_instance, int baudrate)
{
    session->platform_serial.serial_instance = serial_instance;
    session->platform_serial.baudrate = baudrate;
    for(int i = 0; i < UDDS_ARRAY_SIZE; i++) session->udds_entities[i] = nullptr; //FIXME: move initialisation somwhere elese

    baremetal_init_timer(); // start internal timer

    if (!uxr_init_serial_transport(&session->transport_serial, &session->platform_serial, 0, 0, 0))
    {
        LOG("Error at create transport.\r\n");
        return false;
    }

    return true;
}

static const char *ROS2_DDS_ENTITY_PREFIX_NAME[] = {
    "rt",
    "rt",
    "rq",
    "rr",
    "rs",
    "rp",
    "ra",
};

const char* ros2udds::getPrefixString(EntityNamePrefix prefix)
{
  return ROS2_DDS_ENTITY_PREFIX_NAME[prefix];
}

bool ros2udds::initSession(SessionHandle * session, uint32_t session_key, uxrOnTopicFunc on_topic_func, void * args)
{
    session->active = false;
    session->session_key = session_key;

    // init session
    uxr_init_session(&session->session, &session->transport_serial.comm, session->session_key);
    // set callbacks
    // uxr_set_status_callback(&session->session,on_status_func,args);
    uxr_set_topic_callback(&session->session,on_topic_func,args);

    if(!uxr_create_session(&session->session))
    {
        LOG("Error at create session.\r\n");
        return false;
    }

    session->active = true;

    // create output stream
    session->output_stream_id = uxr_create_output_reliable_stream(&session->session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // create input stream
    session->input_stream_id = uxr_create_input_reliable_stream(&session->session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    return true;
}

// bool ros2udds::deleteSession(SessionHandle * session)
// {
//     //TODO implement session deletion
// }

int ros2udds::addEntity(SessionHandle * session, Entity * entity)
{
        int index = -1;
        if(session != nullptr && entity != nullptr)
        {
        switch (entity->id.type)
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
            index = findEmpty(session->udds_entities, UDDS_DATA_READERS_START_INDEX, UDDS_DATA_READERS_START_INDEX + UDDS_MAX_DATA_READERS);
            break;

        case UXR_DATAWRITER_ID:
            index = findEmpty(session->udds_entities, UDDS_DATA_WRITERS_START_INDEX, UDDS_DATA_WRITERS_START_INDEX + UDDS_MAX_DATA_WRITERS);
            break;

        case UXR_TOPIC_ID:
            index = findEmpty(session->udds_entities, UDDS_TOPICS_START_INDEX, UDDS_TOPICS_START_INDEX + UDDS_MAX_TOPICS);
            break;

        default:
            break;
        }
        if (index != -1)
        {
            entity->active = false;
            session->udds_entities[index] = entity;
            return 1;
        }
        return 0;
    }
}

//TODO: register in one approach
int ros2udds::registerEntities(SessionHandle * session)
{
    if (session == nullptr || session->active == false || session->udds_entities[UDDS_PARTICIPANT_INDEX] == nullptr)
    {
        LOG("EntitiesManager: registration start failure!\r\n");
        return 0;
    }

    char entity_profile[256];
    uint16_t request;
    uint8_t status;
    int successful_init = 0;
    
    for(int i=0; i<UDDS_ARRAY_SIZE; i++)
    {
        if(session->udds_entities[i]==nullptr)
            continue;

        session->udds_entities[i]->active = false;

        switch(session->udds_entities[i]->id.type)
        {
            case UXR_PARTICIPANT_ID:
                sprintf(entity_profile, UDDS_PARTICIPANT_XML, session->udds_entities[i]->name);
                request = uxr_buffer_create_participant_xml(&session->session, session->output_stream_id, 
                session->udds_entities[i]->id, 0, entity_profile, UXR_REPLACE);
                break;
            case UXR_PUBLISHER_ID:
                request = uxr_buffer_create_publisher_xml(&session->session, session->output_stream_id,
                 session->udds_entities[i]->id, session->udds_entities[UDDS_PARTICIPANT_INDEX]->id,"", UXR_REPLACE);
                break;
            case UXR_SUBSCRIBER_ID:
                request = uxr_buffer_create_subscriber_xml(&session->session, session->output_stream_id,
                 session->udds_entities[i]->id, session->udds_entities[UDDS_PARTICIPANT_INDEX]->id,"", UXR_REPLACE);
                break;
            case UXR_TOPIC_ID:
                sprintf(entity_profile, UDDS_TOPIC_XML, session->udds_entities[i]->name, session->udds_entities[i]->topic_type);
                request = uxr_buffer_create_topic_xml(&session->session, session->output_stream_id,
                 session->udds_entities[i]->id, session->udds_entities[UDDS_PARTICIPANT_INDEX]->id, entity_profile, UXR_REUSE);
                 break;
            case UXR_DATAWRITER_ID:
                sprintf(entity_profile, UDDS_WRITER_XML, session->udds_entities[i]->name, session->udds_entities[i]->topic_type);
                request = uxr_buffer_create_datawriter_xml(&session->session, session->output_stream_id,
                 session->udds_entities[i]->id, session->udds_entities[UDDS_PUBLISHER_INDEX]->id, entity_profile, UXR_REPLACE);
                 break;
            case UXR_DATAREADER_ID:
                sprintf(entity_profile, UDDS_READER_XML, session->udds_entities[i]->name, session->udds_entities[i]->topic_type);
                request = uxr_buffer_create_datareader_xml(&session->session, session->output_stream_id,
                session->udds_entities[i]->id, session->udds_entities[UDDS_SUBSCRIBER_INDEX]->id, entity_profile, UXR_REPLACE);
                 break;
            default:
                break;
        }

         if(!uxr_run_session_until_all_status(&session->session, 50, &request, &status, 1))
         {
             // TODO add more info
             if(status == UXR_STATUS_ERR_ALREADY_EXISTS)
             {
                LOG("Entity already exists.\r\n");
                session->udds_entities[i]->active = true;
                successful_init++;
             }
             else
             {
                LOG("Entity initialisation failure.\r\n",i);
                 session->udds_entities[i]->active = false;
             }
         }
         else
         {
             LOG("Entity successfully registered.\r\n",i);
            session->udds_entities[i]->active = true;
            successful_init++;
         }
        LOG("Type: %d\r\n"
            "ID: %d\r\n"
            "Status: %d\r\n\r\n",
            session->udds_entities[i]->id.type,
            session->udds_entities[i]->id.id,
            status
            );
         
    }
    return successful_init;
}