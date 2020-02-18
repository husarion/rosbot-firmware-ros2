
#include <mbed.h>
#include <EntitiesManager.h>
#include <debug_logs.h>
#include "examples/PublishHelloWorld/HelloWorld.h"
#include <baremetal_time.h>

#ifdef UXR_CREATE_BUFFER_BEST_EFFORT
#define BUFFER_SIZE    4096
static uint8_t output_best_effort_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS];
#else
#define STREAM_HISTORY 4
#define BUFFER_SIZE    2048 * STREAM_HISTORY
static uint8_t output_reliable_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS];
static uint8_t input_reliable_stream_buffer[BUFFER_SIZE * UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS];
#endif

using namespace udds;

static EntitiesManager::Entity test_entities[4] = 
{
    {{1,UXR_PARTICIPANT_ID},"test_participant",nullptr,false},
    {{1,UXR_PUBLISHER_ID},nullptr,nullptr,false},
    {{1,UXR_TOPIC_ID},"HelloWorldPubSubTopic","HelloWorld",false},
    {{1,UXR_DATAWRITER_ID},"HelloWorldPubSubTopic","HelloWorld",false}
};

DigitalOut led(LED3);

void test()
{
    LOG("Program started!\r\n");
    led = 1;

    //TODO init session 
    udds_session_t session;
    session.active = false;
    session.session_key = 0xAABBCCDD;
    session.platform_serial.baudrate = 230400;
    
    baremetal_init_timer();

    //init serial transport
    if (!uxr_init_serial_transport(&session.transport_serial, &session.platform_serial, 0, 0, 0))
    {
        LOG("Error at create transport.\n");
        return;
    }

    LOG("Transport created successfully\r\n");

    // init session
    uxr_init_session(&session.session, &session.transport_serial.comm, session.session_key);

    LOG("Session initialized successfully\r\n");

    if(!uxr_create_session(&session.session))
    {
        LOG("Error at create session.\n");
        return;
    }

    LOG("Session created successfully!\r\n");

    session.active = true;

    // create output stream
    session.output_stream_id = uxr_create_output_reliable_stream(&session.session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);
    session.input_stream_id = uxr_create_input_reliable_stream(&session.session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // add UDDS entities to manager
    EntitiesManager entities_manager;

    for(int i=0; i<4; i++)
    {
        entities_manager.addEntity(&test_entities[i]);
    }

    // register entities
    if(!entities_manager.registerEntities(&session))
    {
        LOG("Error at entities creation!\r\n");
        return;
    }

    // Write topics
    bool connected = true;
    uint32_t count = 0;
    while(connected)
    {
        HelloWorld topic = {++count, "Hello DDS world!"};

        ucdrBuffer ub;
        uint32_t topic_size = HelloWorld_size_of_topic(&topic, 0);
        uxr_prepare_output_stream(&session.session, session.output_stream_id, test_entities[3].id, &ub, topic_size);
        HelloWorld_serialize_topic(&ub, &topic);

        // LOG("Send topic: %s, id: %i\r\n", topic.message, topic.index);
        connected = uxr_run_session_time(&session.session, 1000);
    }
    LOG("Stopping...\r\n");
    // Delete resources
    uxr_delete_session(&session.session);
    uxr_close_serial_transport(&session.transport_serial);
    LOG("Closing the session and transport...\r\n");
}