#include <mbed.h>
#include "ros2udds.h"
#include <debug_logs.h>
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

const char *ROS2_DDS_ENTITY_PREFIX_NAME[] = {
    "rt",
    "rt",
    "rq",
    "rr",
    "rs",
    "rp",
    "ra",
};

bool ros2udds::initTransport(SessionUdds *session, void *serial_instance, int baudrate)
{
    session->platform_serial.serial_instance = serial_instance;
    session->platform_serial.baudrate = baudrate;

    baremetal_init_timer(); // start internal timer

    if (!uxr_init_serial_transport(&session->transport_serial, &session->platform_serial, 0, 0, 0))
    {
        LOG("Error at create transport.\n");
        return false;
    }
    return true;
}

bool ros2udds::initSession(SessionUdds * session, uint32_t session_key, uxrOnTopicFunc on_topic_func, void * args)
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
        LOG("Error at create session.\n");
        return false;
    }

    session->active = true;

    // create output stream
    session->output_stream_id = uxr_create_output_reliable_stream(&session->session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // create input stream
    session->input_stream_id = uxr_create_input_reliable_stream(&session->session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    return true;
}


bool ros2udds::deleteSession(SessionUdds * session)
{
    //TODO implement session deletion
}

const char* ros2udds::getPrefixString(EntityNamePrefix prefix)
{
  return ROS2_DDS_ENTITY_PREFIX_NAME[prefix];
}