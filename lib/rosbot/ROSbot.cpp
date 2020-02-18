#include "ROSbot.h"
#include <debug_logs.h>
#include <BufferedSerial.h>

static BufferedSerial serial(FT_SERIAL_TX, FT_SERIAL_RX,768);

#define BATTERY_PUB_DATA_WRITER_ID 1

static ros2udds::EntityUdds udds_tree_base[] ={
    {{1,UXR_PARTICIPANT_ID},"rosbot",nullptr,false},
    {{1,UXR_PUBLISHER_ID},nullptr,nullptr,false},
    {{1,UXR_SUBSCRIBER_ID},nullptr,nullptr,false},
};

ROSbot::ROSbot()
: _battery_pub("battery",BATTERY_PUB_DATA_WRITER_ID)
, _connected(false)
{}

ROSbot::~ROSbot(){}

void ROSbot::addEntitiesToRegister()
{
    int num = 0;
    if(_entities_manager.addEntities(udds_tree_base,3)) num+=3;
    if(_entities_manager.addEntity(&_battery_pub.topic.topic_udds)) num++;
    if(_entities_manager.addEntity(&_battery_pub.data_writer_udds)) num++;
    if(num == 5)
        LOG("UDDS: Entities successfully added to register!\r\n");
    else
        LOG("UDDS: There was an error during participants registration!\r\n");
}

void ROSbot::init()
{
    LOG("ROSbot initialization started!");
    ros2udds::initTransport(&_session,(void*)&serial,460800);
    addEntitiesToRegister();
}

bool ROSbot::restoreCommunication()
{
    bool res = ros2udds::initSession(&_session,0xAABBCCDD);
    if(res)
    {
        LOG("Session initialization was successfull!\r\n");
        if(res = _entities_manager.registerEntities(&_session))
        {
            LOG("All entities registered successfully!\r\n");
        }
        else
        {
            LOG("Problems with entities registration!\r\n");
        }
    }
    return res;
}

void ROSbot::spin()
{
    uint32_t spin_cnt = 0;
    uint16_t request_list[20];
    uint8_t status_list[20];
    while (1)
    {
        if (_connected)
        {
            if (spin_cnt % 100 == 0)
            {
                _battery_pub.topic.voltage = 12.0f;
                _battery_pub.publish(&_session);
            }
            uxr_run_session_until_all_status(&_session.session, 5, request_list, status_list, 10);
            ThisThread::sleep_for(10);
        }
        else
        {
            //try to restore communication every 5 seconds
            _connected = restoreCommunication();
            if(!_connected) ThisThread::sleep_for(5000);
        }
        spin_cnt++;
    }
}