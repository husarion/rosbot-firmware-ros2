#include "ROSbot.h"
#include <debug_logs.h>
#include <BufferedSerial.h>
#include "rosbot_kinematics.h"
#include "rosbot_sensors.h"
#define BATTERY_PUB_DATA_WRITER_ID 1
#define ODOM_PUB_DATA_WRITER_ID 2
#define VELOCITY_SUB_DATA_READER_ID 1
#define MAIN_LOOP_SPIN_DELTA_TIME 10

static BufferedSerial serial(FT_SERIAL_TX, FT_SERIAL_RX,768);
static DigitalOut sens_power(SENS_POWER_ON,0);
static DigitalOut led2(LED2,0);
static DigitalOut led3(LED3,0);

volatile bool is_speed_watchdog_enabled = true;
volatile bool is_speed_watchdog_active = false;

rosbot_kinematics::RosbotOdometry odometry;
int speed_watchdog_interval = 1000; //ms
Timer odom_watchdog_timer;
volatile uint32_t last_speed_command_time=0;

static ros2udds::EntityUdds udds_tree_base[] ={
    {{1,UXR_PARTICIPANT_ID},"rosbot",nullptr,false},
    {{1,UXR_PUBLISHER_ID},nullptr,nullptr,false},
    {{1,UXR_SUBSCRIBER_ID},nullptr,nullptr,false},
};

// typedef void (*uxrOnStatusFunc) (struct uxrSession* session,
//                                  uxrObjectId object_id,
//                                  uint16_t request_id,
//                                  uint8_t status,
//                                  void* args);
extern "C"
{
    static void on_status_callback(struct uxrSession* session,
                                 uxrObjectId object_id,
                                 uint16_t request_id,
                                 uint8_t status,
                                 void* args)
    {
        (void)session;
        ROSbot * rosbot_ptr = (ROSbot*)args;
        rosbot_ptr->processOnStatus(object_id, request_id, status); 
    } 


// typedef void (*uxrOnTopicFunc) (struct uxrSession* session,
//                                 uxrObjectId object_id,
//                                 uint16_t request_id,
//                                 uxrStreamId stream_id,
//                                 struct ucdrBuffer* ub,
//                                 void* args);
    static void on_topic_callback(struct uxrSession* session,
                                uxrObjectId object_id,
                                uint16_t request_id,
                                uxrStreamId stream_id,
                                struct ucdrBuffer* ub,
                                void* args)
    {
        (void)session; (void)request_id; (void)stream_id;
        ROSbot * rosbot_ptr = (ROSbot*)args;
        rosbot_ptr->processOnTopic(object_id.id, ub);
    }
}

ROSbot::ROSbot()
: _battery_pub("battery",BATTERY_PUB_DATA_WRITER_ID)
, _pose_pub("odom",ODOM_PUB_DATA_WRITER_ID)
, _vel_sub("cmd_vel",&ROSbot::velocityCallback,this,VELOCITY_SUB_DATA_READER_ID)
, _connected(false)
, _connection_error(0)
, _drive(RosbotDrive::getInstance())
{}

ROSbot::~ROSbot(){}

void ROSbot::addEntitiesToRegister()
{
    int num = 0;
    num+=_entities_manager.addEntities(udds_tree_base,3);
    num+=_entities_manager.addEntities(&_battery_pub.topic.topic_udds);
    num+=_entities_manager.addEntities(&_battery_pub.data_writer_udds);
    num+=_entities_manager.addEntities(&_pose_pub.topic.topic_udds);
    num+=_entities_manager.addEntities(&_pose_pub.data_writer_udds);
    num+=_entities_manager.addEntities(&_vel_sub.topic.topic_udds);
    num+=_entities_manager.addEntities(&_vel_sub.data_reader_udds);
    if(num == 9)
        LOG("UDDS: Entities successfully added to register!\r\n");
    else
        LOG("UDDS: There was an error during participants registration!\r\n");
}

void ROSbot::init()
{
    ThisThread::sleep_for(1000);
    LOG("ROSbot initialization started!\r\n");
    ros2udds::initTransport(&_session,(void*)&serial,460800);
    odom_watchdog_timer.start();

    _drive.setupMotorSequence(MOTOR_FR,MOTOR_FL,MOTOR_RR,MOTOR_RL);
    _drive.init(rosbot_kinematics::custom_wheel_params,RosbotDrive::DEFAULT_REGULATOR_PARAMS);
    _drive.enable(true);
    _drive.enablePidReg(true);

    addEntitiesToRegister();

    // battery message initialization
    _battery_pub.topic.power_supply_health = sensor_msgs::BatteryHealth::POWER_SUPPLY_HEALTH_UNKNOWN;
    _battery_pub.topic.power_supply_status = sensor_msgs::BatteryStatus::POWER_SUPPLY_STATUS_UNKNOWN;
    _battery_pub.topic.power_supply_technology = sensor_msgs::BatteryTechnology::POWER_SUPPLY_TECHNOLOGY_LION;
    _battery_pub.topic.present = true;

    // odom message initialization
    strcpy(_pose_pub.topic.header.frame_id,"odom");
    _pose_pub.topic.pose.position.x = 0;
    _pose_pub.topic.pose.position.y = 0;
    _pose_pub.topic.pose.position.z = 0;
    _pose_pub.topic.pose.orientation.x = 0;
    _pose_pub.topic.pose.orientation.y = 0;
    _pose_pub.topic.pose.orientation.z = 0;
    _pose_pub.topic.pose.orientation.w = 1;
}

bool ROSbot::restoreCommunication()
{
    bool res = ros2udds::initSession(&_session,0xAABBCCDD, on_topic_callback, this);
    if(res)
    {
        LOG("Session initialization was successfull!\r\n");
        if(res = _entities_manager.registerEntities(&_session))
        {
            LOG("All entities registered successfully!\r\n");

            // subscribtion
            uint16_t request_id;
            _vel_sub.subscribe(&_session, &request_id);
        }
        else
        {
            LOG("Problems with entities registration!\r\n");
        }
    }
    return res;
}

void ROSbot::processOnTopic(uint16_t id, ucdrBuffer* ub)
{
    if(id == _vel_sub.data_reader_udds.id.id)
        _vel_sub.call(ub);
}

void ROSbot::processOnStatus(uxrObjectId object_id, uint16_t request_id, uint8_t status)
{

}

void ROSbot::velocityCallback(const geometry_msgs::Twist &msg)
{
    RosbotDrive & drive = RosbotDrive::getInstance();
    rosbot_kinematics::setRosbotSpeed(drive,msg.linear.x, msg.angular.z);
    last_speed_command_time = odom_watchdog_timer.read_ms();
    is_speed_watchdog_active = false;
}

void ROSbot::processCommunicationStatus(bool status)
{
    if(status)
    {
        _connection_error = 0;
        _connected = true;
    }
    else
    {
        if(++_connection_error > 10) _connected = false;
    }
}

void ROSbot::spin()
{
    uint32_t spin_cnt = 0;
    uint64_t current_time, last_time=0;

    while (1)
    {
        current_time = Kernel::get_ms_count();

        if(is_speed_watchdog_enabled)
        {
            if(!is_speed_watchdog_active && (odom_watchdog_timer.read_ms() - last_speed_command_time) > speed_watchdog_interval)
            {
                rosbot_kinematics::setRosbotSpeed(_drive, 0.0f, 0.0f);
                is_speed_watchdog_active = true;
            }
        }

        if(spin_cnt % 2 == 0)
        {
            float dtime = current_time - last_time / 1000.0f;
            rosbot_kinematics::updateRosbotOdometry(_drive, odometry, dtime);
        }

        if (_connected)
        {
            if (spin_cnt % 100 == 0)
            {
                _battery_pub.topic.voltage = rosbot_sensors::updateBatteryWatchdog();
                _battery_pub.publish(&_session);
            }
            if (spin_cnt % 5 == 0)
            {
                _pose_pub.topic.pose.position.x = odometry.odom.robot_x_pos;
                _pose_pub.topic.pose.position.y = odometry.odom.robot_y_pos;
                _pose_pub.topic.pose.orientation.z = sin(odometry.odom.robot_angular_pos * 0.5);
                _pose_pub.topic.pose.orientation.w = cos(odometry.odom.robot_angular_pos * 0.5);
                _pose_pub.publish(&_session);
            }
            processCommunicationStatus(uxr_run_session_until_confirm_delivery(&_session.session, 10));
        }
        else
        {
            //try to restore communication every 5 seconds
            if (spin_cnt % 500 == 0) _connected = restoreCommunication();
        }
        ThisThread::sleep_until(current_time + MAIN_LOOP_SPIN_DELTA_TIME);
        spin_cnt++;
    }
}