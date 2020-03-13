#include "ROSbot.h"
#include <debug_logs.h>
#include <BufferedSerial.h>
#include "rosbot_sensors.h"
#define MAIN_LOOP_SPIN_DELTA_TIME 10

// static BufferedSerial serial(FT_SERIAL_TX, FT_SERIAL_RX, 2048,2);
static BufferedSerial serial(RPI_SERIAL_TX, RPI_SERIAL_RX, 2048,2);
static DigitalOut sens_power(SENS_POWER_ON,0);
static DigitalOut led2(LED2,0);
static DigitalOut led3(LED3,0);

volatile bool is_speed_watchdog_enabled = true;
volatile bool is_speed_watchdog_active = false;

int speed_watchdog_interval = 1000; //ms
Timer odom_watchdog_timer;
volatile uint32_t last_speed_command_time=0;

static ros2udds::Entity udds_tree_base[] ={
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
: _battery_pub(&_session,"battery")
, _pose_pub(&_session,"odom")
, _transform_pub(&_session,"/tf")
, _vel_sub(&_session,"cmd_vel",&ROSbot::velocityCallback,this)
, _connected(false)
, _connection_error(0)
, _entities_cnt(0)
, _drive(RosbotDrive::getInstance())
{}

ROSbot::~ROSbot(){}

void ROSbot::addEntitiesToRegister()
{
    int num = 0;
    for(int i=0;i<3;i++) num+=ros2udds::addEntity(&_session, &udds_tree_base[i]);
    num+=_battery_pub.addToRegister();
    num+=_pose_pub.addToRegister();
    num+=_vel_sub.addToRegister();
    num+=_transform_pub.addToRegister();
    _entities_cnt = num;
    LOG("%d entites added to register\r\n",num);    
}

void ROSbot::init()
{
    ThisThread::sleep_for(1000);
    LOG("ROSbot initialization started!\r\n");
    ros2udds::initTransport(&_session,(void*)&serial,DDS_SERIAL_BAUDRATE);
    odom_watchdog_timer.start();

    _drive.setupMotorSequence(MOTOR_FR,MOTOR_FL,MOTOR_RR,MOTOR_RL);
    _drive.init(rosbot_kinematics::custom_wheel_params,RosbotDrive::DEFAULT_REGULATOR_PARAMS);
    _drive.enable(true);
    _drive.enablePidReg(true);

    addEntitiesToRegister();

    // battery message initialization
    _battery_pub.topic.cell_voltage_size = 3;
    _battery_pub.topic.cell_temperature_size = 3;
    _battery_pub.topic.power_supply_health = sensor_msgs::BatteryHealth::POWER_SUPPLY_HEALTH_UNKNOWN;
    _battery_pub.topic.power_supply_status = sensor_msgs::BatteryStatus::POWER_SUPPLY_STATUS_UNKNOWN;
    _battery_pub.topic.power_supply_technology = sensor_msgs::BatteryTechnology::POWER_SUPPLY_TECHNOLOGY_LION;
    _battery_pub.topic.present = true;

    // odom message initialization
    strcpy(_pose_pub.topic.header.frame_id,"odom");
    _pose_pub.topic.pose.position.x = 0.0;
    _pose_pub.topic.pose.position.y = 0.0;
    _pose_pub.topic.pose.position.z = 0.0;
    _pose_pub.topic.pose.orientation.x = 0.0;
    _pose_pub.topic.pose.orientation.y = 0.0;
    _pose_pub.topic.pose.orientation.z = 0.0;
    _pose_pub.topic.pose.orientation.w = 1.0;

    strcpy(_transform_pub.topic.header.frame_id, "odom"); 
    strcpy(_transform_pub.topic.child_frame_id, "base_link");
    _transform_pub.topic.transform.translation.x = 0.0;
    _transform_pub.topic.transform.translation.y = 0.0;
    _transform_pub.topic.transform.translation.z = 0.0;
    _transform_pub.topic.transform.rotation.x = 0.0;
    _transform_pub.topic.transform.rotation.y = 0.0;
    _transform_pub.topic.transform.rotation.z = 0.0;
    _transform_pub.topic.transform.rotation.w = 1.0;
    memset(&_odometry.buffor,0,sizeof(float)*sizeof(_odometry.buffor[0])); //TODO: odometry module
}

bool ROSbot::restoreCommunication()
{
    if(ros2udds::initSession(&_session, 0xAABBCCDD, on_topic_callback, this))
    {
        LOG("Session initialization was successfull!\r\n");
        if(_entities_cnt == ros2udds::registerEntities(&_session))
        {
            LOG("All entities registered successfully!\r\n");

            // subscribtion
            uint16_t request_id;
            _vel_sub.subscribe(&request_id);
            return true;
        }
        else
        {
            LOG("Problems with entities registration!\r\n");
        }
    }
    return false;
}

void ROSbot::processOnTopic(uint16_t id, ucdrBuffer* ub)
{
    if(id == _vel_sub.getId()) _vel_sub.call(ub);
}

void ROSbot::processOnStatus(uxrObjectId object_id, uint16_t request_id, uint8_t status)
{

}

void ROSbot::velocityCallback(const geometry_msgs::Twist &msg)
{
    rosbot_kinematics::setRosbotSpeed(_drive,msg.linear.x, msg.angular.z);
    last_speed_command_time = odom_watchdog_timer.read_ms();
    is_speed_watchdog_active = false;
    LOG("Twist Vx: %.2f Wz: %.2f\r\n",msg.linear.x, msg.angular.z);
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
    uint64_t current_time; 
    float current_time_odom, last_time_odom=0.0f;

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
            current_time_odom = odom_watchdog_timer.read();
            rosbot_kinematics::updateRosbotOdometry(_drive, _odometry, current_time_odom - last_time_odom);
            last_time_odom = current_time_odom;
        }

        if (_connected)
        {
            if (spin_cnt % 100 == 0)
            {
                _battery_pub.topic.voltage = rosbot_sensors::updateBatteryWatchdog();
                _battery_pub.publish();
            }
            
            if (spin_cnt % 5 == 0)
            {
                _pose_pub.topic.pose.position.x = _odometry.odom.robot_x_pos;
                _pose_pub.topic.pose.position.y = _odometry.odom.robot_y_pos;
                _pose_pub.topic.pose.orientation.z = sin(_odometry.odom.robot_angular_pos * 0.5);
                _pose_pub.topic.pose.orientation.w = cos(_odometry.odom.robot_angular_pos * 0.5);
                _pose_pub.publish();
            }
            processCommunicationStatus(uxr_run_session_time(&_session.session, 5));
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