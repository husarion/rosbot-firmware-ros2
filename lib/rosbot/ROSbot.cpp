#include "ROSbot.h"
#include <debug_logs.h>
#include <BufferedSerial.h>
#include "rosbot_sensors.h"
#include <baremetal_time.h>
#include <inttypes.h>

#define MAIN_LOOP_SPIN_DELTA_TIME 10

#ifndef DDS_USB_SERIAL
    #define JSON_NANO_SERIAL 0
#endif

#ifndef DDS_RPI_SERIAL
    #define DDS_RPI_SERIAL 0
#endif

#if DDS_USB_SERIAL == 1
static BufferedSerial serial(FT_SERIAL_TX, FT_SERIAL_RX, 2048,2);
#elif DDS_RPI_SERIAL == 1
static BufferedSerial serial(RPI_SERIAL_TX, RPI_SERIAL_RX, 2048,2);
#endif 

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

// typedef void (*uxrOnTimeFunc) (struct uxrSession* session,
//                                int64_t current_timestamp,
//                                int64_t transmit_timestamp,
//                                int64_t received_timestamp,
//                                int64_t originate_timestamp,
//                                void* args);
    static void on_time_callback(struct uxrSession *session,
                                 int64_t current_timestamp,
                                 int64_t transmit_timestamp,
                                 int64_t received_timestamp,
                                 int64_t originate_timestamp,
                                 void *args)
    {
        (void)session;
        ROSbot * rosbot_ptr = (ROSbot *)args;
        rosbot_ptr->processOnTime(current_timestamp, transmit_timestamp, received_timestamp, originate_timestamp);
    }
}

ROSbot::ROSbot()
: _battery_pub(&_session,"battery")
, _pose_pub(&_session,"odom")
, _transform_pub(&_session,"tf")
, _vel_sub(&_session,"cmd_vel",&ROSbot::velocityCallback,this)
, _time_sub(&_session,"rosbot_time",&ROSbot::syncTimeFromRemote,this)
, _connected(false)
, _connection_error(0)
, _entities_cnt(0)
, _ms_when_synced_time(0)
, _drive(RosbotDrive::getInstance())
{
    for(int i=0;i<USER_ROS2_PUBLISHER_MAX; i++) _publishers[i] = nullptr;
    for(int i=0;i<USER_ROS2_SUBSCRIBER_MAX; i++) _subscribers[i] = nullptr;
    int i = 0;
    _publishers[i] = &_battery_pub; _subscribers[i++] = &_vel_sub;
    _publishers[i++] = &_pose_pub;
    _publishers[i++] = &_transform_pub;
}

ROSbot::~ROSbot(){}

void ROSbot::addEntitiesToRegister()
{
    int num = 0;
    for(int i=0;i<3;i++) num+=ros2udds::addEntity(&_session, &udds_tree_base[i]);

    for(int i =0; i < USER_ROS2_PUBLISHER_MAX; i++) 
        if(_publishers[i] != nullptr) num+=_publishers[i]->addToRegister();
    for(int i =0; i < USER_ROS2_SUBSCRIBER_MAX; i++) 
        if(_subscribers[i] != nullptr) num+=_subscribers[i]->addToRegister();

    _entities_cnt = num;
    LOG("%d entites added to register\r\n",num);    
}

void ROSbot::processOnTime(int64_t current_timestamp,
                          int64_t transmit_timestamp,
                          int64_t received_timestamp,
                          int64_t originate_timestamp)
{
    LOG("current_timestamp: %lld\r\n", current_timestamp);
    LOG("transmit_timestamp: %lld\r\n", transmit_timestamp);
    LOG("received_timestamp: %lld\r\n", received_timestamp);
    LOG("originate_timestamp: %lld\r\n", originate_timestamp);
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

    strcpy(_transform_pub.topic.transforms.header.frame_id, "odom"); 
    strcpy(_transform_pub.topic.transforms.child_frame_id, "base_link");
    _transform_pub.topic.transforms.transform.translation.x = 0.0;
    _transform_pub.topic.transforms.transform.translation.y = 0.0;
    _transform_pub.topic.transforms.transform.translation.z = 0.0;
    _transform_pub.topic.transforms.transform.rotation.x = 0.0;
    _transform_pub.topic.transforms.transform.rotation.y = 0.0;
    _transform_pub.topic.transforms.transform.rotation.z = 0.0;
    _transform_pub.topic.transforms.transform.rotation.w = 1.0;
    memset(&_odometry.buffor,0,sizeof(float)*sizeof(_odometry.buffor[0])); //TODO: odometry module
}

bool ROSbot::restoreCommunication()
{    
    if(ros2udds::initSession(&_session, 0xAABBCCDD))
    {
        LOG("Session initialization was successfull!\r\n");
        ros2udds::registerSessionCallbacks(&_session,on_topic_callback,nullptr,(void*)this);
        if(_entities_cnt == ros2udds::registerEntities(&_session))
        {
            LOG("All entities registered successfully!\r\n");
            LOG("Synchronize time using NTP\r\n");
            uxr_sync_session(&_session.session, 5);
            // subscribtion
            uint16_t request_id[USER_ROS2_SUBSCRIBER_MAX];
            for(int i =0; i < USER_ROS2_SUBSCRIBER_MAX; i++) 
                if(_subscribers[i] != nullptr) _subscribers[i]->subscribe(request_id+i);
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
    for(int i =0; i < USER_ROS2_SUBSCRIBER_MAX; i++) 
    {
        if(_subscribers[i] && id == _subscribers[i]->getId())
        {
            _subscribers[i]->call(ub);
            break;
        } 
    }
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
            // if(spin_cnt % 500 == 0)
            // {
            //     if(uxr_sync_session(&_session.session, 5))
            //     {
            //         LOG("new time_offset: %lld\r\n", _session.session.time_offset);
            //     }
            // }

            if (spin_cnt % 100 == 0)
            {
                _battery_pub.topic.voltage = rosbot_sensors::updateBatteryWatchdog();
                _battery_pub.topic.header.stamp = now();
                _battery_pub.publish();
            }
            
            if (spin_cnt % 5 == 0)
            {
                _transform_pub.topic.transforms.header.stamp = _pose_pub.topic.header.stamp = now();
                _transform_pub.topic.transforms.transform.translation.x = _pose_pub.topic.pose.position.x = _odometry.odom.robot_x_pos;
                _transform_pub.topic.transforms.transform.translation.y = _pose_pub.topic.pose.position.y = _odometry.odom.robot_y_pos;
                _transform_pub.topic.transforms.transform.rotation.z = _pose_pub.topic.pose.orientation.z = sin(_odometry.odom.robot_angular_pos * 0.5);
                _transform_pub.topic.transforms.transform.rotation.w = _pose_pub.topic.pose.orientation.w = cos(_odometry.odom.robot_angular_pos * 0.5);
                _pose_pub.publish();
                _transform_pub.publish();
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

builtin_interfaces::Time ROSbot::now()
{
#if defined(NTP_TIME_SYNCH)
  builtin_interfaces::Time ret_time;
  int64_t nanos = uxr_epoch_nanos(&_session.session);
  ret_time.sec = (int32_t)(nanos / 1000000000);
  ret_time.nanosec = (uint32_t)(nanos - ret_time.sec * 1000000000);
#else
  builtin_interfaces::Time ret_time = _synced_time_from_remote;
  int64_t msec_offset = baremetal_millis() - _ms_when_synced_time;
  uint32_t remain_msec = msec_offset%1000;

  ret_time.sec += (int32_t)(msec_offset/1000);
  if((ret_time.nanosec/(uint32_t)1000000 + remain_msec) < 1000)
  {
    ret_time.nanosec += (uint32_t)(remain_msec*(uint32_t)1000000);
  }
  else // >= 1sec
  {
    ret_time.sec += 1;
    ret_time.nanosec = (uint32_t)(ret_time.nanosec + remain_msec*(uint32_t)1000000 - (uint32_t)1000000000);
  }
#endif

  return ret_time;
}

//deprecated
void ROSbot::syncTimeFromRemote(const builtin_interfaces::Time & msg)
{
  _ms_when_synced_time             = baremetal_millis();
  _synced_time_from_remote.sec     = msg.sec;
  _synced_time_from_remote.nanosec = msg.nanosec;
}
