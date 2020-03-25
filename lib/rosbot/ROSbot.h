#ifndef __ROSBOT_H__
#define __ROSBOT_H__

#include <mbed.h>
#include <RosbotDrive.h>
#include <rcl.h>
#include <sensor_msgs/BatteryState.hpp>
#include <geometry_msgs/Twist.hpp>
#include <geometry_msgs/PoseStamped.hpp>
#include <geometry_msgs/TransformStamped.hpp>
#include <tf2_msgs/TFMessage.hpp>
#include "rosbot_kinematics.h"

class ROSbot
{
    public:
        ROSbot();
        ~ROSbot();
        void init();
        void spin();
        void processOnTopic(uint16_t id, ucdrBuffer* ub);
        void processOnStatus(uxrObjectId object_id, uint16_t request_id, uint8_t status);

    private:
        void processCommunicationStatus(bool status);
        void addEntitiesToRegister();
        bool restoreCommunication();
        void velocityCallback(const geometry_msgs::Twist &msg);
        void syncTimeFromRemote(const builtin_interfaces::Time & msg);
        builtin_interfaces::Time now();

    private:
        ros2udds::SessionHandle _session;
        ros2udds::Publisher<sensor_msgs::BatteryState> _battery_pub;
        ros2udds::Publisher<geometry_msgs::PoseStamped> _pose_pub;
        ros2udds::Publisher<tf2_msgs::TFMessage> _transform_pub;
        ros2udds::Subscriber<geometry_msgs::Twist, ROSbot> _vel_sub;
        ros2udds::Subscriber<builtin_interfaces::Time, ROSbot> _time_sub;

        bool _connected;
        int _connection_error;
        int _entities_cnt;
        int64_t _us_when_synced_time;
        rosbot_kinematics::RosbotOdometry _odometry;
        builtin_interfaces::Time _synced_time_from_remote;

    private:
        RosbotDrive & _drive;
};

#endif /* __ROSBOT_H__ */