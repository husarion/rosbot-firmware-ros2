#ifndef __ROSBOT_H__
#define __ROSBOT_H__

#include <mbed.h>
#include <RosbotDrive.h>
#include <rcl.h>
#include <sensor_msgs/BatteryState.hpp>
#include <geometry_msgs/Twist.hpp>
#include <geometry_msgs/PoseStamped.hpp>
#include <geometry_msgs/TransformStamped.hpp>
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

    private:
        ros2udds::SessionHandle _session;
        ros2udds::Publisher<sensor_msgs::BatteryState> _battery_pub;
        ros2udds::Publisher<geometry_msgs::PoseStamped> _pose_pub;
        ros2udds::Publisher<geometry_msgs::TransformStamped> _transform_pub;
        ros2udds::Subscriber<geometry_msgs::Twist, ROSbot> _vel_sub;

        bool _connected;
        int _connection_error;
        int _entities_cnt;
        rosbot_kinematics::RosbotOdometry _odometry;
    private:
        RosbotDrive & _drive;
};

#endif /* __ROSBOT_H__ */