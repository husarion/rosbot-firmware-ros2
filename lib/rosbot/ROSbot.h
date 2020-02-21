#ifndef __ROSBOT_H__
#define __ROSBOT_H__

#include <mbed.h>
#include <RosbotDrive.h>
#include <ros2udds.h>
#include <EntitiesManager.h>
#include <publisher.h>
#include <subscriber.h>
#include <sensor_msgs/BatteryState.hpp>
#include <geometry_msgs/Twist.hpp>
#include <geometry_msgs/PoseStamped.hpp>

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
        ros2udds::SessionUdds _session;
        ros2udds::EntitiesManager _entities_manager;
        ros2udds::Publisher<sensor_msgs::BatteryState> _battery_pub;
        ros2udds::Publisher<geometry_msgs::PoseStamped> _pose_pub;
        ros2udds::Subscriber<geometry_msgs::Twist, ROSbot> _vel_sub;

        bool _connected;
        int _connection_error;
    private:
        RosbotDrive & _drive;
};

#endif /* __ROSBOT_H__ */