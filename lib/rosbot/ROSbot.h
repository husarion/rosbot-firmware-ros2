#ifndef __ROSBOT_H__
#define __ROSBOT_H__

#include <mbed.h>
#include <ros2udds.h>
#include <EntitiesManager.h>
#include <publisher.h>
#include <BatteryState.hpp>

class ROSbot
{
    public:
        ROSbot();
        ~ROSbot();
        void init();
        void spin();
    private:
        void addEntitiesToRegister();
        bool restoreCommunication();

    private:
        ros2udds::SessionUdds _session;
        ros2udds::EntitiesManager _entities_manager;
        ros2udds::Publisher<sensor_msgs::BatteryState> _battery_pub;
        bool _connected;
};

#endif /* __ROSBOT_H__ */