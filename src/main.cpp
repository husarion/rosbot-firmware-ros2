/** @file main.cpp
 * ROSbot ROS2 firmware using Mbed OS and eProsima Micro XRCE-DDS.
 *
 * @date 3-13-2020
 * @author byq77
 * @version 0.1.0
 */
#include <ROSbot.h>

int main()
{
    ROSbot rosbot;
    rosbot.init();
    rosbot.spin();
}