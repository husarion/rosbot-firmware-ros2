/*
 * topic.hpp
 *
 *  Created on: May 16, 2018
 *      Author: Kei
 *      Repository: https://github.com/ROBOTIS-GIT/ros2arduino
 *      License: Apache 2.0
 */

#ifndef ROS2_TOPIC_HPP_
#define ROS2_TOPIC_HPP_

#include "ros2udds.h"
#include "topic_id_number.h"

namespace ros2udds
{

/* Base Message Type */
template <typename MsgT>
class Topic
{

public:
    Topic(const char *type, const char *name, uint8_t id)
    : type_(type)
    , name_(name)
    , id_(id)
    {}

    virtual ~Topic(){}
    virtual bool serialize(ucdrBuffer *msg_buf, const MsgT *topic) = 0;
    virtual bool deserialize(ucdrBuffer *msg_buf, MsgT *topic) = 0;
    virtual uint32_t size_of_topic(const MsgT *topic, uint32_t size)
    {
        (void)(topic);
        (void)(size);

        return 0;
    }

    const char *type_;
    const char *name_;
    uint16_t id_;
};

} // namespace ros2udds

#endif /* ROS2_TOPIC_HPP_ */
