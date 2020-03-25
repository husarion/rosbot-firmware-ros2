// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


/*! 
 * @file TransformStamped.hpp
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */
#ifndef _GEOMETRY_MSGS_TRANSFORM_STAMPED_HPP_
#define _GEOMETRY_MSGS_TRANSFORM_STAMPED_HPP_


#include "../topic.hpp"

#include "../std_msgs/Header.hpp"
#include "../geometry_msgs/Transform.hpp"

namespace geometry_msgs {


class TransformStamped : public ros2udds::Topic<TransformStamped>
{
public: 
    std_msgs::Header header;
    char child_frame_id[64];
    geometry_msgs::Transform transform;

  TransformStamped():
    Topic("geometry_msgs::msg::dds_::TransformStamped_", "TransformStamped", GEOMETRY_MSGS_TRANSFORM_STAMPED_ID),
    header(), transform()
  { 
    memset(child_frame_id, 0, sizeof(child_frame_id));
  }

  bool serialize(ucdrBuffer* msg_buf, const TransformStamped* topic)
  {
    ucdrBuffer* writer = (ucdrBuffer*)msg_buf;
    (void) header.serialize(writer, &topic->header);
    (void) ucdr_serialize_string(writer, topic->child_frame_id);
    (void) transform.serialize(writer, &topic->transform);

    return !writer->error;
  }

  bool deserialize(ucdrBuffer* msg_buf, TransformStamped* topic)
  {
    ucdrBuffer* reader = (ucdrBuffer*)msg_buf;
    (void) header.deserialize(reader, &topic->header);
    (void) ucdr_deserialize_string(reader, topic->child_frame_id, sizeof(topic->child_frame_id));
    (void) transform.deserialize(reader, &topic->transform);
    
    return !reader->error;
  }

  uint32_t size_of_topic(const TransformStamped* topic, uint32_t size)
  {
    uint32_t previousSize = size;
    size += header.size_of_topic(&topic->header, size);
    size += ucdr_alignment(size, 4) + 4 + (uint32_t)(strlen(topic->child_frame_id) + 1);
    size += transform.size_of_topic(&topic->transform, size);

    return size - previousSize;
  }

};

} // namespace geometry_msgs


#endif // _GEOMETRY_MSGS_TRANSFORM_STAMPED_HPP_
