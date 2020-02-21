#ifndef __ENTITIES_MANAGER_H__
#define __ENTITIES_MANAGER_H__

#include "ros2udds.h"

#define UDDS_PARTICIPANT_INDEX 0
#define UDDS_PUBLISHER_INDEX 1
#define UDDS_SUBSCRIBER_INDEX 2
#define UDDS_TOPICS_START_INDEX 3

#define UDDS_MAX_TOPICS 20
#define UDDS_MAX_DATA_READERS 10
#define UDDS_MAX_DATA_WRITERS 10

#define UDDS_DATA_READERS_START_INDEX (UDDS_TOPICS_START_INDEX + UDDS_MAX_TOPICS)
#define UDDS_DATA_WRITERS_START_INDEX (UDDS_DATA_READERS_START_INDEX + UDDS_MAX_DATA_READERS)  

#define UDDS_ARRAY_SIZE (3 + UDDS_MAX_TOPICS + UDDS_MAX_DATA_WRITERS + UDDS_MAX_DATA_READERS)

namespace ros2udds
{
    class EntitiesManager
    {
        public:
            EntitiesManager();
            virtual ~EntitiesManager(){};

            int addEntities(EntityUdds * entities, size_t length=1);
            void removeEntity(EntityUdds * entity);
            bool registerEntities(SessionUdds * session);
            bool unregisterEntities(SessionUdds * session);
            
        private:
            static int findEmpty(EntityUdds **data, int start_index, int stop_index)
            {
                for(int i=start_index; i <= stop_index; i++)
                    if(data[i] == nullptr) return i;
                return -1;
            }
            
            int _entity_cnt;
            EntityUdds * _udds_entities[UDDS_ARRAY_SIZE];
    };
} // ros2udds

#endif /* __ENTITIES_MANAGER_H__ */