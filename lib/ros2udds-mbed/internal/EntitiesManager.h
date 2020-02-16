#ifndef __ENTITIES_MANAGER_H__
#define __ENTITIES_MANAGER_H__

#include "udds_session_helper.h"

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

namespace udds
{
    class EntitiesManager
    {
        public:

            struct Entity
            {
                uxrObjectId id;
                const char * name;
                const char * topic_type;
                bool active;
            };

            EntitiesManager();
            virtual ~EntitiesManager(){};

            bool addEntity(Entity * entity);
            bool addEntities(Entity *, size_t length);
            bool removeEntity(Entity * entity);
            bool registerEntities(udds_session_t * session);
            bool unregisteEntities(udds_session_t * session);


        private:
            static int findEmpty(Entity **data, int start_index, int stop_index)
            {
                for(int i=start_index; i <= stop_index; i++)
                {
                    if(data[i] == nullptr) return i;
                }
                return -1;
            }
            
            int _entity_cnt;
            Entity * _udds_entities[UDDS_ARRAY_SIZE];
    };
}

#endif /* __ENTITIES_MANAGER_H__ */