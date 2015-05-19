#include "blackboard/blackboard.h"

#include <ros/node_handle.h>
#include <blackboard/KeyUpdate.h>
#include <blackboard/update.h>

//#include <boost/thread/lock_types.hpp>

#include <ros/master.h>

namespace bb
{

// ----------------------------------------------------------------------------------------------------

void Blackboard::initialize(const std::string& name)
{
    ros::NodeHandle nh(name);
    pub_key_ = nh.advertise<blackboard::KeyUpdate>("bb/keys", 1, true); // true = latching is true

//    // Find other blackboards
//    ros::master::V_TopicInfo topic_infos;
//    ros::master::getTopics(topic_infos);

//    for(ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic)
//    {
//        const ros::master::TopicInfo& topic_info = *it_topic;

//        if (topic_info.datatype == "blackboard/KeyUpdate")
//        {
//            std::cout << topic_info.name << std::endl;
//        }
//    }
}

// ----------------------------------------------------------------------------------------------------

void Blackboard::update(const Update& update)
{
    const std::map<Key, std::vector<KeyUpdate> >& updates = update.updates();

    {
        // get upgradable access
        boost::upgrade_lock<boost::shared_mutex> lock(blackboard_mutex_);

        // get exclusive access
        boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);

        // Update values
        for(std::map<Key, std::vector<KeyUpdate> >::const_iterator it = updates.begin(); it != updates.end(); ++it)
        {
            Data& d = data_[it->first];

            const std::vector<KeyUpdate>& key_updates = it->second;
            for(std::vector<KeyUpdate>::const_iterator it2 = key_updates.begin(); it2 != key_updates.end(); ++it2)
            {
                const KeyUpdate& kup = *it2;
                d.buffer.insert(kup.timestamp, kup.value);
            }
        }

    }

    // Check triggers
    for(std::map<Key, std::vector<KeyUpdate> >::const_iterator it = updates.begin(); it != updates.end(); ++it)
    {
        const std::vector<KeyUpdate>& key_updates = it->second;
        for(std::vector<KeyUpdate>::const_iterator it2 = key_updates.begin(); it2 != key_updates.end(); ++it2)
        {
            const KeyUpdate& kup = *it2;
            checkTriggers(it->first, kup.timestamp, kup.value);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Blackboard::checkTriggers(Key key, Time t, const Variant& value)
{
    Data& d = data_[key];

    // Call triggers
    const std::vector<trigger_function>& trigger_functions = d.trigger_functions;
    for(std::vector<trigger_function>::const_iterator it = trigger_functions.begin(); it != trigger_functions.end(); ++it)
    {
        (*it)(*this, key);
    }

    if (d.pub.getNumSubscribers() > 0 && d.serializer)
    {
        blackboard::ValueUpdate msg;
        msg.timestamp = t;
        ROSWBytes bytes(msg);
        d.serializer->serialize(value, bytes);
        d.pub.publish(msg);
    }

    updateConnections();  // Random location; TODO
}

} // end namespace bb

