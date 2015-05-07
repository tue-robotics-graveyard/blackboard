#include "blackboard/blackboard.h"

#include <ros/node_handle.h>
#include <blackboard/KeyUpdate.h>

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

} // end namespace bb

