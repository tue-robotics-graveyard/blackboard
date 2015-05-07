#ifndef BLACKBOARD_BLACKBOARD_H_
#define BLACKBOARD_BLACKBOARD_H_

#include "blackboard/key.h"
#include "blackboard/variant.h"

#include <vector>
#include <map>
#include <string>
#include <iostream>

// ROS
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <blackboard/ValueUpdate.h>
#include <blackboard/KeyUpdate.h>

namespace bb
{

class Blackboard;
typedef void (*trigger_function)(const Blackboard&);

struct Data
{
    std::vector<trigger_function> trigger_functions;
    Variant value;
    ros::Publisher pub;
    std::map<std::string, ros::Subscriber> subs;
};

struct BlackboardConnection
{
    ros::Subscriber sub_keys;
};

class Blackboard
{

public:

    Blackboard()
    {
    }

    ~Blackboard()
    {

    }

    Key getKey(const char* name)
    {
        std::map<std::string, Key>::iterator it = key_map_.find(name);
        if (it != key_map_.end())
            return it->second;

        Key key = key_map_.size();
        key_map_[name] = key;
        data_.push_back(Data());

        Data& d = data_.back();

        ros::NodeHandle nh("~");
        d.pub = nh.advertise<blackboard::ValueUpdate>("bb/data/" + std::string(name), 1);

        sendKeys();

        return key;
    }

    Key findKey(const char* name) const
    {
        std::map<std::string, Key>::const_iterator it = key_map_.find(name);
        if (it != key_map_.end())
            return it->second;

        return -1;
    }

    void addTrigger(Key key, trigger_function func)
    {
        data_[key].trigger_functions.push_back(func);
    }

    template<typename T>
    void setValue(Key key, T value)
    {
        Data& d = data_[key];

        // Set value
        d.value.setValue<T>(value);

        // Call triggers
        const std::vector<trigger_function>& trigger_functions = d.trigger_functions;
        for(std::vector<trigger_function>::const_iterator it = trigger_functions.begin(); it != trigger_functions.end(); ++it)
        {
            (*it)(*this);
        }

        if (d.pub.getNumSubscribers() > 0)
        {
            blackboard::ValueUpdate msg;
            msg.data = "test";
            d.pub.publish(msg);
        }

        updateConnections();  // Random location; TODO
    }

    template<typename T>
    const T& getValue(Key key) const
    {
        const Data& d = data_[key];
        return d.value.getValue<T>();        
    }

//    void initCommunication(const std::string& name = "~");

    void sendKeys()
    {
        blackboard::KeyUpdate msg;

        for(std::map<std::string, Key>::const_iterator it = key_map_.begin(); it != key_map_.end(); ++it)
            msg.added_keys.push_back(it->first);

        pub_key_.publish(msg);
    }

    void initialize(const std::string& name = "~");

    void addExternal(const std::string& bb_id)
    {
        std::map<std::string, BlackboardConnection>::iterator it = connections_.find(bb_id);
        if (it != connections_.end())
            return;

        BlackboardConnection& c = connections_[bb_id];

        ros::NodeHandle nh;
        nh.setCallbackQueue(&key_cb_queue_);
        c.sub_keys = nh.subscribe<blackboard::KeyUpdate>(bb_id + "/bb/keys", 1, boost::bind(&Blackboard::cbKeyInfo, this, _1, bb_id));
    }

    void cbKeyInfo(const blackboard::KeyUpdateConstPtr& msg, const std::string& bb_id)
    {
        for(std::vector<std::string>::const_iterator it = msg->added_keys.begin(); it != msg->added_keys.end(); ++it)
        {
            Key key = findKey(it->c_str());
            if (key < 0)
                continue;

            Data& d = data_[key];
            std::map<std::string, ros::Subscriber>::iterator it_sub = d.subs.find(bb_id);
            if (it_sub != d.subs.end())
                continue;

            ros::NodeHandle nh;
            nh.setCallbackQueue(&value_cb_queue_);
            ros::Subscriber& sub = d.subs[bb_id];
            sub = nh.subscribe<blackboard::ValueUpdate>(bb_id + "/bb/data/" + *it, 1, boost::bind(&Blackboard::cbValue, this, _1, key));

        }
    }

    void cbValue(const blackboard::ValueUpdateConstPtr& msg, Key key)
    {
        Data& d = data_[key];
        std::cout << "Received value: " << msg->data << std::endl;
    }

    void updateConnections()
    {
        key_cb_queue_.callAvailable();
    }

    void updateValues()
    {
        value_cb_queue_.callAvailable();
    }

private:

    std::map<std::string, Key> key_map_;

    std::vector<Data> data_;

    ros::Publisher pub_key_;

    ros::CallbackQueue key_cb_queue_;

    ros::CallbackQueue value_cb_queue_;

    std::map<std::string, BlackboardConnection> connections_;

};

} // end namespace bb

#endif
