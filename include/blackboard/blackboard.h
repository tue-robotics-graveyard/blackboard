#ifndef BLACKBOARD_BLACKBOARD_H_
#define BLACKBOARD_BLACKBOARD_H_

#include "blackboard/key.h"
#include "blackboard/variant.h"

#include <vector>
#include <map>
#include <string>
#include <iostream>

namespace bb
{

class Blackboard;
typedef void (*trigger_function)(const Blackboard&);

struct Data
{
    std::vector<trigger_function> trigger_functions;
    Variant value;
};

class Blackboard
{

public:

    Blackboard();

    ~Blackboard();

    Key getKey(const char* name)
    {
        std::map<std::string, Key>::iterator it = key_map_.find(name);
        if (it != key_map_.end())
            return it->second;

        Key key = key_map_.size();
        key_map_[name] = key;
        data_.push_back(Data());

        return key;
    }

    Key getKey(const char* name) const
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
    }

    template<typename T>
    const T& getValue(Key key) const
    {
        const Data& d = data_[key];
        return d.value.getValue<T>();
    }

private:

    std::map<std::string, Key> key_map_;

    std::vector<Data> data_;

};

} // end namespace bb

#endif
