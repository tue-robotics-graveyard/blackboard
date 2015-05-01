#ifndef BLACKBOARD_BLACKBOARD_H_
#define BLACKBOARD_BLACKBOARD_H_

#include "blackboard/key.h"

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

    void addTrigger(Key key, trigger_function func)
    {
        data_[key].trigger_functions.push_back(func);
    }

    void setValue(Key key, const char* value)
    {
        const std::vector<trigger_function>& trigger_functions = data_[key].trigger_functions;
        for(std::vector<trigger_function>::const_iterator it = trigger_functions.begin(); it != trigger_functions.end(); ++it)
        {
            (*it)(*this);
        }

        // TODO: set value

    }

private:

    std::map<std::string, Key> key_map_;

    std::vector<Data> data_;

};

} // end namespace bb

#endif
