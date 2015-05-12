#ifndef BLACKBOARD_UPDATE_H_
#define BLACKBOARD_UPDATE_H_

#include "blackboard/key.h"
#include "blackboard/variant.h"

#include <vector>
#include <map>

namespace bb
{

struct KeyUpdate
{
    Time timestamp;
    Variant value;
};

class Update
{

public:

    Update()
    {
    }

    ~Update()
    {
    }

    template<typename T>
    void setValue(Key key, Time t, T value)
    {
        std::vector<KeyUpdate>& ups = updates_[key];
        ups.push_back(KeyUpdate());
        KeyUpdate& u = ups.back();
        u.value = value;
        u.timestamp = t;
    }

    const std::map<Key, std::vector<KeyUpdate> >& updates() const { return updates_; }

private:

    std::map<Key, std::vector<KeyUpdate> > updates_;

};

} // end namespace bb

#endif
