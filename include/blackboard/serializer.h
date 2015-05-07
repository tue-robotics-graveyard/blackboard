#ifndef BLACKBOARD_SERIALIZER_H_
#define BLACKBOARD_SERIALIZER_H_

#include <vector>
#include "blackboard/variant.h"

namespace bb
{

class RBytes
{

public:

    virtual const unsigned char* ptr() const = 0;
    virtual std::size_t size() const = 0;
};

class WBytes
{

public:

    virtual unsigned char* ptr() = 0;
    virtual std::size_t size() const = 0;
    virtual bool resize(std::size_t size) = 0;
};

class Serializer
{

public:

    virtual void serialize(const Variant& data, WBytes& bytes) = 0;

    virtual void deserialize(const RBytes& bytes, Variant& v) = 0;

};

}

#endif
