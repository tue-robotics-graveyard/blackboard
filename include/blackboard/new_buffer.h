#ifndef BLACKBOARD_NEW_BUFFER_H_
#define BLACKBOARD_NEW_BUFFER_H_

#include <vector>
#include <assert.h>
#include <stdexcept>

// ----------------------------------------------------------------------------------------------------

template<typename T>
void remove(void* v)
{
    T* p = static_cast<T*>(v);
    p->~T();
}

// ----------------------------------------------------------------------------------------------------

class Buffer
{

public:

    Buffer() : remover_(0) {}

    ~Buffer()
    {
        for(std::size_t i = 0; i < data_.size(); i += type_size_)
        {
            void* p = &data_[0] + i;
            remover_(p);
        }
    }

    template<typename T>
    void add(const T& v)
    {
        int type_index = get_type_index<T>();

        if (!remover_)
        {
            type_size_ = sizeof(T);
            remover_ = remove<T>;
            type_index_ = type_index;
        }
        else
        {
            assert(type_index == type_index_);
            if (type_index != type_index_)
                throw std::invalid_argument("In Buffer::add() (" + std::string(__FILE__) + "): Incorrect type");
        }

        std::size_t offset = data_.size();
        data_.resize(data_.size() + sizeof(T));
        new (&data_[0] + offset) T(v);
    }

    template<typename T>
    const T& get(std::size_t i) const
    {
        return *reinterpret_cast<const T*>(&data_[i * type_size_]);
    }

    std::size_t size() const { return data_.size() / type_size_; }

private:

    std::vector<char> data_;

    void (*remover_)(void* v);

    std::size_t type_size_;

    int type_index_;

    static int next_type_index()
    {
        static int next_type_index_(0);
        return next_type_index_++;
    }

    template <typename T_>
    static int get_type_index()
    {
        static int result(next_type_index());
        return result;
    }

};

#endif

