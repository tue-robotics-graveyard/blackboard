#include "blackboard/new_buffer.h"

#include <iostream>

// ----------------------------------------------------------------------------------------------------

class TestClass
{
public:
    TestClass(int i) : i_(i) { std::cout << "TestClass(" << i << ")" << std::endl; }
    ~TestClass() { std::cout << "~TestClass() i = " << i_ << std::endl; }
    TestClass(const TestClass& t) : i_(t.i_) { std::cout << "TestClass(const TestClass&) i = " << i_ << std::endl; }

private:

    int i_;
};


// ----------------------------------------------------------------------------------------------------

template<typename T>
void serialize(const T& p)
{

}

class Bla
{
public:

    Bla() : value_(0) {}

    template<typename T>
    void set(const T& t)
    {
        value_ = new T(t);
        serialize<T>(t);
    }

    template<typename T>
    const T* get() const
    {
        return static_cast<const T*>(value_);
    }

private:

    void* value_;

};

// ----------------------------------------------------------------------------------------------------

template<>
void serialize<float>(const float& p)
{
    std::cout << "float" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

template<>
void serialize<long>(const long& p)
{
    std::cout << "long" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main()
{
//    Bla bla;
//    bla.set<float>(1.23);

//    std::cout << *bla.get<float>() << std::endl;

    std::cout << "Start" << std::endl;

    Buffer b;
    for(int i = 0; i < 1000000; ++i)
        b.add(float(i) / 10);

    std::cout << "Stop" << std::endl;

    std::cout << b.get<float>(123) << std::endl;

    TestClass t(5);

    const TestClass& p = t;

    b.add(p);

    std::cout << b.size() << std::endl;

    std::cout << "---" << std::endl;

    return 0;    
}

