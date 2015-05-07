#include <blackboard/blackboard.h>

#include <iostream>

#include <ros/this_node.h>

bb::Key my_key;

// ----------------------------------------------------------------------------------------------------

class IntSerializer : public bb::Serializer
{

    void serialize(const bb::Variant& data, bb::WBytes& bytes)
    {
        int i = data.getValue<int>();
        bytes.resize(sizeof(i));
        memcpy(bytes.ptr(), (unsigned char*)&i, bytes.size());
    }

    void deserialize(const bb::RBytes& bytes, bb::Variant& v)
    {
        int* i = (int*)&bytes.ptr()[0];
        v.setValue<int>(*i);
    }
};

// ----------------------------------------------------------------------------------------------------

void test_trigger(const bb::Blackboard& b, const bb::Key& key)
{
    std::cout <<  ros::this_node::getName() << ": Value update: " << b.getValue<int>(key) << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: test_blackboard i N" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "test_blackboard_" + std::string(argv[1]));

    bb::Blackboard b;
    b.initialize();

    int my_id = atoi(argv[1]);
    int N = atoi(argv[2]);

    // Add external boards
    for(int i = 0; i < N; ++i)
    {
        if (i == my_id)
            continue;

        std::stringstream ss_name;
        ss_name << "test_blackboard_" << i;
        b.addExternal(ss_name.str());
    }

    for(int i = 0; i < N; ++i)
    {
        std::stringstream ss_key_name;
        ss_key_name << "key_" << i;

        bb::Key k = b.addKey(ss_key_name.str().c_str(), new IntSerializer);

        if (i == my_id)
            my_key = k;

        b.addTrigger(k, test_trigger);
    }

    ros::Rate r(10);
    while(ros::ok())
    {
        b.updateValues();
        b.setValue(my_key, my_id * 123);
        r.sleep();
    }

    return 0;
}
