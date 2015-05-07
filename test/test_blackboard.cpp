#include <blackboard/blackboard.h>

#include <iostream>

void test_trigger(const bb::Blackboard& b)
{
    std::cout << "Triggers!" << std::endl;

    bb::Key k = b.findKey("test");
    std::cout << b.getValue<const char*>(k) << std::endl;
}

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

    bb::Key k = b.getKey("test");

    b.addTrigger(k, test_trigger);

    ros::Rate r(10);
    while(ros::ok())
    {
        b.setValue(k, "bla");
        r.sleep();
    }

    return 0;
}
