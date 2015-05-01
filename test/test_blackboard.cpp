#include <blackboard/blackboard.h>

#include <iostream>

void test_trigger(const bb::Blackboard& b)
{
    std::cout << "Triggers!" << std::endl;
}

int main(int argc, char **argv)
{
    bb::Blackboard b;

    bb::Key k = b.getKey("test");

    b.addTrigger(k, test_trigger);

    b.setValue(k, "bla");

    return 0;
}
