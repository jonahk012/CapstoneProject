#include <webots/Robot.hpp>
#include <iostream>

int main()
{
    webots::Robot *robot = new webots::Robot();

    std::cout << "Flight Controller Connected to \"" << robot->getName() << "\"" << std::endl;

    return 0;
}