#include <iostream>
#include <webots/Robot.hpp>


int main()
{
    webots::Robot *robot = new webots::Robot();

    std::cout << "flight Controller Connected to \"" << robot->getName() << "\"" << std::endl;

    return 0;
}