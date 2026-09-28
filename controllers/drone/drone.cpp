#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <vector>




int main()
{
    webots::Supervisor *supervisor = new webots::Supervisor();
    webots::Node *droneNode = supervisor->getFromDef("MY_DRONE");
    const double *position = droneNode->getPosition();
     
    //std::cout << "flight Controller Connected to the \"" << robot->getName() << "\"" << std::endl;
    
    std::cout << "Drone Position: "
              << "X: " << position[0] << ", "
              << "Y: " << position[1] << ", "
              << "Z: " << position[2] << std::endl;
    return 0;
}

