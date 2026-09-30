#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <vector>
#include "../../config/DroneConfig.hpp"



int main()
{
    // set up drone (robot) object
    webots::Robot *drone = new webots::Robot();
    
    const int timeStep = drone->getBasicTimeStep();
    
    webots::InertialUnit *imu =  drone->getInertialUnit("imu");
    
    
    while(drone->step(timeStep) != -1)
    { 
      
    }
    return 0;
}

