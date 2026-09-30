#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <vector>
#include "dataLoader.hpp"



int main()
{
    // set up drone (robot) object
    webots::Supervisor *droneSuper = new webots::Supervisor();
    
    const int timeStep = droneSuper->getBasicTimeStep();
    
    //webots::InertialUnit *imu =  drone->getInertialUnit("imu");
    
    loadConfigData(droneSuper);
    
    while(droneSuper->step(timeStep) != -1)
    { 
      
    }
    return 0;
}

// void setMass(Supervisor droneSuper)
// {
    
// }
