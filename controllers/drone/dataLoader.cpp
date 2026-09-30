#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include "../../config/DroneConfig.hpp"

void loadConfigData(webots::Supervisor* droneSuper)
{
   // gets the physics node and field
   webots::Node *droneNode = droneSuper->getFromDef("MY_DRONE");
   webots::Field *physicsField = droneNode->getField("physics");
   webots::Node *physicsNode = physicsField->getSFNode();
   
   
   // sets the mass field to the drone config mass constant
   webots::Field *massField = physicsNode->getField("mass");
   massField->setSFFloat(DroneConfig::mass);
   std::cout << "Loaded Mass" << std::endl;
   
   // loads the center of mass from the drone config file
   webots::Field *comField = physicsNode->getField("centerOfMass");
   const double comFieldValues[3] = {DroneConfig::centerOfMass.x, DroneConfig::centerOfMass.y, DroneConfig::centerOfMass.z};
   comField->setMFVec3f(0, comFieldValues);
   std::cout << "Loaded COM" << std::endl;
   
   // loads the MOI values (moment of inertia)
   webots::Field *inertiaField = physicsNode->getField("inertiaMatrix");
   const double inertiaFieldValues[3] = {DroneConfig::Ixx, DroneConfig::Iyy, DroneConfig::Izz};
   inertiaField->setMFVec3f(0, inertiaFieldValues);
   std::cout << "Loaded MOI" << std::endl;
   

}
