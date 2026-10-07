#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Motor.hpp>
#include <vector>
#include "dataLoader.hpp"
#include "../../config/DroneConfig.hpp"


int main()
{
    // set up drone (robot) object
    webots::Supervisor *droneSuper = new webots::Supervisor();
    const int timeStep = droneSuper->getBasicTimeStep();
    
    // loading data from droneConfig
    //loadConfigData(droneSuper);
    
    
    // loading the gps
    webots::GPS *gps = droneSuper->getGPS("gps");
    if(gps)
      gps->enable(32);
      
    // loading the gyro
    webots::Gyro *gyro = droneSuper->getGyro("gyro");
    if(gyro)
      gyro->enable(32);
     
    // loading the inertial units (orientation of the drone)
    webots::InertialUnit *imu = droneSuper->getInertialUnit("inertial unit");
    if(imu)
      imu->enable(32);
    
    // loading the acceleration
    webots::Accelerometer *acc = droneSuper->getAccelerometer("accelerometer");
    if(acc)
      acc->enable(32);
    
    for(int i = 1; i <= DroneConfig::motorCount; i++)
     {
       webots::Motor *motor = droneSuper->getMotor("motor" + std::to_string(i));
       if(motor)
       {
       motor->setPosition(INFINITY);
         if(i == 1 || i == 3)
       motor->setVelocity(2);
       else
       motor->setVelocity(-2);
       }
       else{
       std::cout << "Could not find motor " << std::to_string(i) << std::endl;
       }
     }
    
     
      
    while(droneSuper->step(timeStep) != -1)
    { 
      const double gpsValues[3] = {gps->getValues()[0], gps->getValues()[1], gps->getValues()[2]};
     const double gyroValues[3] = {gyro->getValues()[0], gyro->getValues()[1], gyro->getValues()[2]};
      const double imuValues[3] = {imu->getRollPitchYaw()[0], imu->getRollPitchYaw()[1], imu->getRollPitchYaw()[2]};
      const double accValues[3] = {acc->getValues()[0], acc->getValues()[1], acc->getValues()[2]};
    }
    return 0;
}

