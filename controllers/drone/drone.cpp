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
    loadConfigData(droneSuper);
    
    
    // loading the gps
    webots::GPS *gps = droneSuper->getGPS("gps");
    gps->enable(32);
      
    // loading the gyro
    webots::Gyro *gyro = droneSuper->getGyro("gyro");
    gyro->enable(32);
     
    // loading the inertial units (orientation of the drone)
    webots::InertialUnit *imu = droneSuper->getInertialUnit("inertial unit");
    imu->enable(32);
    
    // loading the acceleration
    webots::Accelerometer *acc = droneSuper->getAccelerometer("accelerometer");
    acc->enable(32);
    
    for(int i = 0; i < DroneConfig::motorCount; i++)
     {
       webots::Motor *motor = droneSuper->getMotor("motor" + std::to_string(i));
       if(motor)
       {
       motor->setPosition(INFINITY);
       if(i % 2 == 0)
         motor->setVelocity(-15);
       else
         motor->setVelocity(15);
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
      
      std::cout << "Position: " 
                << "X: " <<gpsValues[0] 
                << " Y: " <<gpsValues[1] 
                << " Z: " <<gpsValues[2] 
                << std::endl;
                
      std::cout << "Angular Velocity: " 
                << "X: " <<gyroValues[0] 
                << " Y: " <<gyroValues[1] 
                << " Z: " <<gyroValues[2] 
                << std::endl;
                
       std::cout << "Orientation: " 
                << "Roll: " <<imuValues[0] 
                << " Pitch: " <<imuValues[1] 
                << " Yaw: " <<imuValues[2] 
                << std::endl;
                
       std::cout << "Acceration: " 
                << "X: " <<accValues[0] 
                << " Y: " <<accValues[1] 
                << " Z: " <<accValues[2] 
                << std::endl;
                
        
    }
    return 0;
}

