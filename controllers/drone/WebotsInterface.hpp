/*
This file is where the retreiving of the drone parts takes place
later long if I decide to replace this with a real drone,
it will be easy to make another file, "drone interface", that can easily retreive the 
modules on the drone
*/
#pragma once
#include <iostream>
#include <print>
#include <vector>
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include "DroneState.hpp"
using namespace webots;



struct WebotsInterface
{
  // make the drone and the timestep
  Supervisor* drone;
  int timeStep;
  double dt;
  // sensors
  GPS* gps;
  const double* gpsValues;
  
  Gyro* gyro;
  const double* gyroValues;
  
  InertialUnit* imu;
  const double* imuValues;
  
  // motors
  Motor *frontRightMotor;
  Motor *rearRightMotor;
  Motor *rearLeftMotor; 
  Motor *frontLeftMotor;
  
  std::vector<Motor*> motors;
  
  std::vector<double> motorInputs;
  
  const double baseThrust = 55.3681;
  
  WebotsInterface()
  {
   drone = new Supervisor();
   timeStep = drone->getBasicTimeStep();
   if(timeStep == 0) timeStep = 32;
   dt = timeStep/1000.0;
   gps = drone->getGPS("gps");
   gps->enable(timeStep);
   gpsValues = gps->getValues();
   
   gyro = drone->getGyro("gyro");
   gyro->enable(timeStep);
   gyroValues = gyro->getValues();
   
   imu = drone->getInertialUnit("inertial_unit");
   imu->enable(timeStep);
   imuValues = imu->getRollPitchYaw();
   
   frontRightMotor = drone->getMotor("m1_motor");
   rearRightMotor = drone->getMotor("m2_motor");
   rearLeftMotor = drone->getMotor("m3_motor"); 
   frontLeftMotor = drone->getMotor("m4_motor");
   motors = {frontRightMotor, rearRightMotor, rearLeftMotor, frontLeftMotor};
   motorInputs = {0.0, 0.0, 0.0, 0.0};
   setupMotors();
  }
  
  
  bool running()
  {
    if (drone->step(timeStep) == -1)
      return false;
    return true;
  }
  
  std::vector<const double*> updateSensors(){return {gps->getValues(), gyro->getValues(), imu->getRollPitchYaw()};}
  
  bool setupMotors()
  {
    for(int m = 0; m < 4; m++)
    {
      if(motors[m])
      {
      motors[m]->setPosition(INFINITY);
      if(m == 0 || m == 2)
        motors[m]->setVelocity(0); // negative
      else
        motors[m]->setVelocity(0); // postive
      }
      else
        return false;
      }
      return true;
  }
  
  bool updateMotorThrust(DroneState& state)
  {
    for(int m = 0; m < 4; m++)
    {
      if(m%2==0)
        motors[m]->setVelocity(-motorInputs[m]);
      else
        motors[m]->setVelocity(motorInputs[m]);
    }
    state.previousAltitude = gpsValues[2];
    return true;
  }
  
  void printTelemetry()
  {

    
    std::cout << "GPS -> X: " << gpsValues[0]<< " Y: " << gpsValues[1]<< " Z: " << gpsValues[2]<< std::endl;
    std::cout << "Gyro -> X: " << gyroValues[0]<< " Y: " << gyroValues[1]<< " Z: " << gyroValues[2]<< std::endl;
    std::cout << "Imu -> X: " << imuValues[0]<< " Y: " << imuValues[1]<< " Z: " << imuValues[2]<< std::endl;
    
    //std::cout << "Altitude Correction: " << corr.altitudeCorrection << std::endl;
  
  }
};