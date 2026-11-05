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
#include <webots/Keyboard.hpp>
#include <webots/Accelerometer.hpp>
#include "DroneState.hpp"
#include "DroneTargets.hpp"
using namespace webots;



struct WebotsInterface
{
  Keyboard *keyboard;
  double breaking;
  // make the drone and the timestep
  Supervisor* drone;
  int timeStep;
  double dt;
  double time;
  // sensors
  GPS* gps;
  const double* gpsValues;
  
  Gyro* gyro;
  const double* gyroValues;
  
  InertialUnit* imu;
  const double* imuValues;
  
  Accelerometer* acl;
  const double* aclValues;
  
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
   breaking = 0.7;
   drone = new Supervisor();
   timeStep = drone->getBasicTimeStep();
   if(timeStep == 0) timeStep = 32;
   dt = timeStep/1000.0;
   time = drone->getTime();
   gps = drone->getGPS("gps");
   gps->enable(timeStep);
   gpsValues = gps->getValues();
   
   gyro = drone->getGyro("gyro");
   gyro->enable(timeStep);
   gyroValues = gyro->getValues();
   
   imu = drone->getInertialUnit("inertial_unit");
   imu->enable(timeStep);
   imuValues = imu->getRollPitchYaw();
   
   acl = drone->getAccelerometer("accelerometer");
   acl->enable(timeStep);
   aclValues = acl->getValues();
   
   keyboard = new Keyboard;
   keyboard->enable(timeStep);
  
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
    time = drone->getTime();
    return true;
  }
  
  bool checkKeyboardInputs(DroneTargets& targets, std::vector<const double*> sensors)
  {
    // later make it so if im not currently trying to move, i activate a method
    // that corrects velocity, so that will break for me and correct the drifting after
    // the disturbance becomes zero
    int key = keyboard->getKey(); // z is 90, x is 88
    if(key != -1)
    {
    if(key == 90) targets.desiredAltitude += 2.0 * dt;
    else if(key == 88)  targets.desiredAltitude -= 2.0 * dt;
    if(key == Keyboard::UP)  targets.disturbanceX += 2.0 * dt;
    else if(key == Keyboard::DOWN)  targets.disturbanceX -= 2.0 * dt;
    if(key == Keyboard::RIGHT)  targets.disturbanceY -= 2.0 * dt;
    else if(key == Keyboard::LEFT)  targets.disturbanceY += 2.0 * dt;
      
    // else if(key == Keyboard::PAGEUP) desiredImu[2] += 2 *dt;
    // else if(key == Keyboard::PAGEDOWN) desiredImu[2] -= 2 *dt;
    
    }
    else
    {
       targets.disturbanceX *= breaking;
       targets.disturbanceY *= breaking;
      return false;
    }
    return true;
  }
  std::vector<const double*> updateSensors(){return {gps->getValues(), gyro->getValues(), imu->getRollPitchYaw(), acl->getValues()};}
  
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