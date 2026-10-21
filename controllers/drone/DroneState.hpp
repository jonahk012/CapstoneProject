#pragma once

struct DroneState
{
  double x = 0.0;
  double y = 0.0;
  double altitude = 0.0;
  double previousAltitude = 0.0;
  
  double verticalVelocity = 0.0;
  
  double velocityRoll = 0.0;
  double velocityPitch = 0.0;
  double velocityYaw = 0.0;
  
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  // later add the desired into a file that contains the targets, can later add a path for it to follow
  double desiredAltitude = 1.0;
  
  
  bool update(std::vector<const double*> sensorValues, double dt)
  {
    x = sensorValues[0][0];
    y = sensorValues[0][1];
    altitude = sensorValues[0][2];
    
    verticalVelocity = (sensorValues[0][2]-previousAltitude)/dt; // need to /dt but i dont have it rn
    
    velocityRoll = sensorValues[0][0];
    velocityPitch = sensorValues[0][1];
    velocityYaw = sensorValues[0][2];
    
    roll = sensorValues[0][0];
    pitch = sensorValues[0][1];
    yaw = sensorValues[0][2];
    
    return true;
  }
  void printTelemetry()
  {

    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << altitude << std::endl;
    std::cout << verticalVelocity << std::endl;
    std::cout << desiredAltitude << std::endl;
    
  
  }
};