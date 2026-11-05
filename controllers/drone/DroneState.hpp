#pragma once

struct DroneState
{
  double x = 0.0;
  double y = 0.0;
  double altitude = 0.0;
  
  double previousAltitude = 0.0;
  double previousX = 0.0;
  double previousY = 0.0;
  
  double verticalVelocity = 0.0;
  double xVelocity = 0.0;
  double yVelocity = 0.0;
  
  double velocityRoll = 0.0;
  double velocityPitch = 0.0;
  double velocityYaw = 0.0;
  
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  
  bool accelerating = false;
  // later add the desired into a file that contains the targets, can later add a path for it to follow
  
  
  bool update(std::vector<const double*> sensorValues, double dt, bool accel)
  {
    x = sensorValues[0][0];
    y = sensorValues[0][1];
    altitude = sensorValues[0][2];
    
    verticalVelocity = (sensorValues[0][2]-previousAltitude)/dt; // need to /dt but i dont have it rn
    xVelocity = (sensorValues[0][0]-previousX)/dt;
    yVelocity = (sensorValues[0][1]-previousY)/dt;
    
    velocityRoll = sensorValues[1][0];
    velocityPitch = sensorValues[1][1];
    velocityYaw = sensorValues[1][2];
    
    roll = sensorValues[2][0];
    pitch = sensorValues[2][1];
    yaw = sensorValues[2][2];
    
    accelerating = accel;
    
    // update previous vars
    previousAltitude = sensorValues[0][2];
    previousX = sensorValues[0][0];
    previousY = sensorValues[0][1];
    return true;
  }
  void printTelemetry()
  {
    
  
  }
};