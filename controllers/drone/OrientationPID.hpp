#pragma once
#include <algorithm>
#include <vector>
#include <iostream>

struct OrientationPID
{
  const double Kp = 1.0; 
  const double Kd = 0.8;
  
  std::vector<double> imuCorrection(double desiredRoll, double desiredPitch, double desiredYaw, const double* imuValues, const double* gyroValues)
  {
    double errorRoll = desiredRoll - imuValues[0];
    double errorPitch = desiredPitch - imuValues[1];
    double errorYaw = desiredYaw - imuValues[2];
    
    
    double outputPRoll = errorRoll * Kp;
    double outputPPitch = errorPitch * Kp;
    double outputPYaw = errorYaw * Kp;
    
    double outputDRoll = Kd * gyroValues[0];
    double outputDPitch = Kd * gyroValues[1];
    double outputDYaw = Kd * gyroValues[2];
      
    
    return {outputPRoll - outputDRoll, outputPPitch - outputDPitch, outputPYaw - outputDYaw};
  }
};