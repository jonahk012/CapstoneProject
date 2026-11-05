#pragma once
#include <iostream>
#include <vector>

struct VelocityPID
{
  const double Kp = 0.5; 
  const double Kd = 0.8;
  std::vector<double> velocityCorrection(double xVelocity, double yVelocity, double verticalVelocity, double desiredVVelocity, double desiredXYVelocity, const double* aclValues)// pass in the velocitys, and the target velocity
  {
    double errorXV = desiredXYVelocity - xVelocity;
    double errorYV = desiredXYVelocity - yVelocity;
    double errorVV = desiredVVelocity - verticalVelocity;
    
    //std::cout << errorXV << std::endl;
    
    double outputPX = errorXV * Kp;
    double outputPY = errorYV * Kp;
    double outputPV = errorVV * Kp;
    
    double outputDX = Kd * aclValues[0];//* acceleration X
    double outputDY = Kd * aclValues[1];//* acceleration Y
    double outputDV = Kd * aclValues[2];//* acceleration V
      
    return {outputPX - outputDX, outputPY - outputDY, outputPV - outputDV};
  }
};