#pragma once
#include <iostream>
#include <vector>

struct VelocityPID
{
  const double Kp_xy = 0.8, Kd_xy = 0.5; 
  
  std::vector<double> velocityCorrection(double xVelocity, double yVelocity, double verticalVelocity, double desiredVVelocity, double desiredXYVelocity, const double* aclValues, double dt)// pass in the velocitys, and the target velocity
  {
    double errorXV = desiredXYVelocity - xVelocity;
    double errorYV = desiredXYVelocity - yVelocity;
    //double errorVV = desiredVVelocity - verticalVelocity;
    
    //std::cout << errorXV << std::endl;
    
    double outputPX = errorXV * Kp_xy;
    double outputPY = errorYV * Kp_xy;
    // double outputPV = errorVV * Kp_v;
    
    double outputDX = Kd_xy * aclValues[0];// acceleration X
    double outputDY = Kd_xy * aclValues[1];// acceleration Y
    // double outputDV = Kd_v * aclValues[2];// acceleration V
    double outputX = outputPX - outputDX;
    double outputY = outputPY - outputDY;
 
    
    return {outputX, outputY};
  }
};