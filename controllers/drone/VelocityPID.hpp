#pragma once
#include <iostream>
#include <vector>

struct VelocityPID
{
  const double Kp_xy = 0.8, Ki_xy = 0.1, Kd_xy = 2.5; 
  double integErrorX = 0.0, integErrorY = 0.0;
  std::vector<double> velocityCorrection(double xVelocity, double yVelocity, double verticalVelocity, double desiredVVelocity, double desiredXYVelocity, const double* aclValues, double dt)// pass in the velocitys, and the target velocity
  {
    double errorXV = desiredXYVelocity - xVelocity;
    double errorYV = desiredXYVelocity - yVelocity;
    //double errorVV = desiredVVelocity - verticalVelocity;
    
    //std::cout << errorXV << std::endl;
    
    double outputPX = errorXV * Kp_xy;
    double outputPY = errorYV * Kp_xy;
    // double outputPV = errorVV * Kp_v;
    
    integErrorX += errorXV * dt;
    integErrorX = std::clamp(integErrorX, -0.07, 0.07);
    double outputIX = Ki_xy * integErrorX;
    
    integErrorY += errorYV * dt;
    integErrorY = std::clamp(integErrorY, -0.07, 0.07);
    double outputIY = Ki_xy * integErrorY;
    
    
    double outputDX = Kd_xy * aclValues[0];// acceleration X
    double outputDY = Kd_xy * aclValues[1];// acceleration Y
    // double outputDV = Kd_v * aclValues[2];// acceleration V
    double outputX = outputPX + outputIX - outputDX;
    double outputY = outputPY + outputIY - outputDY;
 
    
    return {outputX, outputY};
  }
};