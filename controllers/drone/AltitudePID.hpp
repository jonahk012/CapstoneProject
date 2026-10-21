#pragma once
#include <algorithm>

struct AltitudePID
{
  const double Kp = 1.7; // constant proportional
  const double Ki = .2; // constant integral
  const double Kd = 4.5; // constant derivative
  double integError = 0.0;

  
double altitudeCorrection(double desiredAltitude, double currentAltitude, double verticalVelocity, double dt)
  {
    double error = desiredAltitude - currentAltitude; // altitude error
    
    double outputP = Kp*error; // calculate the proportional output
    
    integError += error* dt; // calculates the integral error
    integError = std::clamp(integError, -0.05, 0.05); // clamps the integral error to -.5 to .5 so it dosnt go crazy and overshoot
    double outputI = Ki * integError; // calculate the integral output
    
    double outputD = Kd * verticalVelocity; // calculate the derivative output
    
    double correction = outputP + outputI - outputD; // calculate the correction for the drone
    return correction;
     
     // instead of returning, i can set it to the DroneCorrection header file, then access it back in the interface, 
    // i can make a update method where i put the number and name so like
    // droneCorr.update(correction, "altitude") 
  }
};