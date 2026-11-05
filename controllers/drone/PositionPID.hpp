/* 
This file is gonna be "kinda scrapped," because this isnt the way I wanna do correction
instead what will be replacing it will be a velocityPID or something adjasent to that
what that will do, is it will just correct drift when the disturbance isnt being affected

what this file will eventually do is maybe like pathfinding? that would be kinda cool and wouldnt need
controlling by using the keys
*/



#pragma once
#include <algorithm>
#include <vector>
#include <iostream>
#include "DroneTargets.hpp"
struct PositionPID
{
    const double Kp = 3.0; // constant proportional
    const double Kd = 4.0; // constant derivative 4.5
    
    bool positionCorrection(double desiredX, double desiredY, const double* gpsValues, double xVelocity, double yVelocity, DroneTargets& targets) // eventually pass in the needed information to do the position correction
    // maybe add disturbance, or just make it the desired position
    {
      // at the very end use atan from i think cmath or algorithm library
      // to translate the acceleration correction to the wanted
      // Orientation.
      
      
      double errorX = desiredX - gpsValues[0];
      double errorY = desiredY - gpsValues[1];
      
      double outputPX = errorX * Kp;
      double outputPY = errorY * Kp;
      
      double outputDX = Kd * xVelocity;
      double outputDY = Kd * yVelocity;
      
      
      std::vector<double> correction = {outputPX - outputDX, outputPY - outputDY};
      targets.desiredRoll = -std::atan(correction[0] / 9.81);
      targets.desiredPitch = std::atan(correction[1] / 9.81);
      return true;
    }
};