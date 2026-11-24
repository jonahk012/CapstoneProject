// in this file i will connect to the supervisor of the drone and restart the sim
// until i find the best values for Kp Ki and Kd
#include "DroneTestConstants.hpp"
#include "AltitudePID.hpp"
#include "WebotsInterface.hpp"
#include "DroneState.hpp"
#include "PositionPID.hpp"
#include "OrientationPID.hpp"
#include "DroneTargets.hpp"
#include "VelocityPID.hpp"
#include <vector>
#include <fstream>
#include <string>
#include <cmath>

WebotsInterface interface = WebotsInterface();
AltitudePID aPID;
DroneState state;
PositionPID pPID;
OrientationPID imuPID;
VelocityPID vPID;
DroneTargets targets;

DroneTestConstants c = DroneTestConstants(interface, aPID, state, pPID, imuPID, vPID, targets);

// what i wanna do in main is have targets and points
// i wanna make it so we want the least amount of points possible
// i am gonna research some machine training or something idk
int main()
{
  double testRunTime = 5.0;
  
  c.runLoop(testRunTime);
  
  return 1;
}

