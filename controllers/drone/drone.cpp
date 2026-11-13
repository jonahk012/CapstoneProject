/*
This is the main file that will connect everything that i have made
including all of the flight controller, PID controllers, etc...
*/
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

bool mixMotors(double altitudeCorrection, std::vector<double> imuCorrection, std::vector<double> velocityCorrection);
bool recordTelemetry();
void clearCSV();

WebotsInterface interface = WebotsInterface();
AltitudePID aPID;
DroneState state;
PositionPID pPID;
OrientationPID imuPID;
VelocityPID vPID;
DroneTargets targets;

const double maxMotorThrust = 600;
bool firstLoop = true;
std::vector<const double*> sensors;

int main()
{
  
  // the sensors include... gps, gyro, and imu (orientation)
  // gps is index 0, gyro is 1, imu is 2, accelerometer is 3
  // you have to index 2 times bc its a 2d vector/array
  sensors = interface.updateSensors();
  
  
  while(interface.running()) // returns true if the loop is continuing
  {
    //updates the sensors every frame (every timestep)
    sensors = interface.updateSensors();
    // clamps the x and y disturbance so i cant tilt past a certain point causing a big no no
    targets.disturbanceX = std::clamp(targets.disturbanceX, -.5, .5);
    targets.disturbanceY = std::clamp(targets.disturbanceY, -.5, .5);
    // goes to the interface header file and checks if the
    // keyboard is pressed, it affects the disturbance in targets
    // additionally, returns true if a key is being pressed and false if not
    // - LATER - try to include in drone.cpp file a way to call a new method 
    // in another file that corrects the velocity of the drone
    bool accelerating = interface.checkKeyboardInputs(targets, sensors);
    
    // std::cout << "D: "<<targets.disturbanceX << std::endl;
    //std::cout << state.accelerating << std::endl;
    // sends the newly updated sensors to the drone state file which contains the current 
    // telemetry of the drone
    state.update(sensors, interface.dt, accelerating);
    // calculate the altitude correction thrust for each motor
    // double altitudeCorrection(double desiredAltitude, double currentAltitude, double verticalVelocity, double dt)
    double altitudeCorrection = aPID.altitudeCorrection(targets.desiredAltitude, sensors[0][2], state.verticalVelocity, interface.dt);
    
    // VELOCITY CORRECCTIONNNN
    //   std::vector<double> velocityCorrection(double xVelocity, double yVelocity, double verticalVelocity, double desiredVVelocity, double desiredXYVelocity, const double* aclValues)
    std::vector<double> velocityCorrection = vPID.velocityCorrection(state.xVelocity, state.yVelocity, state.verticalVelocity, targets.desiredVVelocity, targets.desiredXYVelocity, sensors[3], interface.dt);
    targets.desiredPitch = std::atan(velocityCorrection[0] / 9.81);
    targets.desiredRoll = -std::atan(velocityCorrection[1] / 9.81);
    //pPID.positionCorrection(targets.desiredX, targets.desiredY, sensors[0],state.xVelocity, state.yVelocity, targets);
    // bool imuCorrection(double desiredRoll, double desiredPitch, double desiredYaw, const double* imuValues, const double* gyroValues)
    std::vector<double> imuCorrection = imuPID.imuCorrection(targets.desiredRoll, targets.desiredPitch, targets.desiredYaw, sensors[2], sensors[1]);
    

    mixMotors(altitudeCorrection, imuCorrection, velocityCorrection);
    interface.updateMotorThrust(state);
    
    //interface.printTelemetry();
    //targets.printTelemetry();
    recordTelemetry();
  }
}

bool mixMotors(double altitudeCorrection, std::vector<double> imuCorrection, std::vector<double> velocityCorrection)
{
   //𝑇compensated=  𝑇alt / ( cos(𝜙)⋅cos(𝜃) )

    double tiltFactor = std::cos(sensors[2][0]) * std::cos(sensors[2][1]);
    tiltFactor = std::max(0.85, tiltFactor);
    double totalThrust = (interface.baseThrust + altitudeCorrection) / tiltFactor;
    
    std::cout << imuCorrection[2] << std::endl;
    
    interface.motorInputs[0] = totalThrust + altitudeCorrection - targets.disturbanceX + targets.disturbanceY - imuCorrection[1] - imuCorrection[0] + imuCorrection[2]; // FR
    interface.motorInputs[1] = totalThrust + altitudeCorrection + targets.disturbanceX + targets.disturbanceY + imuCorrection[1] - imuCorrection[0] - imuCorrection[2]; // RR
    interface.motorInputs[2] = totalThrust + altitudeCorrection + targets.disturbanceX - targets.disturbanceY + imuCorrection[1] + imuCorrection[0] + imuCorrection[2]; // RL
    interface.motorInputs[3] = totalThrust + altitudeCorrection - targets.disturbanceX - targets.disturbanceY - imuCorrection[1] + imuCorrection[0] - imuCorrection[2]; // FL
    
    for(int i = 0; i <4; i++)
    {
      interface.motorInputs[i] = std::clamp(interface.motorInputs[i], 0.0, maxMotorThrust);
    }
  return true;
}

void clearCSV()
{
  std::ofstream fileToClear("../../storedTelemetry/telemetry.csv", std::ios::trunc);
  fileToClear.close();
}


bool recordTelemetry()
{ 
  if(firstLoop)
    clearCSV();
    
  std::ofstream csvFile("../../storedTelemetry/telemetry.csv", std::ios::app);
  if(!csvFile.is_open())
  {
    std::cout << "Cannot Find csvFile or file is open" << std::endl;
    return false;
  }
  
  if(firstLoop)
    csvFile << "dAlt,cAlt\n"; //later on add a bunch of more data to store for later viewing or sum
    
  csvFile << std::to_string(targets.desiredXYVelocity) <<","<< std::to_string(state.xVelocity)<<"\n";
  csvFile.close();
  firstLoop = false;
  return true;
}