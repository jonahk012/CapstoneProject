/*
This is the main file that will connect everything that i have made
including all of the flight controller, PID controllers, etc...
*/
#include "AltitudePID.hpp"
#include "WebotsInterface.hpp"
#include "DroneState.hpp"

#include <vector>
#include <fstream>
#include <string>

bool mixMotors(double altitudeCorrection);
bool recordTelemetry();
void clearCSV();

WebotsInterface interface = WebotsInterface();
AltitudePID aPID;
DroneState state;

bool firstLoop = true;
int main()
{

  // the sensors include... gps, gyro, and imu (orientation)
  // gps is index 0, gyro is 1, imu is 2
  // you have to index 2 times bc its a 2d vector/array
  std::vector<const double*> sensors = interface.updateSensors();
  
  while(interface.running()) // returns true if the loop is continuing
  {
    //updates the sensors every frame (every timestep)
    sensors = interface.updateSensors();
    
    // sends the newly updated sensors to the drone state file which contains the current 
    // telemetry of the drone
    state.update(sensors, interface.dt);
    
    // calculate the altitude correction thrust for each motor
    // double altitudeCorrection(double desiredAltitude, double currentAltitude, double verticalVelocity, double dt)
    double altitudeCorrection = aPID.altitudeCorrection(state.desiredAltitude, sensors[0][2], state.verticalVelocity, interface.dt);
    mixMotors(altitudeCorrection);
    interface.updateMotorThrust(state);
    
    //interface.printTelemetry();
    recordTelemetry();
  }
}

bool mixMotors(double altitudeCorrection)
{
  for(int i = 0; i <4; i++)
  {
    interface.motorInputs[i] = interface.baseThrust + altitudeCorrection;
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
    std::cout << "Cannot Find csvFile" << std::endl;
    return false;
  }
  
  if(firstLoop)
    csvFile << "dAlt,cAlt\n"; //later on add a bunch of more data to store for later viewing or sum
    
  csvFile << std::to_string(state.desiredAltitude) <<","<< std::to_string(state.altitude)<<"\n";
  csvFile.close();
  firstLoop = false;
  return true;
}