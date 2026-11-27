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
#include <sstream>
#include<iostream>

std::vector<double> getNextConstants();
// what i wanna do in main is have targets and points
// i wanna make it so we want the least amount of points possible
// i am gonna research some machine training or something idk

// i will load the data the write the data from a csv file called testingPID
// i will add to the file whever i try out a new PID combination
// i could have the csv file have headers like
// Kp, Ki, Kd, s, o, t
// s will stand for score, the lower the score the better
// and t will be for time to if needed, test for longer time
// o can be a vector 3 with x y and z overshoot
// i will write the dat in DroneTestConstants 
// but in this file i will read the data from the csv file

int main()
{
  // if the test time is -1 then it wont run with repeating
  double testRunTime = 10;
  WebotsInterface interface = WebotsInterface();
  
  while(true)
  {
    AltitudePID aPID;
    DroneState state;
    PositionPID pPID;
    OrientationPID imuPID;
    VelocityPID vPID;
    DroneTargets targets;
    DroneTestConstants c = DroneTestConstants(interface, aPID, state, pPID, imuPID, vPID, targets);
    
    c.runLoop(testRunTime, getNextConstants());
  }
  
}

std::vector<double> getNextConstants()
{
  
  // opens testingPID as a input file (to read it)
  std::ifstream csvFile("testingPID.csv", std::istream::in);
  if(!csvFile)
  {
    std::cout << "Cannot Find csvFile or file is open" << std::endl;
    return {0,0,0};
  }
  // temp string that holds lines of csvFile
  std::string line;
  std::string lastLine;
  // csvFile.seekg(0, std::ifstream::beg);
  // while it has a next line, stores it in line and prints it
  while(std::getline(csvFile, line))
  {
    lastLine = line;
  }
  std::istringstream lastLineStream(lastLine);
  
  std::vector<double> previousRunValues;
  while(std::getline(lastLineStream, line, ','))
   if(line != "")
     previousRunValues.push_back(std::stod(line));
    
  // for(double s : previousRunValues)
    // std::cout << s << std::endl;
  // --IMPORTANT--
  // after i get the previous testing PID values, i will need to get the
  // new ones, i might use some sort of gradient thing idk
  return previousRunValues;
}
