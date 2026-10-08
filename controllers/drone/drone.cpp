#include <iostream>
#include <cmath>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
using namespace webots;



int main()
{
  Supervisor *drone = new Supervisor();
  
  const double timeStep = drone->getBasicTimeStep();
  
  webots::Motor *rearLeftMotor = drone->getMotor("rear left propeller");
  webots::Motor *rearRightMotor = drone->getMotor("rear right propeller");
  webots::Motor *frontLeftMotor = drone->getMotor("front left propeller");
  webots::Motor *frontRightMotor = drone->getMotor("front right propeller");
  Motor *motors[4] = {rearLeftMotor, frontRightMotor,frontLeftMotor, rearRightMotor};
  
  GPS *gps = drone->getGPS("gps");
  gps->enable(32);
  
  for(int m = 0; m < 4; m++)
  {
    motors[m]->setPosition(INFINITY);
    if(m>=2)
      motors[m]->setVelocity(70);
    else
      motors[m]->setVelocity(-70);
  }
  
  while(drone->step(timeStep) != -1)
    if(drone->getTime() >= 1.0)
      break;
      
      
  double targetAltitude = 0.5;
  
  while(drone->step(timeStep) != -1)
  {
    

   
  }
  return 0;
}