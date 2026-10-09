#include <iostream>
#include <cmath>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>

double altitudeControl(double currentAltitude, double desiredAltitude, double verticalVelocity)
{
  double hoverThrust = 0.05*9.81;
  const double Kp = 2.0;
  const double Kd = 3.0;
  
  double error = desiredAltitude - currentAltitude;
  double correction = Kp * error - Kd *verticalVelocity;
  return correction;
}


using namespace webots;



int main()
{
  Supervisor *drone = new Supervisor();
  
  const double timeStep = drone->getBasicTimeStep();
  const double dt = timeStep / 1000.0; //delta time
  
  webots::Motor *frontRightMotor = drone->getMotor("m1_motor");
  webots::Motor *rearRightMotor = drone->getMotor("m2_motor");
  webots::Motor *rearLeftMotor = drone->getMotor("m3_motor");
  webots::Motor *frontLeftMotor = drone->getMotor("m4_motor");
  Motor *motors[4] = {frontRightMotor, rearRightMotor, rearLeftMotor, frontLeftMotor};
  const double base = 55.4;
  
  // Sets up gps
  GPS *gps = drone->getGPS("gps");
  gps->enable(32);
  const double *gpsValues = gps->getValues();
  
  // Sets up inertial unit, roll, pitch, and yaw
  InertialUnit *imu = drone->getInertialUnit("inertial_unit");
  imu->enable(32);
  const double *imuValues = imu->getRollPitchYaw();
    
  Gyro *gyro = drone->getGyro("gyro");
  gyro->enable(32);
  const double *gyroValues = gyro->getValues();
    
  for(int m = 0; m < 4; m++)
  {
    motors[m]->setPosition(INFINITY);
    if(m == 0 || m == 2)
      motors[m]->setVelocity(0); // negative
    else
      motors[m]->setVelocity(0); // postive

  }
  
  // target variables, ideal things like not tiled or at a certain height
  double previousAltitude = 0.015;
  double desiredAltitude = 0.5;
  double motorInputs[4] = {0,0,0,0};
  
  while(drone->step(timeStep) != -1)
  {
    imuValues = imu->getRollPitchYaw();//orientation roll pitch yaw
    gpsValues = gps->getValues();//position xyz
    gyroValues = gyro->getValues();//angular velocity
    
    double verticalVelocity = (gpsValues[2]-previousAltitude)/dt;
    double correction = altitudeControl(gpsValues[2], desiredAltitude, verticalVelocity); 

    
    motorInputs[0] = base + correction;
    motorInputs[1] = base + correction;
    motorInputs[2] = base + correction;
    motorInputs[3] = base + correction;
    
    for(int m = 0; m < 4; m++)
    {
      if(m%2==0)
        motors[m]->setVelocity(-motorInputs[m]);
      else
        motors[m]->setVelocity(motorInputs[m]);
    }
    
    previousAltitude = gpsValues[2];
  }
  return 0;
}