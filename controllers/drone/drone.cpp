#include <iostream>
#include <cmath>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <vector>
#include <algorithm>
#include <webots/Keyboard.hpp>

double integError = 0.0;
double altitudeControl(double currentAltitude, double desiredAltitude, double verticalVelocity, double dt)
{
  const double Kp = 1.7; // constant proportional
  const double Ki = .2; // constant integral
  const double Kd = 4.5; // constant derivative
  double error = desiredAltitude - currentAltitude; // altitude error
  
  double outputP = Kp*error; // calculate the proportional output
  
  integError += error * dt; // calculates the integral error
  integError = std::clamp(integError, -0.05, 0.05); // clamps the integral error to -.5 to .5 so it dosnt go crazy and overshoot
  double outputI = Ki * integError; // calculate the integral output
  
  double outputD = Kd * verticalVelocity; // calculate the derivative output
  
  double correction = outputP + outputI - outputD; // calculate the correction for the drone
  return correction;
}

std::vector<double> positionControl(double currentX, double currentY, const double *desiredPos, const double *velocities, double dt)
{
  const double Kp = 2.5;
  const double Kd = 3.0;
  
  double errorX = desiredPos[0] - currentX;
  double errorY = desiredPos[1] - currentY;
  double desiredX = (Kp * errorX) - (Kd * velocities[0]);
  double desiredY = (Kp * errorY) - (Kd * velocities[1]);
  
  return {desiredX, desiredY};
}

double imuIntegError[3] = {0.0, 0.0, 0.0};
std::vector<double> imuControl(const double *imuValues, const double *desiredImu, const double *angularVelocity,double dt)
{
  // constat for prop, integ, and deriv, for roll, pitch, and yaw
  const double Kp[3] = {2.0, 2.0, 0.5}; 
  const double Ki[3] = {0.0, 0.0, 0.0};
  const double Kd[3] = {0.5, 0.5, 0.05};
  
  double errors[3] = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++) errors[i] = desiredImu[i] - imuValues[i];
  
  double outputP[3] = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++) outputP[i] = Kp[i] * errors[i];
  
  double outputI[3] = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++){
   imuIntegError[i]+= errors[i] * dt;
   imuIntegError[i]= std::clamp(imuIntegError[i], -.1, 0.1);
   outputI[i]= Ki[i]*imuIntegError[i];
   }
  
  double outputD[3] = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++) outputD[i] = Kd[i] * angularVelocity[i];
  
  std::vector<double> correction = {0.0, 0.0, 0.0};
  for(int i = 0; i < 3; i++) correction[i] = outputP[i] - outputD[i] + outputI[i];
  std::cout << correction[2] << std::endl;
  return correction;
}


using namespace webots;



int main()
{
  Supervisor *drone = new Supervisor();
  Keyboard *keyboard = new Keyboard();
  keyboard->enable(32);
    
  const double timeStep = drone->getBasicTimeStep();
  const double dt = timeStep / 1000.0; //delta time in seconds (needed for calculations)
  
  webots::Motor *frontRightMotor = drone->getMotor("m1_motor");
  webots::Motor *rearRightMotor = drone->getMotor("m2_motor");
  webots::Motor *rearLeftMotor = drone->getMotor("m3_motor");
  webots::Motor *frontLeftMotor = drone->getMotor("m4_motor");
  Motor *motors[4] = {frontRightMotor, rearRightMotor, rearLeftMotor, frontLeftMotor};
  const double base = 55.3681;
  
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
  
  // target variables and previous vars, ideal things like not tiled or at a certain height
  //altitude
  double previousAltitude = 0.015;
  double desiredAltitude = .5;
  //imu
  double desiredImu[3] = {0.0, 0.0, 0.0};
  double previousImuValues[3] = {0.0, 0.0, 0.0};
  //position x and y values, the z value is controlled in the altitude section
  double desiredPos[2] = {0.0, 0.0};
  double previousPosValues[2] = {0.0, 0.0};
  
  // correction variables
  double altitudeCorrection = 0.0;
  std::vector<double> imuCorrection = {0.0, 0.0, 0.0};
  std::vector<double> posCorrection = {0.0, 0.0};
  
  
  double motorInputs[4] = {0,0,0,0};
  while(drone->step(timeStep) != -1)
  {
  imuValues = imu->getRollPitchYaw();
  gpsValues = gps->getValues();
    for(int i = 0; i < 3; i++)
    {
      previousImuValues[i] = imuValues[i];
      previousAltitude = gpsValues[2];
      if(i < 2)
        previousPosValues[i] = gpsValues[i];
    }
    if(drone->getTime() > 1)
      break;
  }
  
  
  
  while(drone->step(timeStep) != -1)
  {
  
    
    imuValues = imu->getRollPitchYaw();//orientation roll pitch yaw
    gpsValues = gps->getValues();//position xyz
    gyroValues = gyro->getValues();//angular velocity
    
    int key = keyboard->getKey(); // z is 90, x is 88
    if(key == 90) desiredAltitude += 2 * dt;
    else if(key == 88)  desiredAltitude -= 2 * dt;
    else if(key == Keyboard::UP)  desiredPos[0] += 2 * dt;
    else if(key == Keyboard::DOWN)  desiredPos[0] -= 2 * dt;
    else if(key == Keyboard::RIGHT)  desiredPos[1] -= 2 * dt;
    else if(key == Keyboard::LEFT)  desiredPos[1] += 2 * dt;
    else if(key == Keyboard::PAGEUP) desiredImu[2] += 2 *dt;
    else if(key == Keyboard::PAGEDOWN) desiredImu[2] -= 2 *dt;
    // calculate the correction for the altitude to reach the desired altitude
    double verticalVelocity = (gpsValues[2]-previousAltitude)/dt; // calculate the delta V
    altitudeCorrection = altitudeControl(gpsValues[2], desiredAltitude, verticalVelocity, dt); 
    
    // calculate the x and y velocity and correction for the position x and y
    double xyVelocity[2] = {(gpsValues[0]-previousPosValues[0])/dt, (gpsValues[1]-previousPosValues[1])/dt};
    posCorrection = positionControl(gpsValues[0], gpsValues[1], desiredPos, xyVelocity, dt);
    desiredImu[0] = std::atan(-posCorrection[1] / 9.81);
    desiredImu[1] = std::atan(posCorrection[0] / 9.81);
    
    // calculate correction for the imu
    double angularVelocity[3] = {gyroValues[0], gyroValues[1], gyroValues[2]};
    std::vector<double> imuCorrection = imuControl(imuValues, desiredImu, angularVelocity, dt);
    
     std::cout << "Current: " <<imuValues[2] << std::endl;
    std::cout << "Desired: " <<desiredImu[2] << std::endl;
    // std::cout << "Correct: " << posCorrection[0] << std::endl;
    
    // calculate the motor velocity inputs bases on the corrections
    motorInputs[0] = base+altitudeCorrection - imuCorrection[1] - imuCorrection[0]  +imuCorrection[2]; // FR
    motorInputs[1] = base + altitudeCorrection + imuCorrection[1] - imuCorrection[0] - imuCorrection[2]; // RR
    motorInputs[2] = base + altitudeCorrection + imuCorrection[1] + imuCorrection[0] + imuCorrection[2]; // RL
    motorInputs[3] = base +altitudeCorrection - imuCorrection[1] + imuCorrection[0] - imuCorrection[2]; // FL
    for(int m = 0; m < 4; m++)
    {
      if(m%2==0)
        motors[m]->setVelocity(-motorInputs[m]);
      else
        motors[m]->setVelocity(motorInputs[m]);
    }
    
    previousAltitude = gpsValues[2];
    
    for(int i = 0; i < 3; i++) previousImuValues[i] = imuValues[i];
    
    for(int i = 0; i < 2; i++) previousPosValues[i] = gpsValues[i];
    
  }
  return 0;
}