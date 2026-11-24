#pragma once
struct DroneTargets
{
  double desiredAltitude = 1.0;
  double desiredX = 0.0;
  double desiredY = 0.0;
  
  double desiredRoll = 0.0;
  double desiredPitch = 0.0;
  double desiredYaw = 0.0;
  
  double desiredXYVelocity = 0.0; // used for breaking/slowing down the drone
  double desiredVVelocity = 9.81;
  double disturbanceX = 0.0;
  double disturbanceY = 0.0;
  void printTelemetry()
  {
    std::cout << "D A: " << desiredAltitude << std::endl;
    std::cout << "D X: " << desiredX << std::endl;
    std::cout << "D Y: " << desiredY << std::endl;
    
    std::cout << "D Roll: " << desiredRoll << std::endl;
    std::cout << "D Pitch: " << desiredPitch << std::endl;
    std::cout << "D Yaw: " << desiredYaw << std::endl;
  }
};