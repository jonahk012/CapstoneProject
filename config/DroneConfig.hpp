#pragma once

struct Vector3
{
	double x;
	double y;
	double z;
};

namespace DroneConfig
{

	// World constants 
	constexpr double grav_accel = 9.81; // m/s^2

	// Body of the Drone
	constexpr double mass = 1.5;
	constexpr double imuUpdateRate = 1000.0; //in hz	

	// Moment of Inertia kg*m^2
	// resistance to rotation on each axis
	constexpr double Ixx = 0.008;
	constexpr double Iyy = 0.015;
	constexpr double Izz = 0.017;
		

	// Motors
	// Motor rotation direction: -1 - CCW, +1 - CW
	
	constexpr int motorCount = 4;
	constexpr double motorMass = 0.025;
	constexpr double motorDamping = 0.004;
	constexpr double maxMotorRPM = 838.0;

	// motor 0
	constexpr int motor0_direction = -1;
	constexpr Vector3 motor0Pos{0.1293, -0.2188, 0.0262};

	// motor 1
	constexpr int motor1_direction = -1;
	constexpr Vector3 motor1Pos{-0.1339, 0.2066, 0.0262};

		
	// motor 2
	constexpr int motor2_direction = 1;
	constexpr Vector3 motor2Pos{0.1257, 0.2224, 0.0262};

	// motor 3
	constexpr int motor3_direction = 1;
	constexpr Vector3 motor3Pos{-0.1378, -0.2027, 0.0262};




}