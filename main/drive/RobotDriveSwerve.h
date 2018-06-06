/*
 * Matthew Spofford, Team 4048 Redshift
 * Similar to RobotDrive with a swerve drive modification
 */

#pragma once

#include "WPILib.h"
#include "../math/SwerveMath.h"
#include "SwerveEnclosure.h"

/*
 * This is used for easily controllig the movement of a robot with a swerve
 * drivetrain.  Objects of this class are created using seperate SwerveEnclosure
 * objects, that store and move all of the systems corresponding speed controllers.
 *
 * Each swerve system (with the help of a gyroscope) allows for switching between
 * two modes of movement, robot centric and field centric.  Field centric mode
 * causes the swerve system to always keep the forward/backward and directional
 * movement to stay relative to zero degrees.  Robot centric mode causes the swerve
 * system to act with normal cartesian movement.
 *
 * In order to modify this swerve drive control to support different hardware
 * interfaces not offered by our libraries, either modify currectly existing
 * SwerveEnclosure classes, or create your own SwerveEnclosure class.
 */
class RobotDriveSwerve
{
public:
	enum DriveMode{
		kFieldCentric = 0,
		kRobotCentric = 1
	};

	/*
	 * Defines the RobotDriveSwerve object
	 * Requires SwerveEnclosures as input to allow for various types of hardware
	 * interfaces to be used by this system.
	 * Requires the width and length between the swerve wheels.
	 */
	RobotDriveSwerve(SwerveEnclosure* frontLeftWheel,
					 SwerveEnclosure* frontRightWheel,
					 SwerveEnclosure* rearLeftWheel,
					 SwerveEnclosure* rearRightWheel,
					 double width, double length);
	virtual ~RobotDriveSwerve() = default;

	/*
	 * Allows the drivetrain to function as a swerve drive
	 * Requires the input of a forward, directional, and rotation value, along with
	 * the current angle of a gyro if field centric mode is going to be used
	 */
	void move(double forwardMovement, double directionalMovement, double rotationMovement, double angle = -999);
	
	/*
	 * Stops all movement of each SwerveEnclosure
	 */
	void StopMotor();

	/*
	 * Gets the swerve drive mode (FieldCentric - 0, RobotCentric - 1))
	 */
	int GetMode();
	/*
	 * Sets the swerve drive mode (kFieldCentric - 0, kRobotCentric - 1)
	 * Returns an error if the value given is incorrect
	 */
	void SetMode(DriveMode mode);

	/*
	 * Toggles the swerve drive mode (Toggles between field and robot centric)
	 */
	void ToggleMode();

private:
	//Stores the calculated values used by each wheel of the swerve drive
	std::shared_ptr<SwerveMath> mathSystem;

	//Stores the swerve enclosures being used
	std::shared_ptr<SwerveEnclosure> frontLeftWheel;
	std::shared_ptr<SwerveEnclosure> frontRightWheel;
	std::shared_ptr<SwerveEnclosure> rearLeftWheel;
	std::shared_ptr<SwerveEnclosure> rearRightWheel;

	//Stores the current mode of the swerve drive
	DriveMode m_mode = kFieldCentric;
};
