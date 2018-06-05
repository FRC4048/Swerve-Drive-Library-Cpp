#pragma once

#include "WPILib.h"
#include "SwerveEnclosure.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "ctre/Phoenix.h"

#include <math.h>

/*
 * Used for enclosing a CANTalon speed controller (rotational movement) and a
 * speed controller (directional movement) in order to control a single swerve
 * wheel.  The CANTalon speed controller must have an encoder wired to it in
 * order for this enclosure to function.
 *
 * This class inherits from the SwerveEnclosure class, used by
 * RobotDriveSwerve objects in order for easy control of the swerve system. This
 * class can be inherited from to allow for modification and support different
 * hardware designs.
 *
 * This enclosure requires CTRE's CANTalon libraries in order to be used.
 */
class CANTalonEnclosure : public SwerveEnclosure {

public:
	enum MotorType{
		MoveMotor,
		TurnMotor
	};

	/*
	 * Requires any speed controller for controlling wheel direction, a CANTalon
	 * controller to control wheel rotation, and a gear ratio value.  The default
	 * gear ratio is 1988/1.2.
	 */
	CANTalonEnclosure(	std::string name,
					std::shared_ptr<SpeedController> m_moveMotor,
					std::shared_ptr<WPI_TalonSRX> m_turnMotor,
					double m_gearRatio);
	~CANTalonEnclosure();

	/*
	 * Move the wheel to the given speed and rotational values.
	 * Method is used by RobotDriveSwerve for controlling each individual swerve
	 * wheel.
	 */
	void MoveWheel(double speedVal, double rotationVal) override;
	/*
	 * Stops all movement of an enclosures motors
	 */
	void StopWheel() override;
	/*
	 * Used to invert a corresponding speed controller, based off of the motor type
	 * given.  If the value is set to true, the controller is inverted, and if the
	 * value is set to false. the controller is uninverted.
	 */
	void SetInverted(MotorType type, bool val);
	/*
	 * Sets the PIDF value being used by a swerve enclosure
	 */
	void SetPID(double P, double I, double D, double F = 0);
	/*
	 * Outputs encoder values for the corresponding motor
	 */
	double GetEncoderVal() override;
	/*
	 * Returns the name of the enclosure
	 */
	std::string GetName() override;
	/*
	 * Reverse encoder direction
	 */
	void SetReverseEncoder(bool reverseEncoder);
	/*
	 * Reverse steer motor direction
	 */
	void SetReverseSteerMotor(bool reverseSteer);
private:
	/*
	 * Using the desired angle for the wheel and the current encoder position,
	 * it determines if the wheel could be efficient by reversing the rotation
	 * and movement direction.
	 */
	bool ShouldReverse(double desiredPos);
	/*
	 * Sets the speed value to the movement motor
	 */
	void SetSpeed(double speedVal);
	/*
	 * Sets the that the wheel should turn to
	 */
	void SetAngle(double rotationVal);
	/*
	 * Converts the given angle to a range of -0.5 to 0.5.
	 */
	double ConvertAngle(double angle, double encoderValue);

	std::shared_ptr<SpeedController> moveMotor;
	std::shared_ptr<WPI_TalonSRX> turnMotor;

	std::string name;
	double gearRatio = 1988/1.2;
	bool reverseEncoder = false;
	bool reverseSteer = false;
};

