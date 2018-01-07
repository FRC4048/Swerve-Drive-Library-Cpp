
#pragma once
#include "WPILib.h"

/**
 * This class handles the calculations required to drive a robot using SwerveDrive
 * wheels.  This class supports both robot centric and field centric modes.  Field
 * centric mode causes the swerve system to always keep the forward/backward and
 * directional movement to stay relative to zero degrees.  Robot centric mode
 * causes the swerve system to act with normal cartesian movement.
 */
class SwerveMath {

public:
	/*
	 * Requires the length and width between swerve wheels in order to have
	 * be accurate with calculations.
	 */
	SwerveMath(double length, double width);
	/*
	 * Uses foward speed, strafing speed, and rotational speed values to calculate
	 * the required angle and speed for each wheel.  An angle can also be given so
	 * that field centric mode can be used.  If no angle is given (or equal to -999)
	 *  robot centric will be used.
	 *
	 * FORWARD: positive value = forward movement, negative value = backward
	 * movement
	 * STRAFE: positive value = right direction, negative value = left direction
	 * ROTATION: positive value = clockwise rotation, negative value =
	 * counterclockwise rotation
	 *
	 * Method outputs an array of speed and rotation value for each wheel.
	 * 		0					1
	 * 	0	Front Left Speed	Front Left Angle
	 * 	1	Front Right Speed	Front Right Angle
	 * 	2	Rear Left Speed		Rear Left Angle
	 * 	3	Rear Right Speed	Rear Right Angle
	 */
	double** Calculate(double fwd, double str, double rot, double angle = -999);

private:
	static constexpr double NO_ANGLE = -999;
	static constexpr double PI = acos(-1.0);

	/*
	 * Copies the speed and angle values into a pointer in order to be used by
	 * the enclosures
	 */
	double** CopyArray(double array[][2]);

	double LENGTH, WIDTH;
	double R;
};
