#include <iostream>

#pragma once

/**
 * Base class for enclosure. Implements common behavior that helps with the robot
 * driving:
 * - Move method that takes into account current position and optimizes the
 *   movement to reduce angle rotation
 * - Allows the wheel to make full rotation (when reaching full rotation don't go
 *   back to 0, rather keep rotation in same direction)
 * This class uses abstract lower-level implementations of setSpeed and setAngle
 * to be implemented by hardware-specific sub-classes
 */
class SwerveEnclosure {

public:
	virtual ~SwerveEnclosure() { return; } ;

	/*
	 * Returns the name of the enclosure
	 */
	virtual std::string GetName() = 0;

	/*
	 * Moves wheel at specified speed value and to specified rotational angle
	 */
	virtual void MoveWheel(double speedVal, double rotationAngle) = 0;
	/*
	 * Stops all movement of motors
	 */
	virtual void StopWheel() = 0;

	/*
	 * Outputs value of enclosures encoder as double
	 */
	virtual double GetEncoderVal() = 0;

};
