# Team 4048 Swerve Drive Module
The swerve drive library encapsulates the code necessary to deploy a swerve drive sustem to your robot.
The library is designed with the goal of being adjustable to various hardware configuration and is distributed with some hardware implementations out-of-the-box.

The swerve drive module is divided into several subsystems:
## Math
Responsible for the mathematical calculations required to drive the robot.
The main class (`ServeMath`) contains the methods that would take a drive command input (e.g. Joystick
values) and respond with a drive directive - the speed and angle to set on each drive wheel.
This code is **not** driving the robot - it is simply performing the calculations needed. Also,
the code has no external dependencies and can be adapted to work with any hardware.
## Drive
Responsible for actually interacting with the robot. Its main class (`RobotDriveSwerve`)
is the orchestrator of the robot movement: it takes the drive input, sends it to the Math subsystem
and then drives the hardware through the use of `SwerveEnclosure`.
## Hardware Abstraction
In order to facilitate reuse and testing, the subsystem uses an abstraction layer that allows it to be independent of the hardware
actually used on the robot. This is achieved through a pure virtual class (`SwerveEnclosure`) that declares the
methods required by the rest of the library. Various interfaces for the Enclosure are available:
- CANTalonEnclosure
- GenericEnclosure

Naturally, users of the library can add new implementations for their hardware.

# Usage
Below is a sample usage of the library with the CANTalon enclosure. Modify this to fit your setup, as necessary:

```C++

.
.
.
#include "main/drive/RobotDriveSwerve.h"
#include "main/drive/CANTalonEnclosure.h"

    const double GEAR_RATIO = (1988/1.2);
    const double L = 19;
    const double W = 27.5;

    // --- MOTOR LAYOUT ---
    /*
	 *                  Front
	 *      Wheel 2 -------------- Wheel 1
	 *          |                   |
	 *          |                   |
	 *          |                   |
	 *   Left   |                   |   Right
	 *          |                   |
	 *          |                   |
	 *          |                   |
	 *      Wheel 3 -------------- Wheel 4
	 *                  Back
	 */

    int main() {
	
	//Enclosure initialization
        CANTalonEnclosure swerveEnclosure1 = new CANTalonEnclosure("enc 1", RobotMap::swerveDriveSpeedController1, RobotMap::swerveDriveCANTalon1, GEAR_RATIO);
        CANTalonEnclosure swerveEnclosure2 = new CANTalonEnclosure("enc 2", RobotMap::swerveDriveSpeedController2, RobotMap::swerveDriveCANTalon1, GEAR_RATIO);
        CANTalonEnclosure swerveEnclosure3 = new CANTalonEnclosure("enc 3", RobotMap::swerveDriveSpeedController3, RobotMap::swerveDriveCANTalon1, GEAR_RATIO);
        CANTalonEnclosure swerveEnclosure4 = new CANTalonEnclosure("enc 4", RobotMap::swerveDriveSpeedController4, RobotMap::swerveDriveCANTalon1, GEAR_RATIO);

	//Swerve Drive initialization
        RobotDriveSwerve swerveDrive = new SwerveDrive(	swerveEnclosure1, swerveEnclosure2, swerveEnclosure3, swerveEnclosure4,
					W, L);
	
	//Move robot forward at maximum speed
	swerveDrive.move(1.0, 0.0, 0.0);
    }

```