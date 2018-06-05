#include "RobotDriveSwerve.h"

RobotDriveSwerve::RobotDriveSwerve(SwerveEnclosure* frontLeftWheel,
								   SwerveEnclosure* frontRightWheel,
								   SwerveEnclosure* rearLeftWheel,
								   SwerveEnclosure* rearRightWheel,
								   double width, double length)

{
	if(length == 0.0 || width == 0.0)
		throw std::invalid_argument("Swerve drive Width/Length cannot be zero");

	this->frontLeftWheel.reset(frontLeftWheel);
	this->frontRightWheel.reset(frontRightWheel);
	this->rearLeftWheel.reset(rearLeftWheel);
	this->rearRightWheel.reset(rearRightWheel);

	mathSystem.reset(new SwerveMath(length, width));
}

void RobotDriveSwerve::move(double x, double y, double rotation, double angle)
{
	double** wheelValues;
	x *= -1;

	if(GetMode() == kFieldCentric) {
		wheelValues = mathSystem->Calculate(x, y, rotation, angle);
	}
	else {
		wheelValues = mathSystem->Calculate(x, y, rotation);
	}

	frontLeftWheel->MoveWheel(	wheelValues[0][0], wheelValues[0][1]);
	frontRightWheel->MoveWheel(	wheelValues[1][0], wheelValues[1][1]);
	rearLeftWheel->MoveWheel(	wheelValues[2][0], wheelValues[2][1]);
	rearRightWheel->MoveWheel(	wheelValues[3][0], wheelValues[3][1]);
}

void RobotDriveSwerve::StopMotor()
{
	frontLeftWheel->StopWheel();
	frontRightWheel->StopWheel();
	rearLeftWheel->StopWheel();
	rearRightWheel->StopWheel();
}

int RobotDriveSwerve::GetMode()
{
	return m_mode;
}
void RobotDriveSwerve::SetMode(DriveMode mode)
{
	switch(mode)
	{
	case(kFieldCentric):
		m_mode = kFieldCentric;
		break;
	case(kRobotCentric):
		m_mode = kRobotCentric;
		break;
	default:
		DriverStation::ReportError("ERROR: Invalid drivetrain mode input");
	}
}
