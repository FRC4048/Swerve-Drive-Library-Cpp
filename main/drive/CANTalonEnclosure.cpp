#include "CANTalonEnclosure.h"

CANTalonEnclosure::CANTalonEnclosure(	std::string name,
										std::shared_ptr<SpeedController> m_moveMotor,
										std::shared_ptr<CANTalon> m_turnMotor,
										double m_gearRatio)
{
	this->name = name;
	moveMotor = m_moveMotor;
	turnMotor = m_turnMotor;
	gearRatio = m_gearRatio;
}
CANTalonEnclosure::~CANTalonEnclosure(){ return; }

void CANTalonEnclosure::MoveWheel(double speedVal, double rotationVal)
{
	rotationVal = ConvertAngle(rotationVal, GetEncoderVal());

	if(ShouldReverse(rotationVal))
	{
		if(rotationVal < 0)
			rotationVal += 0.5;
		else
			rotationVal -= 0.5;

		speedVal *= -1;
	}

	SetSpeed(speedVal);
	if(speedVal != 0)
		SetAngle(rotationVal);
}

void CANTalonEnclosure::StopWheel()
{
	moveMotor->StopMotor();
	turnMotor->StopMotor();
}

void CANTalonEnclosure::SetInverted(CANTalonEnclosure::MotorType type, bool val)
{
	if(type == MotorType::TurnMotor)
	{
		turnMotor->SetInverted(val);
	}
	else if(type == MotorType::MoveMotor)
		moveMotor->SetInverted(val);
}

void CANTalonEnclosure::SetPID(double P, double I, double D, double F)
{
	turnMotor->SetPID(P, I, D, F);
}

//Outputs encoder values for the corresponding motor
double CANTalonEnclosure::GetEncoderVal()
{
	return turnMotor->GetEncPosition();
}

void CANTalonEnclosure::SetSpeed(double speedVal)
{
	moveMotor->Set(speedVal);
}

void CANTalonEnclosure::SetAngle(double desiredAngle)
{
	turnMotor->SetSetpoint(desiredAngle);
	turnMotor->Enable();
}

bool CANTalonEnclosure::ShouldReverse(double wa)
{
	double ea = GetEncoderVal();
	ea /= gearRatio;
	ea = fmod(ea, 1);

	//Convert the next wheel angle, which is from -.5 to .5, to 0 to 1
	if (wa < 0) wa += 1;

	//Find the difference between the two (not sure if the conversion from (-0.5 to 0.5) to (0 to 1) above is needed)
	//Difference between the two points. May be anything between -1 to 1, but we are looking for a number between -.5 to .5
	double longDifference = fabs(wa - ea);

	//finds shortest distance (0 to 0.5), always positive though (which is what we want)
	double difference = fmin(longDifference, 1.0-longDifference);

	//If the sum is greater than 1/4, then return true (aka it is easier for them to turn around and go backwards than go forward)
	if (difference > 0.25) return true;
	else return false;
}

double CANTalonEnclosure::ConvertAngle(double angle, double encoderValue)
{
	//angles are between -.5 and .5
	//This is to allow the motors to rotate in continuous circles (pseudo code on the Team 4048 forum)
	double encPos = encoderValue;
	encPos /= gearRatio;

	double temp = angle;
	temp += (int)encPos;

	encPos = fmod(encPos, 1);

	if ((angle - encPos) > 0.5) temp -= 1;

	if ((angle - encPos) < -0.5) temp += 1;

	return temp;
}

std::string CANTalonEnclosure::GetName()
{
	return name;
}
