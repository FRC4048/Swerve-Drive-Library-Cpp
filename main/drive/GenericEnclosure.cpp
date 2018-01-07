#include "GenericEnclosure.h"

GenericEnclosure::GenericEnclosure(	std::string name,
									std::shared_ptr<SpeedController> m_moveMotor,
									std::shared_ptr<SpeedController> m_turnMotor,
									std::shared_ptr<Encoder> m_encoder,
									double m_gearRatio)
{
	this->name = name;
	moveMotor = m_moveMotor;
	turnMotor = m_turnMotor;
	gearRatio = m_gearRatio;

	encoder = m_encoder;
	controlPID.reset(new PIDController(10.0, 0.0, 0.0, encoder.get(), turnMotor.get()));
	controlPID->SetSetpoint(0.0);
	encoder->Reset();
}
GenericEnclosure::~GenericEnclosure(){ return; }

void GenericEnclosure::MoveWheel(double speedVal, double rotationVal)
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

void GenericEnclosure::StopWheel()
{
	moveMotor->StopMotor();
	turnMotor->StopMotor();
}

void GenericEnclosure::SetInverted(GenericEnclosure::MotorType type, bool val)
{
	if(type == MotorType::TurnMotor)
		turnMotor->SetInverted(val);
	else if(type == MotorType::MoveMotor)
		moveMotor->SetInverted(val);
}

void GenericEnclosure::SetPID(double P, double I, double D, double F)
{
	controlPID->SetPID(P, I, D, F);
}

double GenericEnclosure::GetEncoderVal()
{
	return encoder->Get();
}

void GenericEnclosure::SetSpeed(double speedVal)
{
	moveMotor->Set(speedVal);
}

void GenericEnclosure::SetAngle(double desiredAngle)
{
	controlPID->SetSetpoint(desiredAngle);
	controlPID->Enable();
}

bool GenericEnclosure::ShouldReverse(double wa)
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

double GenericEnclosure::ConvertAngle(double angle, double encoderValue)
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

std::string GenericEnclosure::GetName()
{
	return name;
}
