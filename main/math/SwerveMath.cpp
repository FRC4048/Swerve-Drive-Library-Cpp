#include "SwerveMath.h"
#include <stdexcept>

SwerveMath::SwerveMath(double m_length, double m_width)
{
	if(m_length == 0.0 || m_width == 0.0)
		throw std::invalid_argument("Width and Length cannot be zero");

	LENGTH = m_length;
	WIDTH = m_width;
	R = sqrt((LENGTH*LENGTH) + (WIDTH*WIDTH));
}

double** SwerveMath::Calculate(double x, double y, double z, double angle)
{
	if(angle != -999.0)
	{
		angle = angle * PI / 180;
		double temp = x * cos(angle) + y * sin(angle);
		y = -x * sin(angle) + y * cos(angle);
		x = temp;
	}

	double A = y - z*(LENGTH/R);
	double B = y + z*(LENGTH/R);
	double C = x - z*(WIDTH/R);
	double D = x + z*(WIDTH/R);

	double wSpeed1 = sqrt(B*B + C*C);
	double wAngle1 = atan2(B,C) * 180/PI;

	double wSpeed2 = sqrt(B*B + D*D);
	double wAngle2 = atan2(B,D) * 180/PI;

	double wSpeed3 = sqrt(A*A + D*D);
	double wAngle3 = atan2(A,D) * 180/PI;

	double wSpeed4 = sqrt(A*A + C*C);
	double wAngle4 = atan2(A,C) * 180/PI;

	//normalizes speeds so they're within the ranges of -1 to 1
	double maxSpeed = wSpeed1;
	if(wSpeed2 > maxSpeed) maxSpeed = wSpeed2;
	if(wSpeed3 > maxSpeed) maxSpeed = wSpeed3;
	if(wSpeed4 > maxSpeed) maxSpeed = wSpeed4;

	if(maxSpeed > 1)
	{
		wSpeed1/=maxSpeed;
		wSpeed2/=maxSpeed;
		wSpeed3/=maxSpeed;
		wSpeed4/=maxSpeed;
	}

	//Normalizes angles so they are within -1 to 1
	wAngle1 = wAngle1 / 360.0;
	wAngle2 = wAngle2 / 360.0;
	wAngle3 = wAngle3 / 360.0;
	wAngle4 = wAngle4 / 360.0;

	double temp[4][2] =	{	{wSpeed2, wAngle2},
							{wSpeed1, wAngle1},
							{wSpeed4, wAngle4},
							{wSpeed3, wAngle3}
						};
	double** output = CopyArray(temp);

	return output;
}

double** SwerveMath::CopyArray(double array[][2])
{
	double** output = new double*[4];
	for(int i = 0; i < 4; i++)
	{
		output[i] = new double[2];
		for(int k = 0; k < 2; k++)
		{
			output[i][k] = array[i][k];
		}
	}

	return output;
}
