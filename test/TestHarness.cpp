
#include "../main/math/SwerveMath.h"
#include <iostream>
#include <stdio.h>

/*
 * This is the test harness program for testing any changes made to the math
 * portion of the Swerve Drive Library.  This is currently a work in progress, and
 * lacks the complexity of the Java Swerve Drive Library.  This program also needs
 * to be compiled separately from other programs.
 */

using namespace std;

const double MARGIN_VALUE = 0.01;

void mathTester(double** input, double fla, double fls,
		 	 	 	 	 	 	  double fra, double frs,
								  double rla, double rls,
								  double rra, double rrs);

void compareToExpected(double input, double min, double max, char* tag);
double convertAngle(double angle);

int main()
{
    double encoders[4] = {0.2, 0.5, 0.05, -0.1};
    //double* values = SwerveMath::ConvertToAngle(encoders, (1988/1.2));
	SwerveMath* tempCalc = new SwerveMath(1, 1);
    double** output;

	//Nothing
	output = tempCalc->Calculate( 0.0,  0.0,  0.0);
	mathTester(output,  0.0,  0.0,
						  0.0,  0.0,
						  0.0,  0.0,
						  0.0,  0.0);

    //Forward
	output = tempCalc->Calculate( 1.0,  0.0,  0.0);
	mathTester(output,  0.0,  1.0,
						  0.0,  1.0,
						  0.0,  1.0,
						  0.0,  1.0);
    //Backward
	output = tempCalc->Calculate(-1.0,  0.0,  0.0);
	mathTester(output,  0.0, -1.0,
						  0.0, -1.0,
						  0.0, -1.0,
						  0.0, -1.0);
    //Clockwise
	output = tempCalc->Calculate( 0.0,  0.0,  1.0);
	mathTester(output,  0.125,  1.0,
                         -0.125, -1.0,
						 -0.125,  1.0,
						  0.125, -1.0);
    //Counter-Clockwise
	output = tempCalc->Calculate( 0.0,  0.0,  -1.0);
	mathTester(output,  0.125,  -1.0,
                         -0.125,  1.0,
						 -0.125, -1.0,
						  0.125,  1.0);
    //Strafe right
	output = tempCalc->Calculate( 0.0,  1.0,  0.0);
	mathTester(output,  0.25,  1.0,
                        0.25,  1.0,
                        0.25,  1.0,
                        0.25,  1.0);
    //Strafe left
	output = tempCalc->Calculate( 0.0, -1.0,  0.0);
	mathTester(output,  -0.25,  1.0,
                        -0.25,  1.0,
                        -0.25,  1.0,
                        -0.25,  1.0);
    //Diagonal Right
	output = tempCalc->Calculate( 1.0,  1.0,  0.0);
	mathTester(output,  0.125,  1.0,
                        0.125,  1.0,
                        0.125,  1.0,
                        0.125,  1.0);

    //Arc right
	output = tempCalc->Calculate( 1.0,  0.0,  1.0);
	mathTester(output,  0.0625,  1.0,
                        0.1875,  0.414,
                       -0.0625,  1.0,
                       -0.1875,  0.414);


    int temp;
    std::cin >> temp;

	return 0;
}

void mathTester(double** input, double fla, double fls,
		 	 	 	 	 	 	  double fra, double frs,
								  double rla, double rls,
								  double rra, double rrs)
{
    static int trialNum = 0;
    printf("Trial #%d\n", trialNum++);

	compareToExpected(input[0][0], fls - MARGIN_VALUE, fls + MARGIN_VALUE, "FLS");
	compareToExpected(input[0][1], fla - MARGIN_VALUE, fla + MARGIN_VALUE, "FLA");

	compareToExpected(input[1][0], frs - MARGIN_VALUE, frs + MARGIN_VALUE, "FRS");
	compareToExpected(input[1][1], fra - MARGIN_VALUE, fra + MARGIN_VALUE, "FRA");

	compareToExpected(input[3][0], rls - MARGIN_VALUE, rls + MARGIN_VALUE, "RLS");
	compareToExpected(input[3][1], rla - MARGIN_VALUE, rla + MARGIN_VALUE, "RLA");

	compareToExpected(input[2][0], rrs - MARGIN_VALUE, rrs + MARGIN_VALUE, "RRS");
	compareToExpected(input[2][1], rra - MARGIN_VALUE, rra + MARGIN_VALUE, "RRA");

    printf("\n\n");
}

void compareToExpected(double input, double min, double max, char* tag)
{
	bool value = !(max >= input && min <= input);

	if(value)
		printf("ERROR:  %s -> %f not between %f and %f\n\n", tag, input, min, max);
}

double convertAngle(double angle)
{
    double temp = angle;

    if(angle >  0.5) temp -= 1;
    if(angle < -0.5) temp += 1;

    return temp;
}
