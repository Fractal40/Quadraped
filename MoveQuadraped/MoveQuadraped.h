#ifndef MoveQuadraped_h
#define MoveQuadraped_h

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Leg.h>

class MoveQuadraped
{
	public:
		MoveQuadraped();
		int legStep(int legNr);
		void stepTiming(int stepLen, int stepTime);
		int translateBody(int legRF, int legLF, int legLB, int legRB, int newVector);
		void initialise();
		void walkDir(int theta, int vector, int elevate);
	
		//Leg Constructor: Leg(coxalength, femurlength, tibialength, coxa servoindex, femur servoindex, tibia servoindex)
		Leg LegConstruct[4] = {Leg(30, 79, 107, 6, 7, 8), Leg(30, 79, 107, 3, 4, 5), Leg(30, 79, 107, 9, 10, 11), Leg(30, 79, 107, 0, 1, 2)};
		

	private:
		void legIK(int legNr, xLastPos, yLastPos, zLastPos);
		int xLastPos[4], yLastPos[4], zLastPos[4];
		int stepTime_ = 500, stepLen_= 30;
		int legStepCounter = 0, moveSequence = 0, updateFlag = 1;
		const int VECTOR_0 = 120;
	
		const int X0 = 120;
		const int Z0 = 0;
		const int Y0 = 100;
		
		
		int theta_, vector_, elevate_;
	
		int xLeg[4], yLeg[4], zLeg[4];
		int Leg_offset[4];
		int legOrientation[4];
		int x_offset[4], z_offset[4];
};

#endif
