#ifndef MoveQuadraped_h
#define MoveQuadraped_h

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Leg.h>

class MoveQuadraped
{
	public:
		MoveQuadraped(int xBodyOffset, int yBodyOffset, int zBodyOffset, int legAngleOffset, int noLegs);
		int legStep(int legNr);
		void stepTiming(int stepLen, int stepTime);
		int translateBody(int legRF, int legLF, int legLB, int legRB, int newVector);
		void initialise();
		void walkDir(int theta, int vector, int elevate);

		//Leg Constructor: Leg(coxalength, femurlength, tibialength, coxa servoindex, femur servoindex, tibia servoindex)
		Leg LegConstruct[4] = {Leg(COXA_LEN[0], FEMUR_LEN[0], TIBIA_LEN[0], 6, 7, 8), Leg(COXA_LEN[1], FEMUR_LEN[1], TIBIA_LEN[1], 3, 4, 5), Leg(COXA_LEN[2], FEMUR_LEN[2], TIBIA_LEN[2], 9, 10, 11), Leg(COXA_LEN[3], FEMUR_LEN[3], TIBIA_LEN[3], 0, 1, 2)};


	private:
		void legIK(int legNr, xLastPos, yLastPos, zLastPos);
		int xLastPos[4], yLastPos[4], zLastPos[4];
		int stepTime_ = 500, stepLen_= 30;
		int legStepCounter = 0, moveSequence = 0, updateFlag = 1;

		const int COXA_LEN[4] = {30, 30, 30, 30}
		const int FEMUR_LEN[4] = {79, 79, 79, 79};
		const int TIBIA_LEN[4] = {107, 107, 107, 107};

		int xBodyOffset_;
		int yBodyOffset_;
		int zBodyOffset_;
		int legAngleOffset_;
		int noLegs_;

		int theta_, x_, y_, z_;

		int xLeg[4], yLeg[4], zLeg[4];

		int legOrientation[4];

};

#endif