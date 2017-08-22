#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Quadraped.h"
#include "Leg.h"


	Quadraped::Quadraped()
	{
	
	}
	
	void Quadraped::walkDir(int theta, int vector, int elevate)
	{
		theta_ = theta;
		vector_ = vector;
		elevate_ = elevate;
	}
	
	
	
	void Quadraped::legIk(int legNr, int xLastPos, int yLastPos, int zLastPos);
	{
		
		xLastPos[legNr] = xLastPos;
		yLastPos[legNr] = yLastPos;
		zLastPos[legNr]= zLastPos;
		LegConstruct[legNr].calcLegIk(xLastPos, yLastPos, zLastPos);
	}
	void Quadraped::stepTiming(int stepLen, int stepTime)
	{
		
	stepTime_ = stepTime;	
	stepLen_ = stepLen;
	int incrementServos = 1;
	unsigned long updateInterval = stepTime_/stepLen_;
		
		for (int i = 0; i < 4; i++) {
			LegConstruct[i].setLegTiming(updateInterval, incrementServos);
		}
	}
	
	int Quadraped::translateBody(int legRF, int legLF, int legLB, int legRB, int newVector)
	{

	int flagArr[4] = {1 ,1, 1, 1};
	int translateFlag = 0;
	const int RIGHT_FRONT = 0, LEFT_FRONT = 1, LEFT_BACK = 2, RIGHT_BACK = 3;

		for (int i = 0; i < 4; i++) {
			xLeg[i] = xLastPos[i] + (-1 * (newVector * cos((theta_ - legOrientation[i]) * 3.14 / 180) - VECTOR_0 * cos((legOrientation[i] - legOrientation[i]) * 3.14 / 180)));
			zLeg[i] = yLastPos[i] + (-1 * (newVector * sin((theta_ - legOrientation[i]) * 3.14 / 180) - VECTOR_0 * sin((legOrientation[i] - legOrientation[i]) * 3.14 / 180)));
			yLeg[i] = yLastPos[i] + Y0 - elevate_;
		}
			//Serial.print(newVector);
			//Serial.print(" : ");
			//Serial.print(theta_);
			//Serial.print(" : ");
			//Serial.println(legOrientation[3]);
			//Serial.print(" : ");
			//Serial.print(" : ");
			//Serial.print(xLeg[3]);
			//Serial.print(" : ");
			//Serial.print(yLeg[3]);
			//Serial.print(" : ");
			//Serial.println(zLeg[3]);

		if (legRF == 1) {
			LegConstruct[RIGHT_FRONT].legIk(xLeg[RIGHT_FRONT], yLeg[RIGHT_FRONT], zLeg[RIGHT_FRONT]);
			flagArr[0] = LegConstruct[RIGHT_FRONT].updateLeg();
		}
		if (legLF == 1) {
			LegConstruct[LEFT_FRONT].legIk(xLeg[LEFT_FRONT], yLeg[LEFT_FRONT], zLeg[LEFT_FRONT]);
			flagArr[1] = LegConstruct[LEFT_FRONT].updateLeg();
		}
		if (legLB == 1) {
			LegConstruct[LEFT_BACK].legIk(xLeg[LEFT_BACK], yLeg[LEFT_BACK], zLeg[LEFT_BACK]);
			flagArr[2] = LegConstruct[LEFT_BACK].updateLeg();
		}
		if (legRB == 1) {
			LegConstruct[RIGHT_BACK].legIk(xLeg[RIGHT_BACK], yLeg[RIGHT_BACK], zLeg[RIGHT_BACK]);
			flagArr[3] = LegConstruct[RIGHT_BACK].updateLeg();
		}


		if (flagArr[0] + flagArr[1] + flagArr[2] + flagArr[3] == 4) { //Test if all legs have been updated.
			translateFlag = 1;
		}

		return translateFlag;
	}

	
	void Quadraped::initialise()
	{
		for (int i = 0; i < 4; i++) {
			LegConstruct[i].setLegInit();      //Set Initial parameters
			legOrientation[i] = 45 + 90 * i;
			LegConstruct[i].setLegTiming(10, 1);
			xLeg[i] = X0, yLeg[i] = Y0, zLeg[i] = Z0;
			xLastPos[i] = 0, yLostPos[i] = 0, zLastPos[i] = 0;
		}

	}
