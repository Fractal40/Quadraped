#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MoveQuadraped.h"
#include "Leg.h"


MoveQuadraped::MoveQuadraped(int xBodyOffset, int yBodyOffset, int zBodyOffset, int legAngleOffset, int noLegs)
	{

		xBodyOffset_ = xBodyOffset;
		yBodyOffset_ = yBodyOffset;
		zBodyOffset_ = zBodyOffset;
		legAngleOffset_ = legAngleOffset;
		noLegs_ = noLegs;

	}

void MoveQuadraped::initialise()
	{
		for (int i = 0; i < noLegs_; i++) {
			legOrientation[i] = legAngleOffset_ + (360/noLegs_) * i;
			xLegInit[i] = COXA_LEN[i] + FEMUR_LEN[i];
			yLegInit[i] = TIBIA_LEN[i];
			zLegInit[i] = 0;

			//xLastPos[i] = 0, yLostPos[i] = 0, zLastPos[i] = 0;
			LegConstruct[i].setLegInit();      //Set Initial parameters
			LegConstruct[i].setLegTiming(5, 2);
		}

}

void MoveQuadraped::stepTiming(int stepLen, int stepTime)
{
	stepTime_ = stepTime;
	stepLen_ = stepLen;

}

void MoveQuadraped::bodyIk()
{


	theta = atan2(z_,x_) *180/ 3.14;
	vector = sqrt(pow(z_,2)+pow(x_,2));
	vector = constrain(vector, 0, 60);



	for (int i = 0; i < noLegs_; i++) {
		xLeg[i] = vector * cos((theta - legOrientation[i]) * 3.14 / 180) - xLegInit[i] * cos((legOrientation[i] - legOrientation[i]) * 3.14 / 180);
		zLeg[i] = vector * sin((theta - legOrientation[i]) * 3.14 / 180) - zLegInit[i] * sin((legOrientation[i] - legOrientation[i]) * 3.14 / 180);
		yLeg[i] = -1 * (y_ - yLegInit[i]); // -1 because y-axis is inverted
	}

}


void MoveQuadraped::getCoord(int x, int y, int z)
{
	x_ = x;
	y_ = y;
	z_ = z;
}

/*
int MoveQuadraped::legStep(int legNr)
{
	int stepCounterFlag=0;
	stepLen_ = vector_;
	const float STEPHEIGHT = 0.1;
	int xArr[stepLen_+1], zArr[stepLen_+1], yArr[stepLen_+1];

		for (int i = 0; i <= stepLen_; i++) {
			xArr[i] = xLastPos[legNr] + (i * cos((theta_ - legOrientation[legNr]) * 3.14 / 180) + VECTOR_0);
			zArr[i] = yLastPos[legNr] + (i * sin((theta_ - legOrientation[legNr])*3.14 / 180));
			yArr[i] = Y0 - (-STEPHEIGHT * pow(i, 2) + STEPHEIGHT * stepLen_ * i);
		}


		if (updateFlag == 1) {
			LegConstruct[legNr].legIk(xArr[legStepCounter], yArr[legStepCounter], zArr[legStepCounter]);
			//Serial.print(xArr[legStepCounter]);
			//Serial.print(" : ");
			//Serial.print(yArr[legStepCounter]);
			//Serial.print(" : ");
			//Serial.print(zArr[legStepCounter]);
			//Serial.print(" : ");
			//Serial.println(legStepCounter);
			if (legStepCounter < stepLen_) {
			legStepCounter++;
			//stepCounterFlag = 0;      //clear step flag
			} else {
			legStepCounter = 0;
			stepCounterFlag = 1;     //set step flag
			} //end if
		}
		updateFlag = 0;     //clear step fraction flag
		updateFlag = LegConstruct[legNr].updateLeg();
			//Serial.print(" : ");
			//Serial.println(legStepCounter);

		return stepCounterFlag;
	}

	void MoveQuadraped::legIk(int legNr, int xLastPos, int yLastPos, int zLastPos);
	{

		xLastPos[legNr] = xLastPos;
		yLastPos[legNr] = yLastPos;
		zLastPos[legNr]= zLastPos;
		LegConstruct[legNr].calcLegIk(xLastPos, yLastPos, zLastPos);
	}
	void MoveQuadraped::stepTiming(int stepLen, int stepTime)
	{

	stepTime_ = stepTime;
	stepLen_ = stepLen;
	int incrementServos = 1;
	unsigned long updateInterval = stepTime_/stepLen_;

		for (int i = 0; i < 4; i++) {
			LegConstruct[i].setLegTiming(updateInterval, incrementServos);
		}
	}
*/

int MoveQuadraped::pitchBody()
	{
		int translateFlag;
	int flagArr[4] = {1 ,1, 1, 1};
	int rollPitchFlag = 0;
	const int RIGHT_FRONT = 0, LEFT_FRONT = 1, LEFT_BACK = 2, RIGHT_BACK = 3;

	bodyIk();



	for (int i = 0; i < 4; i++) {
			xLeg[i] = -xLeg[i];
			zLeg[i] = -zLeg[i];
			yLeg[i] = yLeg[i];
		}

		if (x_ > 0) {
			yLeg[RIGHT_FRONT] = x_;
			yLeg[LEFT_FRONT] = x_;
			yLeg[RIGHT_BACK] = -x_;
			yLeg[LEFT_BACK] = -x_;
		}


			LegConstruct[RIGHT_FRONT].calcLegIk(xLeg[RIGHT_FRONT], yLeg[RIGHT_FRONT], zLeg[RIGHT_FRONT]);
			flagArr[0] = LegConstruct[RIGHT_FRONT].updateLeg();

			LegConstruct[LEFT_FRONT].calcLegIk(xLeg[LEFT_FRONT], yLeg[LEFT_FRONT], zLeg[LEFT_FRONT]);
			flagArr[1] = LegConstruct[LEFT_FRONT].updateLeg();

			LegConstruct[LEFT_BACK].calcLegIk(xLeg[LEFT_BACK], yLeg[LEFT_BACK], zLeg[LEFT_BACK]);
			flagArr[2] = LegConstruct[LEFT_BACK].updateLeg();

			LegConstruct[RIGHT_BACK].calcLegIk(xLeg[RIGHT_BACK], yLeg[RIGHT_BACK], zLeg[RIGHT_BACK]);
			flagArr[3] = LegConstruct[RIGHT_BACK].updateLeg();


		if (flagArr[0] + flagArr[1] + flagArr[2] + flagArr[3] == 4) { //Test if all legs have been updated.
			translateFlag = 1;
		}

		return translateFlag;
}

int MoveQuadraped::translateBody(int legRF, int legLF, int legLB, int legRB, int newVector)
	{

	int flagArr[4] = {1 ,1, 1, 1};
	int translateFlag = 0;
	const int RIGHT_FRONT = 0, LEFT_FRONT = 1, LEFT_BACK = 2, RIGHT_BACK = 3;

	bodyIk();

	for (int i = 0; i < 4; i++) {
			xLeg[i] = -xLeg[i];
			zLeg[i] = -zLeg[i];
			yLeg[i] = yLeg[i];
	}
	for (int i = 0; i < 4; i++) {
		LegConstruct[i].calcLegIk(xLeg[i], yLeg[i], zLeg[i]);
		flagArr[i] = LegConstruct[i].updateLeg();
	}
/*
		if (legRF == 1) {
			LegConstruct[RIGHT_FRONT].calcLegIk(xLeg[RIGHT_FRONT], yLeg[RIGHT_FRONT], zLeg[RIGHT_FRONT]);
			flagArr[0] = LegConstruct[RIGHT_FRONT].updateLeg();
		}
		if (legLF == 1) {
			LegConstruct[LEFT_FRONT].calcLegIk(xLeg[LEFT_FRONT], yLeg[LEFT_FRONT], zLeg[LEFT_FRONT]);
			flagArr[1] = LegConstruct[LEFT_FRONT].updateLeg();
		}
		if (legLB == 1) {
			LegConstruct[LEFT_BACK].calcLegIk(xLeg[LEFT_BACK], yLeg[LEFT_BACK], zLeg[LEFT_BACK]);
			flagArr[2] = LegConstruct[LEFT_BACK].updateLeg();
		}
		if (legRB == 1) {
			LegConstruct[RIGHT_BACK].calcLegIk(xLeg[RIGHT_BACK], yLeg[RIGHT_BACK], zLeg[RIGHT_BACK]);
			flagArr[3] = LegConstruct[RIGHT_BACK].updateLeg();
		}
*/
		if (flagArr[0] + flagArr[1] + flagArr[2] + flagArr[3] == 4) { //Test if all legs have been updated.
			translateFlag = 1;
		}

		return translateFlag;
}
