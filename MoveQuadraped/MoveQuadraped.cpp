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
			LegConstruct[i].setLegTiming(50, 1);
		}

}

void MoveQuadraped::stepTiming(int stepLen, int stepTime)
{
	stepTime_ = stepTime;
	stepLen_ = stepLen;

}

	void MoveQuadraped::bodyIk(int legRF, int legLF, int legLB, int legRB)
	{
		int legIkFlag[] = {legRF, legLF, legLB, legRB};

		for (int i = 0; i < noLegs_; i++) {
			if (legIkFlag[i] == 1) {
				xLeg[i] = vector * cos((theta - legOrientation[i]) * 3.14 / 180) - xLegInit[i] * cos((legOrientation[i] - legOrientation[i]) * 3.14 / 180);
				zLeg[i] = vector * sin((theta - legOrientation[i]) * 3.14 / 180) - zLegInit[i] * sin((legOrientation[i] - legOrientation[i]) * 3.14 / 180);
				yLeg[i] = -1 * (y_ - yLegInit[i]); // -1 because y-axis is inverted
			}

		} // end for

	}


	void MoveQuadraped::getCoord(int x, int y, int z)
	{
		x_ = x;
		y_ = y;
		z_ = z;

		theta = atan2(z_,x_) *180/ 3.14;
		vector = sqrt(pow(z_,2)+pow(x_,2));
		vector = constrain(vector, 0, 40);

		theta = 90;
		vector = 40;
		stepLen_ = vector;

	}




	void MoveQuadraped::legStep(int legNr)
	{

		float STEPHEIGHT = 0.2;

		switch (legNr) {
			case 0:
				bodyIk(1, 0, 0, 0);
				break;
			case 1:
				bodyIk(0, 1, 0, 0);
				break;
			case 2:
				bodyIk(0, 0, 1, 0);
				break;
			case 3:
				bodyIk(0, 0, 0, 1);
				break;
		}
		delay(1);

		xLeg[legNr] = xLeg[legNr];// - stepLen_;
		zLeg[legNr] = zLeg[legNr];
		yLeg[legNr] = yLegInit[legNr] - (-STEPHEIGHT * pow(stepCounter,2) + STEPHEIGHT * stepLen_ * vector);

		LegConstruct[legNr].calcLegIk(xLeg[legNr], yLeg[legNr], zLeg[legNr]);
		//Serial.print(legNr);
		//Serial.print(" : ");
		//Serial.print(xLeg[legNr]);
		//Serial.print(" : ");
		//Serial.print(yLeg[legNr]);
		//Serial.print(" : ");
		//Serial.println(zLeg[legNr]);
	}

	void MoveQuadraped::walk()
	{

		int updateFlag = 0;

		Serial.print(stepCounter);
		Serial.print(" : ");
		Serial.print(vector);
		Serial.print(" : ");
		Serial.println(step);

		switch (step) {
			case 0:
				if (stepCounter <= stepLen_) {
					vector = stepCounter;
					legStep(2);
					//translateBody(1,1,0,1);
					updateFlag = updateBody();

					if (updateFlag == 1) {
							stepCounter++;
					}

				} else {
					stepCounter = 1;
					step = 0;                 // <---- next step
				}
				break;
			case 1:
				if (stepCounter <= stepLen_) {
					vector = stepCounter;
					legStep(0);
					vector = stepCounter + 10;
					//translateBody(0,1,1,1);
					//updateFlag = updateBody();
					if (LegConstruct[0].updateLeg() == 1) {
							stepCounter++;
					}
				} else {
					stepCounter = 1;
					step = 2;
				}
				break;
			case 2:
				if (stepCounter <= stepLen_) {
					vector = stepCounter;
					legStep(3);
					vector = stepCounter + 20;
					//translateBody(1,1,1,0);
					//updateFlag = updateBody();
					if (LegConstruct[3].updateLeg() == 1) {
							stepCounter++;
					}
				} else {
					stepCounter = 1;
					step = 3;
				}
				break;
			case 3:
				if (stepCounter <= stepLen_) {
					vector = stepCounter;
					legStep(1);
					vector = stepCounter + 30;
				  //translateBody(1,0,1,1);
					//updateFlag = updateBody();
					if (LegConstruct[1].updateLeg() == 1) {
							stepCounter++;
					}
				} else {
					stepCounter = 1;
					step = 0;
				}
				break;

		}
		delay(1);
		//Serial.print(step);
		//Serial.print(" : ");
		//Serial.print(stepCounter);
		//Serial.print(" : ");
		//Serial.println(vector);

	}

void MoveQuadraped::translateBody(int legRF, int legLF, int legLB, int legRB)
{

	int legIkFlag[] = {legRF, legLF, legLB, legRB};

	bodyIk(legRF, legLF, legLB, legRB);

	for (int i = 0; i < 4; i++) {
		if (legIkFlag[i] == 1) {
			xLeg[i] = -xLeg[i];
			zLeg[i] = -zLeg[i];
			yLeg[i] = yLeg[i];
		}
	}

	for (int i = 0; i < 4; i++) {
		if (legIkFlag[i] == 1) {
			LegConstruct[i].calcLegIk(xLeg[i], yLeg[i], zLeg[i]);
		}
	}

}

int MoveQuadraped::updateBody()
{
	int flagArr[] = {1, 1, 1, 1};

	for (int i = 0; i < 4; i++) {
		flagArr[i] = LegConstruct[i].updateLeg();

	}


	if (flagArr[0] + flagArr[1] + flagArr[2] + flagArr[3] == 4) {
		return 1;
	} else {
		return 0;
	}

}
