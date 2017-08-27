#ifndef Leg_h
#define Leg_h

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

class Leg
{
  public:
    Leg(int coxaLen, int femurLen, int tibiaLen, int servoNrCoxa, int servoNrFemur, int servoNrTibia);
    void setLegAngles(int coxaAngle, int femurAngle, int tibiaAngle);
    void setLegInit();
    void convDegToPulse();
    void calcLegIk(int x, int y, int z);
    void setLegTiming(int updateInterval, int increment);
    int updateLeg();
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    int pulse[3], lastPulse[3], servoNr[3];
  private:
    int _coxaAngle, _femurAngle, _tibiaAngle;
    int _servoNrCoxa, _servoNrFemur, _servoNrTibia;
    int _x, _y, _z;
    int _updateInterval=1000, _increment=1;
    int coxaPulse, femurPulse, tibiaPulse, coxaPulseLastpos, femurPulseLastpos, tibiaPulseLastpos;
    int stepFlag = 1;
    int _coxaLen, _femurLen, _tibiaLen; // lengths of leg parts - coxa, femur, tibia
    int const SERVOMINIMUM = 100, SERVOMAXIMUM = 600;
    unsigned long lastUpdate; // last update of position
	int const SERVOMIN[12] = {239,238,262,252,256,256,234,258,234,252,242,252};
	int const SERVOMAX[12] = {453,460,470,468,486,486,446,480,470,462,462,466};


};

#endif
