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
    void setLegParabola(float coxaParabola, float femurParabola, float tibiaParabola);
    int updateLeg();


  private:
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    float lastPulse[3];
    int pulse[3], servoNr[3];
    int _coxaAngle, _femurAngle, _tibiaAngle;
    int _servoNrCoxa, _servoNrFemur, _servoNrTibia;
    int _x, _y, _z;
    int _xOffset, _yOffset, _zOffset;
    int _updateInterval=500, _increment=1;
    int coxaPulse, femurPulse, tibiaPulse, coxaPulseLastpos, femurPulseLastpos, tibiaPulseLastpos;
    int _coxaLen, _femurLen, _tibiaLen; // lengths of leg parts - coxa, femur, tibia
    float stepHeight[3] = {0, 0, 0};
    int stepLengthFlag = 1, stepLength;
    unsigned long lastUpdate; // last update of position

    int const SERVOMINIMUM = 100, SERVOMAXIMUM = 600;
	  int const SERVOMIN[12] = {239,238,262,252,256,256,234,258,234,252,242,252};
	  int const SERVOMAX[12] = {453,460,470,468,486,486,446,480,470,462,462,466};


};

#endif
