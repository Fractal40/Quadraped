
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Leg.h"

Leg::Leg(int coxaLen, int femurLen, int tibiaLen, int servoNrCoxa, int servoNrFemur, int servoNrTibia)
  {
    _coxaLen = coxaLen;
    _femurLen = femurLen;
    _tibiaLen = tibiaLen;
    servoNr[0] = servoNrCoxa;
    servoNr[1] = servoNrFemur;
    servoNr[2] = servoNrTibia;

  }

  void Leg::setLegAngles(int coxaAngle, int femurAngle, int tibiaAngle)
  {
    _coxaAngle = coxaAngle;
    _femurAngle = femurAngle;
    _tibiaAngle = tibiaAngle;
    convDegToPulse();
  }

  void Leg::convDegToPulse()
  {

    //map pulse on degrees for each servo
    pulse[0] = map(_coxaAngle,45,135,SERVOMIN[servoNr[0]],SERVOMAX[servoNr[0]]);
    pulse[1] = map(_femurAngle,45,135,SERVOMIN[servoNr[1]],SERVOMAX[servoNr[1]]);
    pulse[2] = map(_tibiaAngle,45,135,SERVOMIN[servoNr[2]],SERVOMAX[servoNr[2]]);

	   //constrains servo movement between 45 and 135 degrees
	  pulse[0] = constrain(pulse[0], SERVOMIN[servoNr[0]], SERVOMAX[servoNr[0]]);
    pulse[1] = constrain(pulse[1], SERVOMIN[servoNr[1]], SERVOMAX[servoNr[1]]);
    pulse[2] = constrain(pulse[2], SERVOMIN[servoNr[2]], SERVOMAX[servoNr[2]]);

  }

  void Leg::setLegInit()   //Called at setup
  {
    setLegAngles(90,90,90);

    for (int servoIndex=0; servoIndex<3; servoIndex++) {
		    pwm.setPWM(servoNr[servoIndex],0,pulse[servoIndex]);
		    lastPulse[servoIndex] = pulse[servoIndex];    //Initial angles of servos, needed for setting initial pulse for updateLeg()
	  }
 }



  void Leg::calcLegIk(int x, int y, int z)
  {
    _x = x;
    _y = y;
    _z = z;
    float legLen,d,b1,b2;

    legLen = sqrt(_x*_x+_z*_z);
    _coxaAngle = atan2(_z,_x)*180/3,14;                                                                 //calculate angle a
    d= sqrt(pow((legLen-_coxaLen),2)+(_y*_y));
    b1 = atan2((legLen-_coxaLen),(_y))*180/3.14;
    b2 = acos((pow(d,2)+pow(_femurLen,2)-pow(_tibiaLen,2))/(2*d*_femurLen))*180/3.14;
    _femurAngle = (b1+b2)-90;                                                                          //calculate angle b
    _tibiaAngle = acos((-pow(d,2)+pow(_tibiaLen,2)+pow(_femurLen,2))/(2*_femurLen*_tibiaLen))*180/3.14;    //calculate angle c

    _tibiaAngle= _tibiaAngle;       //Adjust for servo orientations
    _femurAngle=_femurAngle+90;
    _coxaAngle=_coxaAngle+90;

    convDegToPulse();
   }

void Leg::setLegTiming(int updateInterval, int increment)
{
    _updateInterval = updateInterval;
    _increment = increment;
}

int Leg::updateLeg()
{

  int incrementServo[3];
  int servoShortestPulse;
  int shortestPulse;


  if((millis() - lastUpdate) > _updateInterval)
  {

    shortestPulse = abs(lastPulse[0] - pulse[0]);
    servoShortestPulse = 0;

    for (int servoIndex=1; servoIndex<3; servoIndex++) {
      if (abs(lastPulse[servoIndex] - pulse[servoIndex]) < shortestPulse) {
        servoShortestPulse = servoIndex;
        shortestPulse = abs(lastPulse[servoIndex] - pulse[servoIndex]);
      } //end if
    } //end for

    for (int servoIndex=0; servoIndex<3; servoIndex++) {
      if (lastPulse[servoShortestPulse]-pulse[servoShortestPulse] != 0) {
        incrementServo[servoIndex] = abs(lastPulse[servoIndex]-pulse[servoIndex]) / abs(lastPulse[servoShortestPulse]-pulse[servoShortestPulse]);
      } //end if
    } // end for



    lastUpdate = millis();


        for (int servoIndex=0; servoIndex<3; servoIndex++){
            if (lastPulse[servoIndex] > pulse[servoIndex]) {
              if ((lastPulse[servoIndex] - pulse[servoIndex]) < incrementServo[servoIndex]) {
                  incrementServo[servoIndex] = (lastPulse[servoIndex] - pulse[servoIndex]);
              } else {
                incrementServo[servoIndex] = incrementServo[servoIndex];
              } // end if else
              lastPulse[servoIndex] -= incrementServo[servoIndex];
              pwm.setPWM(servoNr[servoIndex],0,lastPulse[servoIndex]);
              //Serial.print(servoIndex);
              //Serial.print(" : ");
              //Serial.print(_increment);
              //Serial.print(" : ");
              //Serial.print(incrementServo[servoIndex]);
              //Serial.print(" : ");
              //Serial.print(lastPulse[servoIndex]);
              //Serial.print(" : ");
              //Serial.println(pulse[servoIndex]);
            } // end if
            if (lastPulse[servoIndex] < pulse[servoIndex]) {
              if ((pulse[servoIndex] - lastPulse[servoIndex]) < incrementServo[servoIndex]) {
                  incrementServo[servoIndex] = (pulse[servoIndex] - lastPulse[servoIndex]);
              } else {
                incrementServo[servoIndex] = incrementServo[servoIndex];
              } // end if else
              lastPulse[servoIndex] += incrementServo[servoIndex];
			        pwm.setPWM(servoNr[servoIndex],0,lastPulse[servoIndex]);
              //Serial.print(servoIndex);
              //Serial.print(" : ");
              //Serial.print(_increment);
              //Serial.print(" : ");
              //Serial.print(incrementServo[servoIndex]);
              //Serial.print(" : ");
              //Serial.print(lastPulse[servoIndex]);
              //Serial.print(" : ");
              //Serial.println(pulse[servoIndex]);
            } // end if

          } // end for

  }


       if (lastPulse[0] == pulse[0] && lastPulse[1] == pulse[1] && lastPulse[2] == pulse[2]){
        stepFlag = 1;
		} else {
        stepFlag = 0;
		}
      return stepFlag;

}
