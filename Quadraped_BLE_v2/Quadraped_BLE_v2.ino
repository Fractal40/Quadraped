#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Leg.h>
#include <MoveQuadraped.h>

//Joystick
const int SLAVE_ADDRESS = 0x71;
const int CONDATA_BYTES = 5;
int conData[5];
int theta, vector, elevate;
boolean aButton = false, bButton = false;
//smoothing
const int numReadings = 20;
float zReadings[numReadings], xReadings[numReadings];  // the readings from controller input
int zReadIndex = 0, xReadIndex = 0;              // the index of the current reading
float zTotal = 0, xTotal = 0;                  // the running total
float zAverage = 0, xAverage = 0;                // the average
//endjoystick

int zVector;
int xVector;
int yVector=0;

int moveSequence = 0;

//timer
unsigned long lastTimeConData;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MoveQuadraped Move = MoveQuadraped(34, 0, 34, 45, 4);



void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(100);

  //Joystick_start
  elevate = 0;
  //Joystick_end

  Move.initialise();

  //Move.stepTiming(30, 200);
  //Move.walkDir(90, 30, 0);
  delay(1000);

}

void loop() {
  

  if (millis() - lastTimeConData > 20){
    lastTimeConData = millis();
    getConData();
  }
  //Serial.print(zVector);
  //Serial.print(" : ");
  //Serial.println(xVector);

    //vector = 30; // from getConData()
  //theta = 90;  // from getConData()
  //zVector = -50;
  //xVector = -50;
  //yVector = 0;
  
  Move.getCoord(xVector, yVector, zVector);
  //Move.pitchBody();
  Move.translateBody(1,1,1,1,30);
/*
  if (vector > 0) {
    int moveFlag;

    switch (moveSequence) {
      case 0:
        //Serial.println("CASE 0");
        moveFlag = Move.legStep(3);   //step with leg 3 (RB) and translate body

        if (moveFlag == 1) {
          moveSequence = 4;
          moveFlag = 0;

        } else {
          break;
        } //end if_else
      case 1:
        //Serial.println("CASE 1");

        moveFlag = Move.legStep(0);   //step with leg 0 (RF) and translate body

        if (moveFlag == 1) {

          moveSequence = 2;
          moveFlag = 0;

        } else {
          break;
        } //end if_else
      case 2:
        //Serial.println("CASE 2");
        moveFlag = Move.legStep(2);   //step with leg 2 (LB) and translate body

        if (moveFlag == 1) {
          moveSequence = 3;
          moveFlag = 0;

        } else {
          break;
        } //end if_else
      case 3:
        //Serial.println("CASE 3");
        moveFlag = Move.legStep(1);   //step with leg 1 (LF) and translate body

        if (moveFlag == 1) {

          moveSequence = 4;
          moveFlag = 0;

        } else {
          break;
        } //end if_else
	  case 4:
		moveFlag = Move.translateBody(0,0,0,1,30);
		//Serial.println("CASE 4");
        if (moveFlag == 1) {
          moveSequence = 0;
          moveFlag = 0;

        } else {
          break;
        } //end if_else
    }
  }

*/

}






void getConData()
{


  
  int cnt = 0;
  Wire.requestFrom(SLAVE_ADDRESS, CONDATA_BYTES);
  while (Wire.available()) {
    conData[cnt] = Wire.read();
    cnt++;
  }

  if (conData[0] == 255) {
    zVector = int(conData[2]);
    xVector = int(conData[1]);
    aButton = boolean(conData[3]);
    bButton = boolean(conData[4]);

    xVector = xVector * 5 - 250;
    zVector = zVector * 5 - 250;

  } else {
    xVector = 0;
    zVector = 0;
    aButton = false;
    bButton = false;
  } //end if

  if (aButton == true && yVector > -40) {
    yVector--;  //body up
  } else if (bButton == true && yVector < 40) {
    yVector++;  //body down
  } //end if

//Smoothing of controller data to reduce jittering

//z reading
  zTotal = zTotal - zReadings[zReadIndex];
  zReadings[zReadIndex] = zVector; 
  zTotal = zTotal + zReadings[zReadIndex];
  zReadIndex = zReadIndex + 1;
  
  if (zReadIndex >= numReadings) {
     zReadIndex = 0;
     
  }

  zVector = zTotal / numReadings;  //smoothed zVector data
  

  //horizontal reading
  xTotal = xTotal - xReadings[xReadIndex];
  xReadings[xReadIndex] = xVector; 
  xTotal = xTotal + xReadings[xReadIndex];
  xReadIndex = xReadIndex + 1;
  
  if (xReadIndex >= numReadings) {
     xReadIndex = 0;
     
    
  }

  xVector = xTotal / numReadings; //smoothed xVector data

  

      
}
