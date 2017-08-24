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
//endjoystick

int zVector;
int xVector;
int yVector=0;

int moveSequence = 0;


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


}

void loop() {


  getConData();
  //translateBody(1, 1, 1, 1, vector);
    //vector = 30; // from getConData()
  //theta = 90;  // from getConData()
  Move.getCoord(xVector, yVector, zVector);
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
   

    theta = atan2(zVector, xVector) * 180 / 3.14;
    vector = sqrt(pow(zVector, 2) + pow(xVector, 2));
    vector = constrain(vector, 0, 40);
  } else {
    theta = 0;
    vector = 0;
    aButton = false;
    bButton = false;
  } //end if

  if (aButton == true && yVector > -40) {
    yVector--;  //body up
  } else if (bButton == true && yVector < 40) {
    yVector++;  //body down
  } //end if


      
}
