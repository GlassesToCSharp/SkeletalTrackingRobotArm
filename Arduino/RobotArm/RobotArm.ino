#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include "SerialControl.h"

//#define ARM_DEMO
#define TIME_BETWEEN_REQUESTS_MS 1000
#define TIME_BETWEEN_EMPTY_REQUESTS_MS 1000

bool ledStatus = false;
long timer1 = 0;
long timer2 = 0;


void setup() 
{
  SerialInit();
  DynamixelInit();
  delay(1000);
  
  RequestData();
  
  timer1  = millis();
  timer2 = timer1;
}

void loop() 
{
  // put your main code here, to run repeatedly:

//  if(millis() - timer1 > 250)
//  {
//    timer1 = millis();
//    ledStatus = !ledStatus;
//    Dynamixel.ledStatus(ID,ledStatus);
//  }

#ifdef ARM_DEMO

#define ANGLE_DIFF  10
#define TIME_DIFF  100
  if(millis() - timer2 > TIME_DIFF)
  {
    static int shoulderPos = ID1_MIN;
    static int elbowPos = ID4_MIN;
    timer2 = millis();
//    int shoulderPos = random(ID1_MIN, ID1_MAX);
//    Serial.print("\tShoulder Z: ");
//    Serial.print(shoulderPos);
//    int elbowPos = random(ID4_MIN, ID4_MAX);
//    Serial.print("\t\tElbow Z: ");
//    Serial.println(elbowPos);
//    Dynamixel.moveSpeed(SHOULDER_YAW, shoulderPos, 100);
//    Dynamixel.moveSpeed(ELBOW_PITCH, elbowPos, 100);
    MoveToAngle(SHOULDER_YAW, shoulderPos);
    MoveToAngle(ELBOW_PITCH, elbowPos);

    Serial.println();

    shoulderPos += ANGLE_DIFF;
    elbowPos++;
  }
#undef ANGLE_DIFF
#undef TIME_DIFF
#else
  CheckAndHandleSerialInput();
#endif
}
