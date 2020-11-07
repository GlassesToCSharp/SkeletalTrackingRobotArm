#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include "SerialControl.h"

//#define ARM_DEMO

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
  if(millis() - timer2 > 3000)
  {
    timer2 = millis();
    int shoulderPos = random(ID1_MIN, 3 * ID1_MAX / 4);
    Serial.print("\tShoulder Z: ");
    Serial.print(shoulderPos);
    int elbowPos = random(ID4_MIN, ID4_MAX);
    Serial.print("\t\tElbow Z: ");
    Serial.println(elbowPos);
    Dynamixel.moveSpeed(ELBOW_PITCH, elbowPos, 100);
    Dynamixel.moveSpeed(SHOULDER_YAW, shoulderPos, 100);
  }
#else
  if (GetTimeSinceLastRequest() > 1000) {
    RequestData();
  }

  CheckAndHandleSerialInput();
#endif
}
