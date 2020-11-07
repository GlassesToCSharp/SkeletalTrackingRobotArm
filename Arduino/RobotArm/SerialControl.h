#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H

#include "Constants.h"

bool hasStartMessageBeenReceived = false;
bool isCurrentValueNegative = false;
int elbowAngle = 0;
int shoulderPitch = 0;
int shoulderYaw = 0;
int shoulderRoll = 0;

long requestTimer = 0;

enum MessageIndex
{
  StartByte,
  ShoulderYaw,
  ShoulderPitch,
  ShoulderRoll,
  ElbowPitch,
  EndByte
};

MessageIndex currentMessageIndex = EndByte; // Expecting a start byte

/* ---------------------------- */

void FlushReceiveBuffer();
void TestShoulderAndElbow(char* readByte);
void TestElbowPitch(char* readByte);
void TestShoulderPitch(char* readByte);

void RequestData()
{
  Serial.write(startByte);
  Serial.write('R');
  Serial.write(endByte);

  requestTimer = millis();
}

void SerialInit()
{
  Serial.begin(115200);
  FlushReceiveBuffer();
}


void FlushReceiveBuffer()
{
  if (Serial)
  {
    while (Serial.available())
    {
      Serial.read();
    }
  }
}


void CheckAndHandleSerialInput()
{
  while (Serial.available() > 0)
  {
    char readByte = Serial.read();
    //    TestElbowPitch(&readByte);
    //    TestShoulderPitch(&readByte);
    TestShoulderAndElbow(&readByte);
  }
}


void PrepareForNextJoint()
{
  switch (currentMessageIndex)
  {
    case StartByte:
      currentMessageIndex = ShoulderYaw;
      break;

    case ShoulderYaw:
      if (isCurrentValueNegative)
      {
        shoulderYaw = -shoulderYaw;
      }
      currentMessageIndex = ShoulderPitch;
      break;

    case ShoulderPitch:
      if (isCurrentValueNegative)
      {
        shoulderPitch = -shoulderPitch;
      }
      currentMessageIndex = ShoulderRoll; // CHANGE TO SHOULDER ROLL
      break;

    case ShoulderRoll:
      //            if (isCurrentValueNegative)
      //            {
      //                shoulderRoll = -shoulderRoll;
      //            }
      currentMessageIndex = ElbowPitch;
      break;

    case ElbowPitch:
      //            if (isCurrentValueNegative)
      //            {
      //                shoulderRoll = -shoulderRoll;
      //            }
      currentMessageIndex = EndByte;
      break;
  }

  isCurrentValueNegative = false;
}


void AssignToJointAngle(char* readByte)
{
  // Assume not negative
  switch (currentMessageIndex)
  {
    case ShoulderYaw:
      shoulderYaw = shoulderYaw * 10;
      shoulderYaw = shoulderYaw + (*readByte - 48);
      break;

    case ShoulderPitch:
      shoulderPitch = shoulderPitch * 10;
      shoulderPitch = shoulderPitch + (*readByte - 48);
      break;

    case ShoulderRoll:
      shoulderRoll = shoulderRoll * 10;
      shoulderRoll = shoulderRoll + (*readByte - 48);
      break;

    case ElbowPitch:
      elbowAngle = elbowAngle * 10;
      elbowAngle = elbowAngle + (*readByte - 48);
      break;
  }
}

long GetTimeSinceLastRequest()
{
  return millis() - requestTimer;
}


void TestShoulderAndElbow(char* readByte)
{
  if (*readByte == startByte)
  {
    hasStartMessageBeenReceived = true;
    isCurrentValueNegative = false;
    shoulderPitch = 0;
    shoulderYaw = 0;
    shoulderRoll = 0;
    elbowAngle = 0;
    currentMessageIndex = StartByte;
  }
  else if (hasStartMessageBeenReceived)
  {
    if (*readByte == separator)
    {
      PrepareForNextJoint();
    }
    else if (*readByte == '-')
    {
      isCurrentValueNegative = true;
    }
    else if (*readByte == endByte)
    {
      // Move servos to position.
      SetShoulderYaw(&shoulderYaw);
      SetShoulderPitch(&shoulderPitch);
      SetShoulderRoll(&shoulderRoll);
      SetElbowPitch(&elbowAngle);
      
      RequestData();
      Serial.println();
    }
    else
    {
      AssignToJointAngle(readByte);
    }
  }
}


void TestElbowPitch(char* readByte)
{
  if  (*readByte == startByte)
  {
    hasStartMessageBeenReceived = true;
    elbowAngle = 0;
  }
  else if (*readByte == endByte)
  {
    hasStartMessageBeenReceived = false;
    SetElbowPitch(&elbowAngle);
    Serial.println(elbowAngle);
  }
  else if (hasStartMessageBeenReceived)
  {
    elbowAngle = elbowAngle * 10;
    elbowAngle = elbowAngle + (*readByte - 48);
  }
}


void TestShoulderPitch(char* readByte)
{
  Serial.print("|"); Serial.print(*readByte);
  if (*readByte == startByte)
  {
    hasStartMessageBeenReceived = true;
    shoulderPitch = 0;
  }
  else if (*readByte == endByte)
  {
    hasStartMessageBeenReceived = false;
    SetShoulderPitch(&shoulderPitch);
    Serial.println(shoulderPitch);
  }
  else if (hasStartMessageBeenReceived)
  {
    shoulderPitch = shoulderPitch * 10;
    shoulderPitch = shoulderPitch + (*readByte - 48);
  }
}

#endif
