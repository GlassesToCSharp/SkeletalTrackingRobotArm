#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H

#include "Constants.h"

bool hasStartMessageBeenReceived = false;
bool isCurrentValueNegative = false;
int elbowAngle = 0;
int shoulderPitch = 0;
int shoulderYaw = 0;
int shoulderRoll = 0;

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
void ProcessIncomingByte(const char* const readByte);

void RequestData()
{
  Serial.write(START_BYTE);
  Serial.write(READY_BYTE);
  Serial.write(END_BYTE);
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
    const char readByte = Serial.read();
    ProcessIncomingByte(&readByte);
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
      currentMessageIndex = ShoulderRoll;
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


void AssignToJointAngle(const char* readByte)
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


void ProcessIncomingByte(const char* const readByte)
{
  if (*readByte == START_BYTE)
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
    if (*readByte == SEPARATOR)
    {
      PrepareForNextJoint();
    }
    else if (*readByte == NEGATIVE_SYMBOL)
    {
      isCurrentValueNegative = true;
    }
    else if (*readByte == END_BYTE)
    {
      // Move servos to position.
      SetShoulderYaw(&shoulderYaw);
      SetShoulderPitch(&shoulderPitch);
      SetShoulderRoll(&shoulderRoll);
      SetElbowPitch(&elbowAngle);
    }
    else
    {
      AssignToJointAngle(readByte);
    }
  }
}

#endif
