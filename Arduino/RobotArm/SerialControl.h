#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H

#include "Constants.h"
#include "DynamixelMotorControl.h"

bool hasStartMessageBeenReceived = false;

const uint8_t numberOfAngles = 8;
typedef struct JointAngle {
  uint8_t index;
  bool isNegative = false;
  int16_t angle = 0;
} JointAngle;
JointAngle jointAngles[numberOfAngles];

enum MessageIndex
{
  StartByte = 0,
  RightShoulderYaw,
  RightShoulderPitch,
  RightShoulderRoll,
  RightElbowPitch,
  LeftShoulderYaw,
  LeftShoulderPitch,
  LeftShoulderRoll,
  LeftElbowPitch,
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
    char readByte = Serial.read();
    ProcessIncomingByte(&readByte);
  }
}


MessageIndex GetNextMessageIndex()
{
  switch (currentMessageIndex)
  {
    case StartByte: return RightShoulderYaw;
    case RightShoulderYaw: return RightShoulderPitch;
    case RightShoulderPitch: return RightShoulderRoll;
    case RightShoulderRoll: return RightElbowPitch;
    case RightElbowPitch: return LeftShoulderYaw;
    case LeftShoulderYaw: return LeftShoulderPitch;
    case LeftShoulderPitch: return LeftShoulderRoll;
    case LeftShoulderRoll: return LeftElbowPitch;
    case LeftElbowPitch: return EndByte;
  }
}


uint8_t GetServoIdForIndex()
{
  switch (currentMessageIndex)
  {
    case RightShoulderYaw: return RIGHT_SHOULDER_YAW_ID;
    case RightShoulderPitch: return RIGHT_SHOULDER_PITCH_ID;
    case RightShoulderRoll: return RIGHT_SHOULDER_ROLL_ID;
    case RightElbowPitch: return RIGHT_ELBOW_PITCH_ID;
    case LeftShoulderYaw: return LEFT_SHOULDER_YAW_ID;
    case LeftShoulderPitch: return LEFT_SHOULDER_PITCH_ID;
    case LeftShoulderRoll: return LEFT_SHOULDER_ROLL_ID;
    case LeftElbowPitch: return LEFT_ELBOW_PITCH_ID;
  }
}


bool ReverseServo(const uint8_t servoId)
{
  switch (servoId)
  {
    case RIGHT_SHOULDER_YAW_ID: return false;
    case RIGHT_SHOULDER_PITCH_ID: return true;
    case RIGHT_SHOULDER_ROLL_ID: return false;
    case RIGHT_ELBOW_PITCH_ID: return false;
    case LEFT_SHOULDER_YAW_ID: return true;
    case LEFT_SHOULDER_PITCH_ID: return true;
    case LEFT_SHOULDER_ROLL_ID: return true;
    case LEFT_ELBOW_PITCH_ID: return false;
  }

  return false;
}


void ProcessIncomingByte(const char* const readByte)
{
  if (*readByte == START_BYTE)
  {
    hasStartMessageBeenReceived = true;
    currentMessageIndex = StartByte;
    return;
  }

  if (!hasStartMessageBeenReceived)
  {
    return;
  }

  switch (*readByte)
  {
    case SEPARATOR:
      currentMessageIndex = GetNextMessageIndex();
      jointAngles[currentMessageIndex - 1].index = GetServoIdForIndex();
      jointAngles[currentMessageIndex - 1].angle = 0;
      jointAngles[currentMessageIndex - 1].isNegative = false;
      break;

    case NEGATIVE_SYMBOL:
      jointAngles[currentMessageIndex - 1].isNegative = true;
      break;

    case END_BYTE:
      // Move servos to position.
      for(uint8_t i = 0; i < numberOfAngles; i++)
      {
        JointAngle jointAngle = jointAngles[i];
        int16_t angle = jointAngle.angle;
        if (jointAngle.isNegative)
        {
          angle = -angle;
        }
        MoveToAngle(jointAngle.index, angle, ReverseServo(jointAngle.index));
      }
      break;

    default:
      if (*readByte >= '0' && *readByte <= '9') {
        jointAngles[currentMessageIndex - 1].angle *= 10;
        jointAngles[currentMessageIndex - 1].angle += (*readByte - 48);
      }
      break;
  }
}

#endif
