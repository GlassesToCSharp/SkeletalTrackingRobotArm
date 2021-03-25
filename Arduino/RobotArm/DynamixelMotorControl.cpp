#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <stdint.h>

uint16_t ConvertToServoPosition(const int* angle, const bool reverse = false);

void MoveToAngle(const uint8_t id, int16_t inputAngle, bool setTimerCount = true)
{
    if (inputAngle > SERVO_MAX_POSITION)
    {
        inputAngle = SERVO_MAX_POSITION;
    }
    else if (inputAngle < SERVO_MIN_POSITION)
    {
        inputAngle = SERVO_MIN_POSITION;
    }
    
    Dynamixel.moveSpeed(id, inputAngle, DEFAULT_MOTOR_SPEED);
}

void ResetDynamixelSerial()
{
    Dynamixel.setSerial(&Serial2);
    Dynamixel.begin(1000000, DIRECTION_PIN);
}

void DynamixelInit()
{
    ResetDynamixelSerial();
    
//    Dynamixel.setTempLimit(254,80);         // Set Max Temperature to 80 Celcius
//    Dynamixel.setVoltageLimit(254,65,160);  // Set Operating Voltage from 6.5v to 16v
//    Dynamixel.setMaxTorque(254,512);        // 50% of Torque
//    Dynamixel.setSRL(254,2);                // Set the SRL to Return All

    Dynamixel.setCMargin(BROADCAST_ID, 0, 0);
    Dynamixel.setCSlope(BROADCAST_ID, 50, 50);
    Dynamixel.setPunch(BROADCAST_ID, 0);
  
    // Set the initial position
    MoveToAngle(BROADCAST_ID, SERVO_RESET_POSITION);
}

void SetShoulderYaw(const int* shoulderYaw)
{
    int16_t dynamixelPosition = ConvertToServoPosition(*shoulderYaw);    
    MoveToAngle(RIGHT_SHOULDER_YAW_ID, dynamixelPosition);
}

void SetShoulderPitch(const int* shoulderPitch)
{
    int16_t dynamixelPosition = ConvertToServoPosition(*shoulderPitch);
    MoveToAngle(RIGHT_SHOULDER_PITCH_ID, dynamixelPosition);
}

void SetShoulderRoll(const int* shoulderRoll)
{
    int16_t dynamixelPosition = ConvertToServoPosition(*shoulderRoll);
    MoveToAngle(RIGHT_SHOULDER_ROLL_ID, dynamixelPosition);
}

void SetElbowPitch(const int* elbowPitch)
{
    int16_t dynamixelPosition = ConvertToServoPosition(*elbowPitch);
    MoveToAngle(RIGHT_ELBOW_PITCH_ID, dynamixelPosition);
}

uint16_t ConvertToServoPosition(const int* angle, const bool reverse)
{
    uint16_t position = SERVO_RESET_POSITION;
    uint16_t servoAngle = *angle / SERVO_RESOLUTION;
    if (*angle > 0)
    {
        if (reverse)
        {
            position += servoAngle;
        }
        else
        {
            position -= servoAngle;
        }
    }
    else if (*angle < 0)
    {
        if (reverse)
        {
            position -= servoAngle;
        }
        else
        {
            position += servoAngle;
        }
    }

    return position;
}
