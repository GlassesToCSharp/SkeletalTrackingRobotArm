#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <stdint.h>

uint16_t ConvertToServoPosition(const int16_t angle, const bool reverse = false);

void MoveToAngle(uint8_t const id, const int16_t inputAngle, const bool reverse)
{
    int16_t dynamixelAngle = ConvertToServoPosition(inputAngle, reverse);

    if (dynamixelAngle > SERVO_MAX_POSITION)
    {
        dynamixelAngle = SERVO_MAX_POSITION;
    }
    else if (dynamixelAngle < SERVO_MIN_POSITION)
    {
        dynamixelAngle = SERVO_MIN_POSITION;
    }

    Dynamixel.moveSpeed(id, dynamixelAngle, DEFAULT_MOTOR_SPEED);
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

//    Dynamixel.setAngleLimit(BROADCAST_ID, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
//    Dynamixel.setCMargin(BROADCAST_ID, 0, 0);
//    Dynamixel.setCSlope(BROADCAST_ID, 50, 50);
//    Dynamixel.setPunch(BROADCAST_ID, 0);
  
    // Set the initial position (no reverse)
    MoveToAngle(BROADCAST_ID, 0, false);
}

uint16_t ConvertToServoPosition(const int16_t angle, const bool reverse)
{
    uint16_t position = SERVO_RESET_POSITION;
    uint16_t servoAngle = angle / SERVO_RESOLUTION;
    if (reverse)
    {
        servoAngle = -servoAngle;
    }

    position += servoAngle;

    return position;
}
