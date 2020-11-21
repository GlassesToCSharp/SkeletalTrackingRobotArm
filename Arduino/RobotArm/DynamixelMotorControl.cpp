#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <stdint.h>

typedef struct {
//  uint8_t id;
  uint32_t lastServoAngle;
  uint32_t timeAngleSet;
} ServoConfiguration;

ServoConfiguration servoConfigurations[5];

void MoveToAngle(uint8_t id, uint16_t inputAngle, bool setTimerCount = true)
{
//    uint32_t timerCount = 0;
//    ServoConfiguration * currentConf = &servoConfigurations[id];
//    int16_t motorSpeed = defaultMotorSpeed;
//    char buff[100];
//
//    // If we have a time since last positioning, we need to calculate
//    // how fast to move the servo to the new position.
//    if (currentConf->timeAngleSet != 0)
//    {
//        uint32_t timeDiff = millis() - currentConf->timeAngleSet;
//        uint16_t angleDiff = abs(int16_t(currentConf->lastServoAngle - inputAngle));
//        // The RX-28 servos can reach speeds of 85rpm (at 18.5V, no load).
//        // Only 12V is supplied, so it is expected that this would make the
//        // servos slower. For the calculations, we'll use the given speed.
//        // 85rpm = 1.4rps = 510deg/s = 0.510deg/ms
//        // RX-28 operating range of 300 deg means to go from 0 to 300 should
//        // take 588ms. The value ranges for RX-28 speed control is 0-1023,
//        // which is the range expected for "angleDiff" (the angle parameter
//        // is already in the 0-1023 range). 
//        // Therefore, the fastest speed to go from 0 to 1023 is 1023/588=1.740
//        // (units per ms?). We can use the "map" function to determine the
//        // values in between.
//        uint16_t division = uint16_t((double(angleDiff) / timeDiff) * 1000);
//        motorSpeed = map(division, 0, 1740, 0, 1023);
////        sprintf(buff, "AngleDiff: %u, TimeDiff: %lu, Div: %lu\n", angleDiff, timeDiff, division);
////        Serial.print(buff);
////        sprintf(buff, "CurrentConf timet: %lu, angle: %u\n", currentConf->timeAngleSet, currentConf->lastServoAngle);
////        Serial.print(buff);
//        if (motorSpeed > 1023)
//        {
//            motorSpeed = 1023;
//        }
//        else if (motorSpeed < 0)
//        {
//            motorSpeed = 0;
//        }
//    }
//    
//    if (setTimerCount)
//    {
//        timerCount = millis();
//    }
//    
////    sprintf(buff, "About to set timerCount: %lu, motorSpeed: %u", timerCount, motorSpeed);
////    Serial.println(buff);
////    sprintf(buff, "SET \t ID: %d\tAngle: %u\tTime: %lu", id, inputAngle, timerCount);
////    Serial.println(buff);
//    
//    currentConf->lastServoAngle = inputAngle;
//    currentConf->timeAngleSet = timerCount;
    Dynamixel.moveSpeed(id, inputAngle, defaultMotorSpeed);
}

void ResetDynamixelSerial()
{
    Dynamixel.setSerial(&Serial2);
    Dynamixel.begin(1000000, 15);
}

void DynamixelInit()
{
    ResetDynamixelSerial();
    
//    Dynamixel.setTempLimit(254,80);         // Set Max Temperature to 80 Celcius
//    Dynamixel.setVoltageLimit(254,65,160);  // Set Operating Voltage from 6.5v to 16v
//    Dynamixel.setMaxTorque(254,512);        // 50% of Torque
//    Dynamixel.setSRL(254,2);                // Set the SRL to Return All
  
    // Set the initial position
    MoveToAngle(SHOULDER_YAW, ID1_NAT);
    MoveToAngle(SHOULDER_PITCH, ID2_NAT);
    MoveToAngle(SHOULDER_ROLL, ID3_NAT);
    MoveToAngle(ELBOW_PITCH, ID4_NAT);
}

void SetShoulderYaw(const int* shoulderYaw)
{
    int16_t dynamixelPosition = map(*shoulderYaw, 70, 180, ID1_MIN, ID1_MAX);
    if (dynamixelPosition > ID1_MAX)
    {
        dynamixelPosition = ID1_MAX;
    }
    else if (dynamixelPosition < ID1_MIN)
    {
        dynamixelPosition = ID1_MIN;
    }
    
//    char buff[30];
//    sprintf(buff, "Shoulder Yaw: %d | %d", *shoulderYaw, dynamixelPosition);
//    Serial.print(buff);
    MoveToAngle(SHOULDER_YAW, dynamixelPosition);
}

void SetShoulderPitch(const int* shoulderPitch)
{
    int16_t dynamixelPosition = map(*shoulderPitch, 90, 180, ID2_MIN, ID2_MAX);
    if (dynamixelPosition > ID2_MAX)
    {
        dynamixelPosition = ID2_MAX;
    }
    else if (dynamixelPosition < ID2_MIN)
    {
        dynamixelPosition = ID2_MIN;
    }
    
//    char buff[40];
//    sprintf(buff, "Shoulder Pitch: %d | %d", *shoulderPitch, dynamixelPosition);
//    Serial.print(buff);
    MoveToAngle(SHOULDER_PITCH, dynamixelPosition);
}

void SetShoulderRoll(const int* shoulderRoll)
{
    int16_t dynamixelPosition = map(*shoulderRoll, 0, 180, ID3_MIN, ID3_MAX);
    if (dynamixelPosition > ID3_MAX)
    {
        dynamixelPosition = ID3_MAX;
    }
    else if (dynamixelPosition < ID3_MIN)
    {
        dynamixelPosition = ID3_MIN;
    }
    
//    char buff[40];
//    sprintf(buff, "Shoulder Roll: %d | %d", *shoulderRoll, dynamixelPosition);
//    Serial.print(buff);
    MoveToAngle(SHOULDER_ROLL, dynamixelPosition);
}

void SetElbowPitch(const int* elbowPitch)
{
    int16_t dynamixelPosition = map(*elbowPitch, 150, 0, ID4_MIN, ID4_MAX);
    if (dynamixelPosition > ID4_MAX)
    {
        dynamixelPosition = ID4_MAX;
    }
    else if (dynamixelPosition < ID4_MIN)
    {
        dynamixelPosition = ID4_MIN;
    }
    
//    char buff[40];
//    sprintf(buff, "Elbow Pitch: %d | %d", *elbowPitch, dynamixelPosition);
//    Serial.print(buff);
    MoveToAngle(ELBOW_PITCH, dynamixelPosition);
}
