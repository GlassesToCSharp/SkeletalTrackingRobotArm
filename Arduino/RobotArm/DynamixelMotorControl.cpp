#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <stdint.h>

void MoveToAngle(uint8_t id, uint16_t inputAngle, bool setTimerCount = true)
{
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

    Dynamixel.setCMargin(254, 0, 0);
    Dynamixel.setCSlope(254, 50, 50);
    Dynamixel.setPunch(254, 0);
  
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
    
    MoveToAngle(ELBOW_PITCH, dynamixelPosition);
}
