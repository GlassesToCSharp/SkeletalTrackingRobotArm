#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSoftSerial.h>
#include <stdint.h>

void MoveToAngle(uint8_t id, uint16_t angle)
{
    Dynamixel.moveSpeed(id, angle, motorSpeed);
}

void DynamixelInit()
{
    // Rx - 2, Tx - 3
//    Dynamixel.begin(1000000, 2, 3, 4); // 1000000 works for rx-64
    Dynamixel.begin(1000000, 2, 3, 4); 
    
//    Dynamixel.setTempLimit(254,80);         // Set Max Temperature to 80 Celcius
//    Dynamixel.setVoltageLimit(254,65,160);  // Set Operating Voltage from 6.5v to 16v
//    Dynamixel.setMaxTorque(254,512);        // 50% of Torque
//    Dynamixel.setSRL(254,2);                // Set the SRL to Return All
  
    // Set the initial position
    MoveToAngle(SHOULDER_YAW, 400);
    MoveToAngle(SHOULDER_PITCH, 250);
    MoveToAngle(SHOULDER_ROLL, 200);
    MoveToAngle(ELBOW_PITCH, 750);
}

void SetShoulderPitch(const int* shoulderPitch)
{
    uint16_t dynamixelPosition = map(*shoulderPitch, 90, 180, ID2_MIN, ID2_MAX);
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

void SetShoulderYaw(const int* shoulderYaw)
{
    uint16_t dynamixelPosition = map(*shoulderYaw, 70, 180, ID1_MIN, ID1_MAX);
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

void SetElbowPitch(const int* elbowPitch)
{
    uint16_t dynamixelPosition = map(*elbowPitch, 150, 0, ID4_MIN, ID4_MAX);
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


