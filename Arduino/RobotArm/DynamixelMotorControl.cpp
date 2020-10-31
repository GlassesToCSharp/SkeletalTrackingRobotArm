#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <SoftwareSerial.h>
#include <stdint.h>

#define SERIAL_RX 3
#define SERIAL_TX 4

SoftwareSerial mySerial(SERIAL_RX, SERIAL_TX);

void MoveToAngle(uint8_t id, uint16_t angle)
{
    Dynamixel.moveSpeed(id, angle, motorSpeed);
}

void ResetDynamixelSerial()
{
    if (Serial)
    {
        Serial.end();
    }
  
    Dynamixel.setSerial(&Serial);
    Dynamixel.begin(1000000, 4);

    while(!Serial);
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

void SetShoulderRoll(const int* shoulderRoll)
{
    uint16_t dynamixelPosition = map(*shoulderRoll, 0, 180, ID3_MIN, ID3_MAX);
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
