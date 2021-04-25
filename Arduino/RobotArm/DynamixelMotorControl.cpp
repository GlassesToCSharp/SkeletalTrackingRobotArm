#include <Arduino.h>
#include "Constants.h"
#include "DynamixelMotorControl.h"
#include <DynamixelSerial.h>
#include <stdint.h>

uint16_t ConvertToServoPosition(const int16_t angle, const bool reverse = false);
int set_min_range(uint8_t ID, uint16_t min_range);
int set_max_range(uint8_t ID, uint16_t max_range);

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

    set_min_range(BROADCAST_ID, SERVO_MIN_POSITION);
    set_max_range(BROADCAST_ID, SERVO_MAX_POSITION);
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

int set_min_range(uint8_t ID, uint16_t min_range) {
  const uint8_t len = 5;
  const uint8_t reg = 6;
  uint8_t range_upper = (min_range >> 8) & 0xFF;
  uint8_t range_lower = min_range & 0xFF;
  uint8_t Checksum = (~(ID + len + AX_WRITE_DATA + reg + range_upper + range_lower)) & 0xFF;
  
  digitalWrite(DIRECTION_PIN, HIGH);
  
  Serial2.write(AX_START);                // Send Instructions over Serial
  Serial2.write(AX_START);
  Serial2.write(ID);
  Serial2.write(len);
  Serial2.write(AX_WRITE_DATA);
  Serial2.write(reg);
  Serial2.write(range_lower);
  Serial2.write(range_upper);
  Serial2.write(Checksum);
  Serial2.flush();
  
  digitalWrite(DIRECTION_PIN, LOW);

  return Dynamixel.read_error();
}

int set_max_range(uint8_t ID, uint16_t max_range) {
  const uint8_t len = 5;
  const uint8_t reg = 8;
  uint8_t range_upper = (max_range >> 8) & 0xFF;
  uint8_t range_lower = max_range & 0xFF;
  uint8_t Checksum = (~(ID + len + AX_WRITE_DATA + reg + range_upper + range_lower)) & 0xFF;
  
  digitalWrite(DIRECTION_PIN, HIGH);
  
  Serial2.write(AX_START);                // Send Instructions over Serial
  Serial2.write(AX_START);
  Serial2.write(ID);
  Serial2.write(len);
  Serial2.write(AX_WRITE_DATA);
  Serial2.write(reg);
  Serial2.write(range_lower);
  Serial2.write(range_upper);
  Serial2.write(Checksum);
  Serial2.flush();
  
  digitalWrite(DIRECTION_PIN, LOW);

  return Dynamixel.read_error();
}
