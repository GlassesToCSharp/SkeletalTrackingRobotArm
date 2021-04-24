#ifndef DYNAMIXEL_MOTOR_CONTROL_H
#define DYNAMIXEL_MOTOR_CONTROL_H

extern void ResetDynamixelSerial();
extern void DynamixelInit();
extern void MoveToAngle(const uint8_t id, int16_t angle, const bool reverse = false);

#endif
