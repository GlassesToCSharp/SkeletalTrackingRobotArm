#ifndef DYNAMIXEL_MOTOR_CONTROL_H
#define DYNAMIXEL_MOTOR_CONTROL_H

extern void ResetDynamixelSerial();
extern void DynamixelInit();
extern void SetShoulderYaw(const int* shoulderYaw);
extern void SetShoulderPitch(const int* shoulderPitch);
extern void SetShoulderRoll(const int* shoulderRoll);
extern void SetElbowPitch(const int* elbowPitch);
extern void MoveToAngle(const uint8_t id, int16_t angle, bool setTimerCount = true);

#endif
