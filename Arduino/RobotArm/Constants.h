#ifndef CONSTANTS_H
#define CONSTANTS_H

const uint8_t RIGHT_SHOULDER_YAW_ID = 1;
const uint8_t RIGHT_SHOULDER_PITCH_ID = 2;
const uint8_t RIGHT_SHOULDER_ROLL_ID = 3;
const uint8_t RIGHT_ELBOW_PITCH_ID = 4;

const uint8_t LEFT_SHOULDER_YAW_ID = 5;
const uint8_t LEFT_SHOULDER_PITCH_ID = 6;
const uint8_t LEFT_SHOULDER_ROLL_ID = 7;
const uint8_t LEFT_ELBOW_PITCH_ID = 8;

const uint8_t BROADCAST_ID = 254;

const uint16_t SERVO_RESET_POSITION = 512;
const uint16_t SERVO_MIN_POSITION = 0;
const uint16_t SERVO_MAX_POSITION = 1023;
const float SERVO_RESOLUTION = 0.29; // Degrees per step count

const char SEPARATOR = ',';
const char NEGATIVE_SYMBOL = '-';
const char START_BYTE = 'S';
const char END_BYTE = 'E';
const char READY_BYTE = 'R';

const uint16_t DEFAULT_MOTOR_SPEED = 100;
const uint8_t DIRECTION_PIN = 2;

#endif
