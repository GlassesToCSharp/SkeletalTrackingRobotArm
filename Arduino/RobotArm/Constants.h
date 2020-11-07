#ifndef CONSTANTS_H
#define CONSTANTS_H
//S,82,159,87,97,E
#define SHOULDER_YAW 1
#define ID1_MIN 100 // Shoulder pull back
#define ID1_MAX 600 // Shoulder push up
#define ID1_NAT 200

#define SHOULDER_PITCH 2
#define ID2_MIN 040 
#define ID2_MAX 340
#define ID2_NAT 040

#define SHOULDER_ROLL 3
#define ID3_MIN 212
#define ID3_MAX 812
#define ID3_NAT 512

#define ELBOW_PITCH 4
#define ID4_MIN 650
#define ID4_MAX 1023
#define ID4_NAT 1023

const char separator = ',';
const char startByte = 'S';
const char endByte = 'E';

const uint16_t motorSpeed = 100;

#endif
