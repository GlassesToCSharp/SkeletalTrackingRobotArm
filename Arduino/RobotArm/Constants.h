#define SHOULDER_YAW 1
#define ID1_MIN 200 // Shoulder pull back
#define ID1_MAX 700 // Shoulder push up

#define SHOULDER_PITCH 2
#define ID2_MIN 250 
#define ID2_MAX 550

#define SHOULDER_ROLL 3
#define ID3_MIN 0   // Elbow contract
#define ID3_MAX 250 // Elbow extend

#define ELBOW_PITCH 4
#define ID4_MIN 500
#define ID4_MAX 900

const char separator = ',';
const char startByte = 'S';
const char endByte = 'E';

const uint16_t motorSpeed = 100;
