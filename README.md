# SkeletalTrackingRobotArm
**Objective**: Mimic user's arm motion with an Xbox Kinect V2

## Repository Contents
This repository should contain the code for the following:
1. C# code to read user's arm angles
2. Arduino code to communicate with the motors

## Hardware used
The following hardware was used:
1. Computer running Visual Studio (2015, but 2017 should be fine) and Microsoft Kinect SDK
2. Xbox Kinect **V2** (including adapter)
3. Arduino/Genuino UNO
4. Dynamixel RX-28 servo motors (x6)
5. Custom RS485 Comms board
6. Assorted cables/wires
7. A 3D-printed body (optional)

## Simplified code break-down
The code is split as follows:
### C# (Visual Studio)
The purpose of the C# code was to handle the harder and more computationally intensive algorithms. It is also required to talk to the Kinect and send the arm angles to the Arduino.

The code uses the vectors from various skeletal points. Joint angles are calculated using vector angles from joint-to-joint. The angles are then sent to the Arduino using standard serial protocol.

### Arduino
The Arduino is simply used to talk to the motors. There is no special algorithm going on, just serial data sent to the motors via the custom RS485 comms board.
