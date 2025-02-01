#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include "LinuxDARwIn.h"

using namespace Robot;

// Servo Position Limits
#define ID_21_MAX_CW_LIMIT 3100
#define ID_21_MAX_CCW_LIMIT 1070
#define ID_21_CENTER 1860
#define ID_22_MAX_CW_LIMIT 1450
#define ID_22_MAX_CCW_LIMIT 2270
#define ID_22_CENTER 1860

// Core Functions
void InitializeGripper(CM730 &cm730, int p_gain , int p_gain_gripper, bool complete_arm);
void OpenGripper(CM730 &cm730);
void CloseGripper(CM730 &cm730);
void CenterGripperPosition(CM730 &cm730);

// Wrist Control
void RotateWristRight(CM730 &cm730);
void RotateWristLeft(CM730 &cm730);
void CenterWrist(CM730 &cm730);

// Arm Control
void PositionRightArm(CM730 &cm730);
void DefaulPositionRightArm(CM730 &cm730);

// Utility
bool WaitWhileServoMoving(CM730 &cm730, int servo_id);
bool CheckRightArmIsEnabled();

#endif