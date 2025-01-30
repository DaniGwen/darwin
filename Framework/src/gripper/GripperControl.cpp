#include "GripperControl.h"
#include <stdio.h>
#include <unistd.h>

// Initialize servo torque and gains
void InitializeGripper(CM730 &cm730)
{
    cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteByte(JointData::ID_R_WRIST, MX28::P_P_GAIN, 4, 0);

    cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteByte(JointData::ID_R_GRIPPER, MX28::P_P_GAIN, 4, 0);

    cm730.WriteByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_I_GAIN, 4, 0);

    cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL, MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL, MX28::P_P_GAIN, 4, 0);

    cm730.WriteByte(JointData::ID_R_ELBOW, MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteByte(JointData::ID_R_ELBOW, MX28::P_P_GAIN, 4, 0);
}

// Gripper Functions
void OpenGripper(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_GRIPPER);
    cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, ID_22_MAX_CW_LIMIT, 0);
    LinuxActionScript::PlayMP3("../../Data/mp3/open-door-sound.mp3");
    WaitWhileServoMoving(cm730, JointData::ID_R_GRIPPER);
}

void CloseGripper(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_GRIPPER);
    cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, ID_22_MAX_CCW_LIMIT, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_GRIPPER);
}

void CenterGripperPosition(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_GRIPPER);
    cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, ID_22_CENTER, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_GRIPPER);
}

// Wrist Functions
void RotateWristRight(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_WRIST);
    cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, ID_21_MAX_CW_LIMIT, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_WRIST);
}

void RotateWristLeft(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_WRIST);
    cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, ID_21_MAX_CCW_LIMIT, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_WRIST);
}

void CenterWrist(CM730 &cm730)
{
    printf(" ID[%d]:", JointData::ID_R_WRIST);
    cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, ID_21_CENTER, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_WRIST);
}

// Arm Positioning
void PositionRightArm(CM730 &cm730)
{
    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, 2400, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_SHOULDER_PITCH);

    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, 1850, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_SHOULDER_ROLL);

    cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, 2400, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_ELBOW);
}

// Utility Function
bool WaitWhileServoMoving(CM730 &cm730, int servo_id)
{
    int moving_status;
    int timeout = 100; // 100 * 10ms = 1 second timeout
    while (
        cm730.ReadByte(servo_id, MX28::P_MOVING, &moving_status, 0) == CM730::SUCCESS &&
        moving_status == 1 &&
        timeout-- > 0)
    {
        usleep(10000);
    }

    if (timeout <= 0)
    {
        printf("\nTimeout: Servo did not reach goal!\n");
        return false;
    }
    return true;
}