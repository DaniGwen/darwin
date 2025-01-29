#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"

using namespace Robot;

#define ID_21_MAX_CW_LIMIT 3100
#define ID_21_MAX_CCW_LIMIT 1070
#define ID_22_MAX_CW_LIMIT 1450
#define ID_22_MAX_CCW_LIMIT 2270

void OpenGripper(CM730 &cm730);
void CloseGripper(CM730 &cm730);
void DefaultGripperPosition(CM730 &cm730);
void RotateWristRight(CM730 &cm730);
void RotateWristLeft(CM730 &cm730);
bool IsServoMoving(CM730 &cm730, int servo_id);
void CenterWrist(CM730 &cm730);


int main()
{
	printf("\n===== Gripper control =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if (cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

	int value;
	cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_TORQUE_ENABLE, 1, 0);
	cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_TORQUE_ENABLE, 1, 0);
	cm730.WriteByte(JointData::ID_R_WRIST, MX28::P_P_GAIN, 4, 0);
	cm730.WriteByte(JointData::ID_R_GRIPPER, MX28::P_P_GAIN, 4, 0);

	printf("\rOpen gripper");
	DefaultGripperPosition(cm730);

	int count = 3;

	while (count > 0)
	{
		printf("\rOpen gripper");
		OpenGripper(cm730);
		RotateWristLeft(cm730);

		usleep(50000);

		printf("\rClose gripper");
		CloseGripper(cm730);
		RotateWristRight(cm730);
		CenterWrist(cm730);

		if (cm730.ReadWord(CM730::ID_CM, CM730::P_LED_HEAD_L, &value, 0) == CM730::SUCCESS)
		{
			if (value == 0x7FFF)
				value = 0;
			else
				value++;

			cm730.WriteWord(CM730::P_LED_HEAD_L, value, 0);
		}

		if (cm730.ReadWord(CM730::ID_CM, CM730::P_LED_EYE_L, &value, 0) == CM730::SUCCESS)
		{
			if (value == 0)
				value = 0x7FFF;
			else
				value--;

			cm730.WriteWord(CM730::P_LED_EYE_L, value, 0);
		}

		usleep(50000);
		count--;
	}

	return 0;
}

void OpenGripper(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_GRIPPER);
	cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, ID_22_MAX_CW_LIMIT, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_GRIPPER) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Gripper did not reach goal!\n");
	}
	else
	{
		printf("\nGripper fully opened.\n");
	}
}

void CloseGripper(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_GRIPPER);
	cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, ID_22_MAX_CCW_LIMIT, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_GRIPPER) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Gripper did not reach goal!\n");
	}
	else
	{
		printf("\nGripper fully closed.\n");
	}
}

void DefaultGripperPosition(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_GRIPPER);
	cm730.WriteWord(JointData::ID_R_GRIPPER, MX28::P_GOAL_POSITION_L, MX28::CENTER_VALUE, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_GRIPPER) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Gripper did not reach goal!\n");
	}
	else
	{
		printf("\nGripper is at default position.\n");
	}
}

void RotateWristRight(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_WRIST);
	cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, ID_21_MAX_CW_LIMIT, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_WRIST) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Wrist did not reach goal!\n");
	}
	else
	{
		printf("\nWrist rotated right.\n");
	}
}

void RotateWristLeft(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_WRIST);
	cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, ID_21_MAX_CCW_LIMIT, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_WRIST) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Wrist did not reach goal!\n");
	}
	else
	{
		printf("\nWrist rotated left.\n");
	}
}

void CenterWrist(CM730 &cm730)
{
	printf(" ID[%d]:", JointData::ID_R_WRIST);
	cm730.WriteWord(JointData::ID_R_WRIST, MX28::P_GOAL_POSITION_L, 2000, 0);

	int timeout = 100; // 1 second timeout
	while (IsServoMoving(cm730, JointData::ID_R_WRIST) && timeout-- > 0)
	{
		usleep(10000);
	}

	if (timeout <= 0)
	{
		printf("\nTimeout: Wrist did not reach goal!\n");
	}
	else
	{
		printf("\nWrist centered.\n");
	}
}

bool IsServoMoving(CM730 &cm730, int servo_id)
{
	int moving_status;
	if (cm730.ReadByte(servo_id, MX28::P_MOVING, &moving_status, 0) == CM730::SUCCESS)
	{
		return (moving_status == 1);
	}
	return false; // Error reading status
}