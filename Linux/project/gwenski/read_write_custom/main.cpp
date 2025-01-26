#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ctime>
#include "LinuxDARwIn.h"


using namespace Robot;

int main()
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

	int value;
    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

    cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 7, 0);
    cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 7, 0);
    cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 7, 0);
    
    cm730.WriteByte(JointData::ID_HEAD_PAN,          MX28::P_P_GAIN, 7, 0);
    cm730.WriteByte(JointData::ID_HEAD_TILT,         MX28::P_P_GAIN, 7, 0);
    
    int lastPlayedWow = std::time(0) - 5;
    int lastPlayedNo = std::time(0) - 5;
    int lastPlayedOops = std::time(0) - 5;

	while(1)
	{
		printf("\r");

		printf("GFB:");
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &value, 0) == CM730::SUCCESS)
			printf("%3d", value);
		else
			printf("---");

		printf(" GRL:");
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &value, 0) == CM730::SUCCESS)
			printf("%3d", value);
		else
			printf("---");

		printf(" AFB:");
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &value, 0) == CM730::SUCCESS)
			printf("%3d", value);
		else
			printf("----");

		printf(" ARL:");
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &value, 0) == CM730::SUCCESS)
			printf("%3d", value);
		else
			printf("----");

		printf(" BTN:");
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_BUTTON, &value, 0) == CM730::SUCCESS)
			printf("%1d", value);
		else
			printf("----");

		printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
		if(cm730.ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			int nowWow = std::time(0);
			int nowNo = std::time(0);
			
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
			cm730.WriteWord(JointData::ID_HEAD_TILT, MX28::P_GOAL_POSITION_L, value, 0); 
			
			if (value > 2400 && difftime(nowWow, lastPlayedWow) >= 7)
			{
				LinuxActionScript::PlayMP3("../../../../Data/mp3/Wow.mp3");
				lastPlayedWow = nowWow;
			}
			
			if (value < 1500 && difftime(nowNo, lastPlayedNo) >= 7)
			{
				LinuxActionScript::PlayMP3("../../../../Data/mp3/No.mp3");
				lastPlayedNo = nowNo;
			}
		}
		else
			printf("----");

		printf(" ID[%d]:", JointData::ID_R_SHOULDER_ROLL);
		if(cm730.ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			int now = std::time(0);
			
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
			cm730.WriteWord(JointData::ID_HEAD_PAN, MX28::P_GOAL_POSITION_L, value, 0); 
			
			if (value > 2500 && difftime(now, lastPlayedOops) >= 6)
			{
				LinuxActionScript::PlayMP3("../../../../Data/mp3/Oops.mp3");
				lastPlayedOops = now;
			}
		}
		else
			printf("----");

		printf(" ID[%d]:", JointData::ID_R_ELBOW);
		if(cm730.ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			printf("%4d", value);
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, MX28::GetMirrorValue(value), 0);
		}
		else
			printf("----");

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_LED_HEAD_L, &value, 0) == CM730::SUCCESS)
		{
			if(value == 0x7FFF)
				value = 0;
			else
				value++;

			cm730.WriteWord(CM730::P_LED_HEAD_L, value, 0);
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_LED_EYE_L, &value, 0) == CM730::SUCCESS)
		{
			if(value == 0)
				value = 0x7FFF;
			else
				value--;

			cm730.WriteWord(CM730::P_LED_EYE_L, value, 0);
		}
		
		usleep(50000);
	}

	return 0;
}
