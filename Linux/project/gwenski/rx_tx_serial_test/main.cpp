#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

int main()
{
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
    if (cm730.Connect() == false)
    {
        printf("Fail to connect CM-730!\n");
        return 0;
    }

    Head::GetInstance()->Initialize();
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->MoveToHome(); 

    // cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 0, 0), 0);
    // sleep(1); // Wait for 1 second

    // cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 0, 255), 0);
    // sleep(1); // Wait for 1 second
    // cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 100, 0), 0);

    // cm730.Disconnect(); // Close connection

    return 0;
}
