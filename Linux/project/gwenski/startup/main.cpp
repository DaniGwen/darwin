#include "GripperControl.h"
#include <stdio.h>

int main()
{
    printf("\n===== Start up =====\n\n");

    // Initialize hardware
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
    if (!cm730.Connect())
    {
          LinuxActionScript::PlayMP3("../../../../Data/mp3/girl-scream.mp3");
        printf("Failed to connect CM-730!\n");
        return 0;
    }

     LinuxActionScript::PlayMP3("../../../../Data/mp3/woman-ya.mp3");

    // Change Eyes color to red
    cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 255,255), 0);
 
    return 0;
}
