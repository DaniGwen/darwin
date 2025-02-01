#include "GripperControl.h"
#include <stdio.h>

int main()
{
    printf("\n===== Gripper Control test =====\n\n");

    // Initialize hardware
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
    if (!cm730.Connect())
    {
        printf("Failed to connect CM-730!\n");
        return 0;
    }

    InitializeGripper(cm730, 3, 5, true);
    PositionRightArm(cm730);
    CenterGripperPosition(cm730);

    int count = 2;
    while (count-- > 0)
    {
        OpenGripper(cm730);
        usleep(50000);
        RotateWristLeft(cm730);
        usleep(50000);
        CloseGripper(cm730);
        usleep(50000);
        RotateWristRight(cm730);
        usleep(50000);
    }

    CenterWrist(cm730);
    usleep(50000);
    CenterGripperPosition(cm730);
    usleep(50000);
    DefaulPositionRightArm(cm730);

    return 0;
}
