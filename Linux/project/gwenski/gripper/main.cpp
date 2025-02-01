#include "GripperControl.h"
#include <stdio.h>

int main() {
    printf("\n===== Gripper Control test =====\n\n");

    // Initialize hardware
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
    if (!cm730.Connect()) {
        printf("Failed to connect CM-730!\n");
        return 0;
    }

    // Configure servos using library
    InitializeGripper(cm730);
    PositionRightArm(cm730);
    CenterGripperPosition(cm730);

    // Demo loop
    int count = 2;
    while (count-- > 0) {
        OpenGripper(cm730);
        RotateWristLeft(cm730);
        
        usleep(50000);

        CloseGripper(cm730);
        RotateWristRight(cm730);
      
        usleep(50000);
    }

	CenterWrist(cm730);
	CenterGripperPosition(cm730);
    DefaulPositionRightArm(cm730);
    return 0;
}
