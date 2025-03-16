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
    CM730 cm730(CM730::DEFAULT_BAUDRATE);
    if (!cm730.Connect()) {
        std::cerr << "Failed to connect to CM-730." << std::endl;
        return -1;
    }
rx
    std::cout << "Turning GREEN LED ON" << std::endl;
    cm730.WriteByte(CM730::ID_CM, CM730::P_LED_PANEL, 0x04, nullptr); // 0x04 for Green LED
    sleep(1); // Wait for 1 second

    std::cout << "Turning GREEN LED OFF" << std::endl;
    cm730.WriteByte(CM730::ID_CM, CM730::P_LED_PANEL, 0x00, nullptr); // 0x00 to turn off
    
    cm730.Disconnect(); // Close connection
    return 0;
}
