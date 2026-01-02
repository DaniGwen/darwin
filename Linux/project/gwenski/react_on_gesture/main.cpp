#include <iostream>
#include <unistd.h>
#include <limits>
#include "HeadTracking.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH "../../../../Data/config.ini"

using namespace Robot;

int main(int argc, char *argv[])
{
    // --- 1. Cleanup and Setup ---
    std::cout << "Cleaning up stale processes..." << std::endl;
    system("pkill -9 python3");
    system("pkill -9 mjpg_streamer");
    usleep(100000);

    // --- 2. Mode Selection Prompt ---
    int vision_mode = 0;
    std::cout << "\n=== Darwin-OP Vision Selection ===\n";
    std::cout << "1: Object Detection\n";
    std::cout << "2: Face Detection\n";
    std::cout << "3: Gesture Detection\n";
    std::cout << "Select Mode (1-3): ";
    
    // Input validation
    while(!(std::cin >> vision_mode) || vision_mode < 1 || vision_mode > 3) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Please enter 1, 2, or 3: ";
    }

    // --- 3. Hardware Initialization ---
    std::cout << "\n[INFO] Initializing Hardware..." << std::endl;
    minIni *ini = new minIni(INI_FILE_PATH);
    
    // Initialize Camera (Fixes the 'select timeout' error)
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);

    if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
        std::cerr << "[ERROR] Fail to init Motion Manager!" << std::endl;
        return 0;
    }

    // --- 4. Initialize Head Tracking with Mode ---
    // This will start the correct Python script based on vision_mode
    if (!HeadTracking::GetInstance()->Initialize(ini, &cm730, vision_mode)) {
        std::cerr << "[ERROR] HeadTracking initialization failed!" << std::endl;
        return 0;
    }

    // --- 5. Start Robot Motion ---
    MotionManager::GetInstance()->AddModule((MotionModule*)HeadTracking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->SetEnable(true); // Enable torque
    
    // Stand up
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(9); // Stand Up
    while(Action::GetInstance()->IsRunning()) usleep(8000);

    // --- 6. Main Loop ---
    // The HeadTracking::Run() logic runs in the MotionManager thread/callback,
    // but we can monitor results here or handle high-level logic.
    
    std::cout << "[INFO] Main loop started. Press Ctrl+C to exit." << std::endl;
    
    int wave_counter = 0;
    
    while (true)
    {
        // Get the latest label from the HeadTracking module
        std::string label = HeadTracking::GetInstance()->GetDetectedLabel();

        if (label != "none" && label != "") {
            
            // Logic for Gesture Mode (Mode 3)
            if (vision_mode == 3 && label == "hand_wave") {
                wave_counter++;
                std::cout << "Wave Detected! count: " << wave_counter << std::endl;
                
                if (wave_counter >= 5) { // Threshold to prevent false positives
                    std::cout << ">>> PERFORMING WAVE ACTION <<<" << std::endl;
                    Action::GetInstance()->Start(15); // Wave action page
                    while(Action::GetInstance()->IsRunning()) usleep(8000);
                    wave_counter = 0;
                    Action::GetInstance()->Start(9); // Return to stand
                }
            }
            // Logic for Object/Face modes
            else {
                std::cout << "Detected: " << label << std::endl;
                wave_counter = 0;
            }
        } else {
            if (wave_counter > 0) wave_counter--;
        }

        usleep(50000); // 50ms sleep
    }

    return 0;
}